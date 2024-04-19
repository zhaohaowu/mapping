/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */

#include "system/localization_dead_reckoning.hpp"

#include <algorithm>
#include <memory>
#include <string>

#include "ad_time/ad_time.hpp"
#include "common/msf_common.hpp"
#include "common/transform_config.hpp"
#include "dead_reckoning/dr_locator_factory.hpp"
#include "eval/evaluator_localziation.hpp"
#include "eval/evaluator_motion.hpp"
#include "localization/common/log.hpp"
#include "system/localization_visualizer.hpp"

namespace senseAD {
namespace localization {

uint64_t LocalizationDeadReckoning::dr_origin_id_ = 0;

LocalizationDeadReckoning::~LocalizationDeadReckoning() {
  if (thread_ && thread_->joinable()) thread_->join();
}

adLocStatus_t LocalizationDeadReckoning::Init(const LocalizationParam& param) {
  param_ = param;

  // init DR locator
  if (InitLocator() != LOC_SUCCESS) {
    LC_LERROR(SYSTEM) << "Failed to init DR locator.";
    return LOC_LOCALIZATION_ERROR;
  }

  // init DR evaluator
  if (param_.ci_param.enable_evaluation) {
    if (InitLocEvaluator() != LOC_SUCCESS) {
      LC_LERROR(SYSTEM) << "Failed to init DR evaluator.";
      return LOC_LOCALIZATION_ERROR;
    }
  }

  // init data buffer
  if (OnInitDataBuffer() != LOC_SUCCESS) {
    LC_LERROR(SYSTEM) << "Failed to init DR data buffer.";
    return LOC_LOCALIZATION_ERROR;
  }

  // init thread
  thread_.reset(new std::thread(&LocalizationDeadReckoning::Run, this));
  if (nullptr == thread_) {
    LC_LERROR(SYSTEM) << "Failed to create DR thread.";
    return LOC_ALLOC_MEMORY_FAILED;
  }
  SetThreadName();

  return LOC_SUCCESS;
}

adLocStatus_t LocalizationDeadReckoning::Restart() {
  // reset variable
  ++dr_origin_id_;
  last_imu_timestamp_ = 0;
  last_can_timestamp_ = 0;
  need_restart_ = false;
  last_get_timestamp_ = 0;

  // clear buffer datas
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_data_list_.clear();
  }
  {
    std::lock_guard<std::mutex> lock(window_odom_mutex_);
    window_odom_states_.clear();
  }

  // reinit for evaluator related
  if (param_.ci_param.enable_evaluation) {
    std::string tmp = "_" + std::to_string(dr_origin_id_);
    std::string setting_type = "WIO" + tmp;
    dr_evaluator_->Init(param_.ci_param.results_save_dir, setting_type,
                        param_.ci_param.testcase_id);
  }

  // restart locator
  if (InitLocator() != LOC_SUCCESS) {
    LC_LERROR(SYSTEM) << "Failed to restart dr locator.";
    return LOC_LOCALIZATION_ERROR;
  }

  return LOC_SUCCESS;
}

bool LocalizationDeadReckoning::CheckNeedRestart() const {
  return need_restart_;
}

void LocalizationDeadReckoning::SetThreadName() {
  pthread_setname_np(thread_->native_handle(), "LC_dr");
}

void LocalizationDeadReckoning::SetLocalizationVisualizer(
    std::shared_ptr<LocalizationVisualizer> visualizer) {
  loc_visualizer_ = visualizer;
}

void LocalizationDeadReckoning::SetCanData(uint64_t timestamp,
                                           const VehicleInfo& can_data) {
  last_can_timestamp_ = timestamp;
  dr_locator_->SetCanData(timestamp, can_data);
}

void LocalizationDeadReckoning::SetImuData(uint64_t timestamp,
                                           const Imu& imu_data) {
  std::lock_guard<std::mutex> lock(imu_mutex_);
  imu_data_list_.push_back(std::make_pair(timestamp, imu_data));
}

adLocStatus_t LocalizationDeadReckoning::GetLatestState(
    OdomState* odom_state) const {
  if (nullptr == odom_state) {
    LC_LERROR(SYSTEM) << "nullptr";
    return LOC_NULL_PTR;
  }
  std::unique_lock<std::mutex> locker(window_odom_mutex_);
  if (window_odom_states_.empty()) {
    LC_LINFO_EVERY(SYSTEM, 20) << "window odom states is empty!";
    return LOC_LOCALIZATION_ERROR;
  }
  *odom_state = window_odom_states_.back();
  return LOC_SUCCESS;
}

adLocStatus_t LocalizationDeadReckoning::GetOdomStateInfo(
    OdomStateInfo* info) const {
  OdomState cur_state;
  std::unique_lock<std::mutex> locker(window_odom_mutex_);
  if (window_odom_states_.empty()) {
    LC_LDEBUG_EVERY(SYSTEM, 20) << "window odom states is empty!";
    return LOC_LOCALIZATION_ERROR;
  }
  const auto& back_state = window_odom_states_.back();
  if (last_get_timestamp_ == 0 ||
      (back_state.timestamp * 1e-9 - last_get_timestamp_ * 1e-9) > 0.1) {
    cur_state = back_state;  // get the latest state
  } else {
    auto iter = window_odom_states_.rbegin();
    for (; iter != window_odom_states_.rend(); ++iter) {
      if (iter->timestamp <= last_get_timestamp_) break;
    }
    if (iter != window_odom_states_.rbegin()) --iter;
    cur_state = *iter;  // get the first state not published
  }
  locker.unlock();
  last_get_timestamp_ = cur_state.timestamp;

  // convert odom state info
  return ConvertOdomStateInfo(cur_state, info);
}

adLocStatus_t LocalizationDeadReckoning::QueryOdomPoseByTime(
    uint64_t timestamp, OdomState* odom_state) const {
  if (nullptr == odom_state) return LOC_NULL_PTR;
  std::lock_guard<std::mutex> guard(window_odom_mutex_);
  // time check
  if (window_odom_states_.size() < 2) {
    LC_LDEBUG(SYSTEM) << "not enough valid pose window";
    return LOC_INVALID;
  }
  if (window_odom_states_.front().timestamp > timestamp) {
    LC_LDEBUG(SYSTEM) << "query time delay window too much";
    return LOC_TIME_DELAY;
  }
  // at most 100ms ahead
  static constexpr uint64_t ahead_gap = 1e8;
  if (window_odom_states_.back().timestamp + ahead_gap < timestamp) {
    LC_LDEBUG(SYSTEM) << "query time ahead window too much";
    return LOC_TIME_AHEAD;
  }

  if (window_odom_states_.back().timestamp >= timestamp) {
    // interpolate
    auto iter = window_odom_states_.rbegin();
    while (iter != window_odom_states_.rend() && iter->timestamp > timestamp) {
      ++iter;
    }

    OdomState last_state = *iter;
    if (last_state.timestamp == timestamp) {
      *odom_state = last_state;
      return LOC_SUCCESS;
    }
    OdomState next_state = *(--iter);
    if (next_state.timestamp == timestamp) {
      *odom_state = next_state;
      return LOC_SUCCESS;
    }
    double factor = 1.0 * (timestamp - last_state.timestamp) /
                    (next_state.timestamp - last_state.timestamp);
    odom_state->timestamp = timestamp;
    OdomStateInterp(last_state, next_state, factor, odom_state);
  } else {
    // extrapolate
    static constexpr size_t min_extrapolate_diff = 10;
    if (window_odom_states_.size() < min_extrapolate_diff) {
      LC_LDEBUG(SYSTEM)
          << "query time ahead, but not enough pose to extrapolate";
      return LOC_TIME_AHEAD;
    }

    OdomState latest_state, last_state, ahead_state;
    size_t ind = 0;
    auto iter = window_odom_states_.rbegin();
    for (; iter != window_odom_states_.rend(); ++iter, ++ind) {
      if (0 == ind) {
        latest_state = *iter;
      }
      if (min_extrapolate_diff - 1 == ind) {
        last_state = *iter;
        break;
      }
    }
    ahead_state = latest_state;
    ahead_state.timestamp = latest_state.timestamp +
                            (latest_state.timestamp - last_state.timestamp);
    ahead_state.pose =
        latest_state.pose * (last_state.pose.inverse() * latest_state.pose);

    double factor = 1.0 * (timestamp - latest_state.timestamp) /
                    (ahead_state.timestamp - latest_state.timestamp);
    odom_state->timestamp = timestamp;
    OdomStateInterp(latest_state, ahead_state, factor, odom_state);
  }

  return LOC_SUCCESS;
}

void LocalizationDeadReckoning::Run() {
  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    // check finish request
    if (CheckFinishRequest()) break;

    // check pause request
    if (CheckPauseRequest()) {
      SetPause();
      while (IsPaused()) WaitReleasePaused(1000);
    }

    // DR process core
    DRProcess();
  }
  SetFinish();
}

void LocalizationDeadReckoning::DRProcess() {
  bool continue_to_process = true;
  while (continue_to_process) {
    // odom locator process for once synced imu and
    OdomState odom_state;
    if (LOC_SUCCESS != DRLocatorProcess(&odom_state, &continue_to_process)) {
      break;
    }

    // store odom state into window states
    const auto& s_odom_state = StoreOdomState(odom_state);

    // for evaluate localization output
    if (dr_evaluator_ && s_odom_state.timestamp != 0) {
      dr_evaluator_->WriteResult(s_odom_state.timestamp * kNanoSecToSec,
                                 s_odom_state.pose.matrix());
    }
    if (dr_motion_evaluator_ && s_odom_state.timestamp != 0) {
      MotionEvalData motion_data;
      motion_data.linear_speed = s_odom_state.linear_speed;
      motion_data.linear_acceleration = s_odom_state.linear_acceleration;
      motion_data.angular_speed = s_odom_state.angular_speed;
      dr_motion_evaluator_->WriteResult(s_odom_state.timestamp * kNanoSecToSec,
                                        motion_data);
    }

    // for visualizer
    if (loc_visualizer_ && s_odom_state.timestamp != 0) {
      OdomStateInfo odom_state_info;
      ConvertOdomStateInfo(s_odom_state, &odom_state_info);
      loc_visualizer_->SetDRInfo(odom_state_info);
    }
  }
}

adLocStatus_t LocalizationDeadReckoning::InitLocator() {
  dr_locator_ = dr::DRLocatorFactory::CreateDRLocator(
      param_.dr_param.dead_reckoning_locator_type);
  if (nullptr == dr_locator_) {
    LC_LERROR(SYSTEM) << "Failed to create DR locator.";
    return LOC_ALLOC_MEMORY_FAILED;
  }
  if (dr_locator_->Init(param_) != LOC_SUCCESS) {
    LC_LERROR(SYSTEM) << "Failed to init DR locator.";
    return LOC_LOCALIZATION_ERROR;
  }
  return LOC_SUCCESS;
}

adLocStatus_t LocalizationDeadReckoning::InitLocEvaluator() {
  // create and init localization evaluator
  std::string setting_type = "WIO";
  dr_evaluator_ = std::make_shared<EvaluatorLocalization>();
  if (nullptr == dr_evaluator_) {
    LC_LERROR(SYSTEM) << "Failed to create DR evaluator.";
    return LOC_ALLOC_MEMORY_FAILED;
  }
  if (dr_evaluator_->Init(param_.ci_param.results_save_dir, setting_type,
                          param_.ci_param.testcase_id) != LOC_SUCCESS) {
    LC_LERROR(SYSTEM) << "Failed to init DR evaluator.";
    return LOC_LOCALIZATION_ERROR;
  }

  // create and init motion evaluator
  setting_type = "WIO_MOTION";
  dr_motion_evaluator_ = std::make_shared<EvaluatorMotion>();
  if (nullptr == dr_motion_evaluator_) {
    LC_LERROR(SYSTEM) << "Failed to create DR motion evaluator.";
    return LOC_ALLOC_MEMORY_FAILED;
  }
  if (dr_motion_evaluator_->Init(param_.ci_param.results_save_dir, setting_type,
                                 param_.ci_param.testcase_id) != LOC_SUCCESS) {
    LC_LERROR(SYSTEM) << "Failed to init DR motion evaluator.";
    return LOC_LOCALIZATION_ERROR;
  }
  return LOC_SUCCESS;
}

adLocStatus_t LocalizationDeadReckoning::OnInitDataBuffer() {
  {
    std::lock_guard<std::mutex> lock(window_odom_mutex_);
    window_odom_states_.set_capacity(param_.common_param.window_size);
  }
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_data_list_.set_capacity(kDataSizeUpperLimit);
  }
  return LOC_SUCCESS;
}

adLocStatus_t LocalizationDeadReckoning::DRLocatorProcess(
    OdomState* odom_state, bool* continue_to_process) {
  if (nullptr == odom_state || nullptr == continue_to_process) {
    LC_LERROR(SYSTEM) << "nullptr";
    return LOC_NULL_PTR;
  }

  uint64_t imu_time = 0;
  static auto imu_data = std::make_shared<Imu>();
  {
    std::lock_guard<std::mutex> guard(imu_mutex_);
    if (!imu_data_list_.empty()) {
      *imu_data = imu_data_list_.front().second;
      imu_time = imu_data_list_.front().first;
      imu_data_list_.erase(imu_data_list_.begin());
      (*continue_to_process) = !imu_data_list_.empty();
    } else {
      (*continue_to_process) = false;
      LC_LDEBUG(SYSTEM) << "dr processed imu data is empty.";
      return LOC_LOCALIZATION_ERROR;
    }
  }

  LC_LDEBUG_EVERY(SYSTEM, 20)
      << "DR process: " << imu_time << "/" << Time::ToString(imu_time);

  if (LOC_SUCCESS != dr_locator_->Process(imu_time, imu_data)) {
    LC_LDEBUG(SYSTEM) << "odom locator process failed";
    return LOC_LOCALIZATION_ERROR;
  }
  dr_locator_->GetState(odom_state);
  odom_state->odom_status = GetOdomStatusNum(GetCurrentOdomStatus(*odom_state));

  return LOC_SUCCESS;
}

OdomState LocalizationDeadReckoning::StoreOdomState(
    const OdomState& newest_odom_state) {
  // store history position information
  std::lock_guard<std::mutex> wind_lock(window_odom_mutex_);

  static constexpr uint64_t window_gap = 1e7;
  // Compensate the lost state
  if (window_odom_states_.empty()) {
    auto odom_state = newest_odom_state;
    odom_state.timestamp = static_cast<uint64_t>(std::round(
                               odom_state.timestamp * 1.0 / window_gap)) *
                           window_gap;
    odom_state.origin_id = dr_origin_id_;
    window_odom_states_.push_back(odom_state);
    return odom_state;
  }

  uint64_t last_standard_time = window_odom_states_.back().timestamp;
  if (newest_odom_state.timestamp <= last_standard_time) {
    return OdomState();  // default value, timestamp is 0
  }

  double time_gap =
      (static_cast<double>(newest_odom_state.timestamp) - last_standard_time) /
      static_cast<double>(window_gap);

  if (time_gap >= 100) {
    LC_LERROR_EVERY_SEC(SYSTEM, 1)
        << "odom state timegap larger than 1 sec, timestamp jump possible!";
    window_odom_states_.clear();
    auto odom_state = newest_odom_state;
    odom_state.timestamp = static_cast<uint64_t>(std::round(
                               odom_state.timestamp * 1.0 / window_gap)) *
                           window_gap;
    odom_state.origin_id = dr_origin_id_;
    window_odom_states_.push_back(odom_state);
    return odom_state;
  }

  int interp_num = static_cast<int>(std::round(time_gap));
  if (interp_num >= 1) {
    std::list<OdomState> interp_odom_states;
    const OdomState& s_os = window_odom_states_.back();
    const OdomState& e_os = newest_odom_state;
    for (int i = 1; i <= interp_num; ++i) {
      OdomState o_state;
      uint64_t c_time = last_standard_time + window_gap * i;
      OdomStateInterp(s_os, e_os, i / time_gap, &o_state);
      o_state.timestamp = c_time;
      o_state.state_source = e_os.state_source;
      o_state.origin_id = dr_origin_id_;
      interp_odom_states.push_back(o_state);
    }
    window_odom_states_.insert(window_odom_states_.end(),
                               interp_odom_states.begin(),
                               interp_odom_states.end());
    return window_odom_states_.back();
  }

  return OdomState();  // default value, timestamp is 0
}

OdomStatus LocalizationDeadReckoning::GetCurrentOdomStatus(
    const OdomState& odom_state) {
  if (param_.common_param.enable_dr_restart && need_restart_) {
    return OdomStatus::FAILED;
  }

  // check imu timestamp continuity
  double timegap = 0.0;
  if (last_imu_timestamp_ != 0) {
    timegap = odom_state.timestamp * 1e-9 - last_imu_timestamp_ * 1e-9;
  }
  last_imu_timestamp_ = odom_state.timestamp;
  if (timegap > 1.0) {  // unit: sec
    need_restart_ = true;
    LC_LERROR(SYSTEM) << "FAILED STATUS, reason: imu has large timegap: "
                      << timegap;
    return OdomStatus::FAILED;
  }

  // check can timestamp continuity
  timegap = 0.0;
  if (last_can_timestamp_ != 0) {
    timegap = odom_state.timestamp * 1e-9 - last_can_timestamp_ * 1e-9;
  }
  if (timegap > 2.0) {  // unit: sec
    need_restart_ = true;
    LC_LERROR(SYSTEM) << "FAILED STATUS, reason: can has large timegap: "
                      << timegap;
    return OdomStatus::FAILED;
  }

  // check fusion consistency
  auto odom_status = dr_locator_->GetCurrentLocatorStatus();
  if (odom_status != OdomStatus::GOOD) {
    need_restart_ = true;
    LC_LERROR(SYSTEM) << "FAILED STATUS, reason: locator status failed.";
    return OdomStatus::FAILED;
  }

  // check altitude and lateral velocity
  Eigen::Vector3d velocity = odom_state.linear_speed;
  if (std::fabs(velocity(1)) > 6.0 || std::fabs(velocity(2)) > 6.0) {
    need_restart_ = true;
    LC_LERROR(SYSTEM)
        << "FAILED STATUS, reason: altitude or lateral has large velocity: "
        << velocity(1) << ", " << velocity(2);
    return OdomStatus::FAILED;
  }

  return OdomStatus::GOOD;
}

void LocalizationDeadReckoning::OdomStateInterp(const OdomState& s_ns,
                                                const OdomState& e_ns,
                                                double factor,
                                                OdomState* ns) const {
  ns->odom_status = e_ns.odom_status;
  ns->origin_id = e_ns.origin_id;
  // interpolate for pose, velocity etc
  ns->pose = Utility::SE3interpolate(s_ns.pose, e_ns.pose, factor);
  ns->linear_speed =
      s_ns.linear_speed + (e_ns.linear_speed - s_ns.linear_speed) * factor;
  ns->linear_acceleration =
      s_ns.linear_acceleration +
      (e_ns.linear_acceleration - s_ns.linear_acceleration) * factor;
  ns->angular_speed =
      s_ns.angular_speed + (e_ns.angular_speed - s_ns.angular_speed) * factor;
  // TODO(fy) How about covariance ?
  ns->pose_cov = e_ns.pose_cov;
}

adLocStatus_t LocalizationDeadReckoning::ConvertOdomStateInfo(
    const OdomState& odom_state, OdomStateInfo* info) const {
  if (nullptr == info) {
    LC_LERROR(SYSTEM) << "info nullptr";
    return LOC_NULL_PTR;
  }

  // set timestamp
  info->measurement_time_ns = odom_state.timestamp;

  // set fusion status
  info->odom_status = GetOdomStatusEnum(odom_state.odom_status);

  // set origin id
  info->origin_id = odom_state.origin_id;

  // Fill in the latest odometry information
  // Local horizontal position
  Eigen::Vector3d t = odom_state.pose.translation();
  info->position.x = t(0);
  info->position.y = t(1);
  info->position.z = t(2);

  // attitude
  Eigen::Vector3d ypr =
      odom_state.pose.so3().unit_quaternion().toRotationMatrix().eulerAngles(
          2, 1, 0);
  info->euler_angle.roll = ypr(2);
  info->euler_angle.pitch = ypr(1);
  info->euler_angle.yaw = ypr(0);

  // velocity, acceleration, angular speed

  // local vehicle frame velocity
  info->linear_velocity = {odom_state.linear_speed(0),
                           odom_state.linear_speed(1),
                           odom_state.linear_speed(2)};
  // local vehicle frame acceleration
  info->linear_acceleration = {odom_state.linear_acceleration(0),
                               odom_state.linear_acceleration(1),
                               odom_state.linear_acceleration(2)};
  // local vehicle frame angular velocity
  info->angular_velocity = {odom_state.angular_speed(0),
                            odom_state.angular_speed(1),
                            odom_state.angular_speed(2)};
  // position uncertainty
  info->position_std.x = std::sqrt(odom_state.pose_cov(0, 0));
  info->position_std.y = std::sqrt(odom_state.pose_cov(1, 1));
  info->position_std.z = std::sqrt(odom_state.pose_cov(2, 2));
  // and attitude uncertainty
  info->attitude_std.x = std::sqrt(odom_state.pose_cov(3, 3));
  info->attitude_std.y = std::sqrt(odom_state.pose_cov(4, 4));
  info->attitude_std.z = std::sqrt(odom_state.pose_cov(5, 5));

  return LOC_SUCCESS;
}

}  // namespace localization
}  // namespace senseAD
