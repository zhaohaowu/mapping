/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */

#include "system/frontend/localization_frontend_imu.hpp"

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "ad_time/ad_time.hpp"
#include "common/coordinate_converter.hpp"
#include "eval/evaluator_gnss_status.hpp"
#include "eval/evaluator_localization_status.hpp"
#include "eval/evaluator_localziation.hpp"
#include "eval/evaluator_static.hpp"
#include "imu/hybrid_static_detect.hpp"
#include "ins/ins_locator.hpp"
#include "localization/common/log.hpp"
#include "msf/msf_factor/msf_factor.hpp"
#include "msf/msf_fusion/msf_fusion.hpp"
#include "system/localization_backend.hpp"

namespace senseAD {
namespace localization {

void LocalizationFrontendIMU::SetInsData(uint64_t timestamp, const Ins& ins) {
  std::lock_guard<std::mutex> lock(ins_mutex_);
  ins_data_list_.push_back(std::make_pair(timestamp, ins));
}

void LocalizationFrontendIMU::SetCanData(uint64_t timestamp,
                                         const VehicleInfo& can) {
  static_detector_->SetCanData(timestamp, can);
}

bool LocalizationFrontendIMU::CheckFrontendNeedRestart() {
  if (NavStatus::FATAL_ACCURACY == latest_nav_status_) {
    fatal_accuracy_cnt_++;
  } else {
    fatal_accuracy_cnt_ = 0;
  }
  // CheckNeedRestart's freq is 10Hz
  // if FATAL_ACCURACY exceeds 20s, restart
  if (fatal_accuracy_cnt_ > 200) {
    fatal_accuracy_cnt_ = 0;
    LC_LERROR(SYSTEM) << "navstatus is continuously fatal";
    return true;
  }

  return false;
}

adLocStatus_t LocalizationFrontendIMU::InitLocator() {
  // init locator
  locator_ = std::make_shared<IMULocator>();
  if (nullptr == locator_) {
    LC_LERROR(SYSTEM) << "Failed to create IMU locator.";
    return LOC_ALLOC_MEMORY_FAILED;
  }
  if (locator_->Init(param_) != LOC_SUCCESS) {
    LC_LERROR(SYSTEM) << "Failed to init IMU locator.";
    return LOC_LOCALIZATION_ERROR;
  }

  // init static detector
  size_t imu_freq = param_.common_param.ins_device == "CHNAV" ? 100 : 125;
  static_detector_ = std::make_shared<HybridStaticDetector>(imu_freq);
  if (nullptr == static_detector_) {
    LC_LERROR(SYSTEM) << "failed to create StaticDetector";
    return LOC_LOCALIZATION_ERROR;
  }
  if (LOC_SUCCESS != static_detector_->Init()) {
    LC_LERROR(SYSTEM) << "failed to init StaticDetector";
    return LOC_LOCALIZATION_ERROR;
  }
  // init static evaluator
  if (param_.ci_param.enable_evaluation) {
    static_evaluator_.reset(new EvaluatorStatic);
    static_evaluator_->Init(param_.ci_param.results_save_dir, "STATIC",
                            param_.ci_param.testcase_id);
  }

  return LOC_SUCCESS;
}

adLocStatus_t LocalizationFrontendIMU::InitInitializer() {
  initializer_ = std::make_shared<Initialization>();
  if (nullptr == initializer_) {
    LC_LERROR(SYSTEM) << "Failed to create IMU Initializer";
    return LOC_LOCALIZATION_ERROR;
  }
  if (initializer_->Init(param_) != LOC_SUCCESS) {
    LC_LERROR(SYSTEM) << "Failed to init IMU Initializer.";
    return LOC_LOCALIZATION_ERROR;
  }
  initializer_->SetFrontendLocator(locator_);

  if (param_.common_param.sub_ins_data) {
    ins_init_locator_ = std::make_shared<INSLocator>();
    if (nullptr == ins_init_locator_) {
      LC_LERROR(SYSTEM) << "Failed to create IMU init locator";
      return LOC_ALLOC_MEMORY_FAILED;
    }
    if (ins_init_locator_->Init(param_) != LOC_SUCCESS) {
      LC_LERROR(SYSTEM) << "Failed to init IMU init locator";
      return LOC_LOCALIZATION_ERROR;
    }
  }
  return LOC_SUCCESS;
}

adLocStatus_t LocalizationFrontendIMU::InitDataBuffer() {
  InitCapacity();
  if (param_.common_param.sub_ins_data) {
    std::lock_guard<std::mutex> loc(ins_mutex_);
    ins_data_list_.set_capacity(kDataSizeUpperLimit);
  }
  return LOC_SUCCESS;
}

void LocalizationFrontendIMU::SetThreadName() {
  pthread_setname_np(thread_->native_handle(), "LC_front_imu");
}

adLocStatus_t LocalizationFrontendIMU::ResetSubData() {
  last_init_state_time_ = 0;
  update_fix_state_ = true;
  last_zupt_timestamp_ = 0;
  ClearData();
  if (param_.common_param.sub_ins_data) {
    std::lock_guard<std::mutex> loc(ins_mutex_);
    ins_data_list_.clear();
  }
  for (auto& item : status_counter_) item.second = 0;

  return LOC_SUCCESS;
}

adLocStatus_t LocalizationFrontendIMU::SwitchOriginSubProc() {
  // convert enu position to new origin
  CoordinateConverter::GetInstance()->ConvertENUToNewOrigin(
      fixed_nav_state_.pose.translation(),
      &fixed_nav_state_.pose.translation());
  return LOC_SUCCESS;
}

InitStage LocalizationFrontendIMU::Initialize() {
  if (initializer_->GetInitStage() != InitStage::INIT_DONE) {
    initializer_->Initialize();
    if (param_.common_param.sub_ins_data) {
      InsAidedFastInitialization();
    } else {
      FastInitialization();
    }
  }
  return initializer_->GetInitStage();
}

NavStatus LocalizationFrontendIMU::GetCurrentNavStatus(
    const NavState& nav_state) {
  std::vector<NavStatus> total_status;

  // check1: frontend quality (frame rate, data accuracy ...)
  std::string frontend_reason = "";
  total_status.emplace_back(CheckFrontendStatus(&frontend_reason));

  // check2: backend observation quality (confidence ...)
  std::string backend_reason = "";
  total_status.emplace_back(CheckBackendStatus(&backend_reason));

  // check3: MSF fusion quality (covariance, error state ...)
  std::string msf_reason = "";
  total_status.emplace_back(CheckMSFStatus(nav_state, &msf_reason));

  // check4: output state quality (smoothness ...)
  std::string output_reason = "";
  total_status.emplace_back(CheckOutputStatus(&output_reason));

  // help function: check whether all checks satisfy "thre" level
  static auto check_status = [](const std::vector<NavStatus>& total_status,
                                NavStatus thre) {
    bool check = true;
    for (const auto& stat : total_status) check = check && (stat >= thre);
    return check;
  };

  // consider above check results, and output localization status
  std::string cur_reason =
      frontend_reason + backend_reason + msf_reason + output_reason;
  NavStatus cur_status;
  if (check_status(total_status, NavStatus::HIGH_ACCURACY)) {
    cur_status = NavStatus::HIGH_ACCURACY;
  } else if (check_status(total_status, NavStatus::MID_ACCURACY)) {
    LC_LDEBUG_EVERY(SYSTEM, 10) << "MID_ACCURACY, reason: " << cur_reason;
    cur_status = NavStatus::MID_ACCURACY;
  } else if (check_status(total_status, NavStatus::LOW_ACCURACY)) {
    LC_LDEBUG_EVERY(SYSTEM, 5) << "LOW_ACCURACY, reason: " << cur_reason;
    cur_status = NavStatus::LOW_ACCURACY;
  } else {
    LC_LDEBUG_EVERY(SYSTEM, 1) << "FATAL_ACCURACY, reason: " << cur_reason;
    cur_status = NavStatus::FATAL_ACCURACY;
  }

  // smoothing final status results, strategy: wide out severe into
  int64_t imu_freq = param_.common_param.ins_device == "CHNAV" ? 100 : 125;
  static std::map<NavStatus, int64_t> status_into_thre{
      {NavStatus::FATAL_ACCURACY, 0},             // invalid
      {NavStatus::LOW_ACCURACY, imu_freq},        // about 1s delay check
      {NavStatus::MID_ACCURACY, imu_freq * 2},    // about 2s delay check
      {NavStatus::HIGH_ACCURACY, imu_freq * 3}};  // about 3s delay check
  NavStatus last_status, output_status;
  {
    std::lock_guard<std::mutex> lock(window_mutex_);
    if (window_states_.empty()) return NavStatus::FATAL_ACCURACY;
    last_status = GetNavStatusEnum(window_states_.back().nav_status);
  }
  if (cur_status <= last_status) {
    for (auto iter = status_counter_.begin(); iter != status_counter_.end();
         ++iter) {
      if (iter->first > cur_status)
        iter->second = status_into_thre.at(cur_status);
    }
    output_status = cur_status;
  } else {
    output_status = last_status;
    for (auto iter = status_counter_.begin(); iter != status_counter_.end();
         ++iter) {
      auto status = iter->first;
      if (status <= last_status) continue;
      if (status <= cur_status) ++status_counter_[status];
      if (status_counter_.at(status) >= status_into_thre.at(status)) {
        output_status = status;
      }
    }
  }

  multi_source_nav_status_ = std::move(total_status);
  latest_nav_status_ = output_status;
  return output_status;
}

adLocStatus_t LocalizationFrontendIMU::LocatorProcess(
    NavState* nav_state, bool* continue_to_process) {
  if (nullptr == nav_state || nullptr == continue_to_process) {
    LC_LERROR(SYSTEM) << "nullptr";
    return LOC_NULL_PTR;
  }
  uint64_t frontend_time;
  static auto imu_data = std::make_shared<Imu>();
  {
    std::lock_guard<std::mutex> guard(data_mutex_);
    if (!data_list_.empty()) {
      *imu_data = data_list_.front().second;
      frontend_time = data_list_.front().first;
      data_list_.erase(data_list_.begin());
      (*continue_to_process) = !data_list_.empty();
    } else {
      (*continue_to_process) = false;
      return LOC_LOCALIZATION_ERROR;
    }
  }

  LC_LDEBUG_EVERY_SEC(SYSTEM, 1) << "frontend process imu: " << frontend_time
                                 << "/" << Time::ToString(frontend_time);

  if (LOC_SUCCESS != TimeOrderCheck(frontend_time)) {
    return LOC_LOCALIZATION_ERROR;
  }

  // msf state update
  if (msf_fusion_->IsStarted()) msf_fusion_->StateUpdate();

  // TODO(zm): as sins is visited in msf_fusion_, should consider add mutex if
  // using in other threads
  if (LOC_SUCCESS != locator_->Process(frontend_time, imu_data)) {
    LC_LDEBUG(SYSTEM) << "front imu locator process failed";
    return LOC_LOCALIZATION_ERROR;
  }
  locator_->GetState(nav_state);
  nav_state->nav_status = GetNavStatusNum(GetCurrentNavStatus(*nav_state));

  // when imu is aligned, start fusion
  if (msf_fusion_->IsStarted()) {
    const auto& acc = imu_data->linear_acceleration;
    const auto& gyro = imu_data->angular_velocity;
    IMUMeasurement imu_reading;
    imu_reading << Eigen::Vector3d(acc.x, acc.y, acc.z),
        Eigen::Vector3d(gyro.x, gyro.y, gyro.z);
    msf_fusion_->AddData(nav_state->timestamp, imu_reading);
  }

  // update front end time
  frontend_time_ = frontend_time;

  static_detector_->SetImuData(frontend_time, *imu_data);
  static_detector_->SetLocalizationSpeed(nav_state->linear_speed.norm());
  double static_flag = 0;  // 1 is static, 0 is dynamic
  if (static_detector_->StaticDetect(
          param_.msf_param.imu_static_acc_var_thre) == LOC_SUCCESS) {
    static_flag = 1;
    AddZUPTConstraints(*nav_state, true);
  } else {
    AddZUPTConstraints(*nav_state, false);
  }
  if (param_.ci_param.enable_evaluation && static_evaluator_) {
    static_evaluator_->WriteResult(frontend_time * kNanoSecToSec, static_flag);
  }

  return LOC_SUCCESS;
}

adLocStatus_t LocalizationFrontendIMU::InsAidedFastInitialization() {
  if (ins_init_locator_ == nullptr) {
    LC_LERROR(SYSTEM) << "nullptr ins initialization locator";
    return LOC_LOCALIZATION_ERROR;
  }

  uint64_t ins_time_stamp;
  static auto ins_data = std::make_shared<Ins>();
  {
    std::lock_guard<std::mutex> guard(ins_mutex_);
    if (ins_data_list_.empty()) return LOC_LOCALIZATION_ERROR;
    *ins_data = ins_data_list_.back().second;
    ins_time_stamp = ins_data_list_.back().first;
  }

  if (last_init_state_time_ >= ins_time_stamp) return LOC_LOCALIZATION_ERROR;
  last_init_state_time_ = ins_time_stamp;

  static auto imu_data = std::make_shared<Imu>();
  {
    std::lock_guard<std::mutex> guard(data_mutex_);
    // to ensure always get imu data, here is somewhat brute force
    if (data_list_.empty() == false) {
      float64_t gap = data_list_.back().first * kNanoSecToSec -
                      ins_time_stamp * kNanoSecToSec;
      if (std::fabs(gap) < 0.05) {
        *imu_data = data_list_.back().second;
      } else {
        return LOC_LOCALIZATION_ERROR;
      }
    } else {
      return LOC_LOCALIZATION_ERROR;
    }
  }

  NavState nav_state;
  adLocStatus_t status =
      ins_init_locator_->Process(ins_time_stamp, ins_data, imu_data);
  if (status != LOC_SUCCESS) {
    LC_LWARN(SYSTEM) << "ins init locator error";
    return LOC_LOCALIZATION_ERROR;
  }

  // get state and store into window states for publish, should set CONVERGING
  // status as using INS aided
  ins_init_locator_->GetState(&nav_state);
  nav_state.nav_status = GetNavStatusNum(NavStatus::INITIALIZING);
  StoreState(nav_state);

  return LOC_SUCCESS;
}

adLocStatus_t LocalizationFrontendIMU::FastInitialization() {
  NavState nav_state;
  auto status = initializer_->GetLatestState(&nav_state);
  if (status != LOC_SUCCESS) return status;

  if (last_init_state_time_ >= nav_state.timestamp)
    return LOC_LOCALIZATION_ERROR;
  last_init_state_time_ = nav_state.timestamp;

  nav_state.nav_status = GetNavStatusNum(NavStatus::INITIALIZING);
  StoreState(nav_state);
  return LOC_SUCCESS;
}

NavStatus LocalizationFrontendIMU::CheckFrontendStatus(std::string* reason) {
  // check: frontend process frame rate
  if (frontend_frame_rate_ == -1) {
    *reason = "frontend frame rate is fatal;";
    return NavStatus::FATAL_ACCURACY;
  }
  NavStatus output_status;
  std::string output_reason = "";
  int64_t imu_freq = param_.common_param.ins_device == "CHNAV" ? 100 : 125;
  int frame_rate_gap = std::abs(frontend_frame_rate_ - imu_freq);
  if (frame_rate_gap < imu_freq * 0.2) {
    output_status = NavStatus::HIGH_ACCURACY;
  } else if (frame_rate_gap < imu_freq * 0.65) {
    output_reason = "frontend frame rate is slightly worse;";
    output_status = NavStatus::MID_ACCURACY;
  } else if (frame_rate_gap < imu_freq * 0.85) {
    output_reason = "frontend frame rate is worse;";
    output_status = NavStatus::LOW_ACCURACY;
  } else {
    output_reason = "frontend frame rate is fatal;";
    output_status = NavStatus::FATAL_ACCURACY;
  }
  *reason = output_reason;
  return output_status;
}

NavStatus LocalizationFrontendIMU::CheckBackendStatus(std::string* reason) {
  if (!loc_backends_.count(GNSS) && !loc_backends_.count(SMM)) {
    *reason = "backend confidence is fatal";
    return NavStatus::FATAL_ACCURACY;
  }

  std::map<LocatorType, NavStatus> be_status;
  for (const auto& item : loc_backends_) {
    be_status[item.first] = item.second->GetCurrentLocatorStatus();
  }

  // check: backend observation confidence (integrality, frame rate, map
  // localization confidence ...)
  NavStatus output_status;
  std::string output_reason = "";
  if (be_status.count(SMM)) {
    if (be_status.count(GNSS) &&
        be_status.at(GNSS) == NavStatus::FATAL_ACCURACY) {
      // long periods of missing RTK signal
      output_status = NavStatus::FATAL_ACCURACY;
    } else if (be_status.at(SMM) >= NavStatus::MID_ACCURACY) {
      // output high/mid accuracy just only SMM has high/mid confidence
      output_status = be_status.at(SMM);
    } else {
      bool condition =
          be_status.count(GNSS) && be_status.at(GNSS) > be_status.at(SMM);
      NavStatus st = condition ? be_status.at(GNSS) : be_status.at(SMM);
      output_status =
          st > NavStatus::LOW_ACCURACY ? NavStatus::LOW_ACCURACY : st;
    }
  } else {
    // not output high accuracy for No SMM backend
    NavStatus st = be_status.at(GNSS);
    output_status = st > NavStatus::MID_ACCURACY ? NavStatus::MID_ACCURACY : st;
  }
  if (output_status == NavStatus::MID_ACCURACY) {
    output_reason = "backend confidence is slightly worse;";
  } else if (output_status == NavStatus::LOW_ACCURACY) {
    output_reason = "backend confidence is worse;";
  } else if (output_status == NavStatus::FATAL_ACCURACY) {
    output_reason = "backend confidence is fatal;";
  }
  *reason = output_reason;

  backends_status_ = std::move(be_status);
  return output_status;
}

NavStatus LocalizationFrontendIMU::CheckMSFStatus(const NavState& nav_state,
                                                  std::string* reason) {
  // check1: MSF fusion covariance
  auto rot = nav_state.pose.so3().inverse().matrix();
  // covariance rotate to car-center frame
  Eigen::Matrix3d tra_cov =
      rot * nav_state.pose_cov.topLeftCorner(3, 3) * rot.transpose();
  double x_std = std::sqrt(tra_cov(0, 0));
  double y_std = std::sqrt(tra_cov(1, 1));
  double z_std = std::sqrt(tra_cov(2, 2));
  double heading_std = std::sqrt(nav_state.pose_cov(5, 5));

  bool ha_check1 = true, ma_check1 = true, la_check1 = true;
  if (param_.common_param.using_l4_metric_status_output) {
    ha_check1 =
        x_std < 0.1 && y_std < 0.1 && z_std < 0.15 && heading_std < 0.01;
    ma_check1 = x_std < 0.3 && y_std < 0.3 && z_std < 0.4 && heading_std < 0.05;
    la_check1 = x_std < 1.0 && y_std < 1.0 && z_std < 1.0 && heading_std < 0.1;
  } else {  // longitude error can be ignore in l2 metric
    ha_check1 = x_std < 1.0 && y_std < 0.1 && z_std < 2.0 && heading_std < 0.01;
    ma_check1 = x_std < 8.0 && y_std < 0.6 && z_std < 10.0 && heading_std < 0.1;
    la_check1 =
        x_std < 12.0 && y_std < 2.0 && z_std < 16.0 && heading_std < 0.2;
  }

  // check2: MSF fusion error state
  Eigen::Vector4d error_state;
  bool ha_check2 = true, ma_check2 = true, la_check2 = true;
  if (msf_fusion_->IsStarted() &&
      LOC_SUCCESS == msf_fusion_->GetLatestMaxErrorState(&error_state)) {
    Eigen::Vector3d err_local_tra = error_state.head(3).cwiseAbs();
    double err_heading = std::fabs(error_state(3));
    if (param_.common_param.using_l4_metric_status_output) {
      ha_check2 = err_local_tra(0) < 0.15 && err_local_tra(1) < 0.15 &&
                  err_local_tra(2) < 0.2 && err_heading < 0.01;
      ma_check2 = err_local_tra(0) < 0.3 && err_local_tra(1) < 0.3 &&
                  err_local_tra(2) < 0.5 && err_heading < 0.05;
      la_check2 = err_local_tra(0) < 1.0 && err_local_tra(1) < 1.0 &&
                  err_local_tra(2) < 1.5 && err_heading < 0.1;
    } else {
      ha_check2 = err_local_tra(0) < 1.0 && err_local_tra(1) < 0.15 &&
                  err_local_tra(2) < 2.0 && err_heading < 0.01;
      ma_check2 = err_local_tra(0) < 8.0 && err_local_tra(1) < 0.6 &&
                  err_local_tra(2) < 10.0 && err_heading < 0.1;
      la_check2 = err_local_tra(0) < 12.0 && err_local_tra(1) < 2.0 &&
                  err_local_tra(2) < 16.0 && err_heading < 0.2;
    }
  }

  NavStatus output_status;
  std::string output_reason = "";
  if (ha_check1 && ha_check2) {
    output_status = NavStatus::HIGH_ACCURACY;
  } else if (ma_check1 && ma_check2) {
    if (!ha_check1) output_reason += "msf covariance is slightly worse;";
    if (!ha_check2) output_reason += "msf error state is slightly worse;";
    output_status = NavStatus::MID_ACCURACY;
  } else if (la_check1 && la_check2) {
    if (!ma_check1) output_reason += "msf covariance is worse;";
    if (!ma_check2) output_reason += "msf error state is worse;";
    output_status = NavStatus::LOW_ACCURACY;
  } else {
    if (!la_check1) output_reason += "msf covariance is fatal;";
    if (!la_check2) output_reason += "msf error state is fatal;";
    output_status = NavStatus::FATAL_ACCURACY;
  }
  *reason = output_reason;
  return output_status;
}

NavStatus LocalizationFrontendIMU::CheckOutputStatus(std::string* reason) {
  NavState last_state, llast_state;
  {
    std::lock_guard<std::mutex> lock(window_mutex_);
    if (window_states_.size() <= 1) return NavStatus::FATAL_ACCURACY;
    last_state = *(window_states_.rbegin());
    llast_state = *(++window_states_.rbegin());
  }
  double delta_ts =
      last_state.timestamp * 1.0e-9 - llast_state.timestamp * 1.0e-9;
  SE3d delta_pose = llast_state.pose.inverse() * last_state.pose;
  double lateral_vel = std::fabs(delta_pose.translation()(1) / delta_ts);

  // check: output nav state lateral velocity
  static const double HIGH_VEL_THRE =
      param_.common_param.using_l4_metric_status_output ? 10.0 : 6.0;
  static const double MID_VEL_THRE =
      param_.common_param.using_l4_metric_status_output ? 20.0 : 10.0;
  static const double LOW_VEL_THRE =
      param_.common_param.using_l4_metric_status_output ? 30.0 : 15.0;
  NavStatus output_status;
  std::string output_reason = "";
  if (lateral_vel < HIGH_VEL_THRE) {
    output_status = NavStatus::HIGH_ACCURACY;
  } else if (lateral_vel < MID_VEL_THRE) {
    output_reason = "localization smoothness is slightly worse;";
    output_status = NavStatus::MID_ACCURACY;
  } else if (lateral_vel < LOW_VEL_THRE) {
    output_reason = "localization smoothness is worse;";
    output_status = NavStatus::LOW_ACCURACY;
  } else {
    output_reason = "localization smoothness is fatal;";
    output_status = NavStatus::FATAL_ACCURACY;
  }
  *reason = output_reason;
  return output_status;
}

void LocalizationFrontendIMU::AddZUPTConstraints(const NavState& nav_state,
                                                 bool parking) {
  if (!parking) {
    if (update_fix_state_ == false) {
      LC_LDEBUG(SYSTEM) << "vehicle parking ends";
    }
    update_fix_state_ = true;
    return;
  }
  if (parking && update_fix_state_) {
    fixed_nav_state_ = nav_state;
    update_fix_state_ = false;
    LC_LDEBUG(SYSTEM) << "vehicle parking starts";
  }

  // reduce add factor frequency, 10Hz
  constexpr uint64_t interval_time = 1e8;
  if (nav_state.timestamp < last_zupt_timestamp_ ||
      (nav_state.timestamp - last_zupt_timestamp_) <= interval_time)
    return;
  last_zupt_timestamp_ = nav_state.timestamp;

  Eigen::Matrix<double, 10, 1> full_obs;
  full_obs.segment<3>(0) = fixed_nav_state_.pose.translation();
  full_obs.segment<3>(3) = Vector3d::Zero();
  full_obs.segment<4>(6) = Utility::EigenQtoHamiltonVec(
      fixed_nav_state_.pose.so3().unit_quaternion());
  Eigen::Matrix<double, 9, 9> full_obs_cov;
  full_obs_cov.setIdentity();
  full_obs_cov *= 1.0e-4;

  std::shared_ptr<msf::MSFFactor> msf_factor_mock_full =
      msf::MSFFactorFactory::CreateMSFFactor(IMU, MOCK);
  if (msf_factor_mock_full == nullptr) {
    LC_LERROR(SYSTEM) << "unsupported fusion mode !!!";
    return;
  }
  msf_factor_mock_full->SetObservationState(full_obs);
  msf_factor_mock_full->SetObservationCov(full_obs_cov);
  msf_factor_mock_full->SetSource(MOCK);
  // add delayed 50ms factor
  uint64_t delayed_timestamp = nav_state.timestamp - 5e7;
  // add mock constraint factor into queue
  if (msf_fusion_->IsStarted()) {
    msf_fusion_->AddFactor(delayed_timestamp, msf_factor_mock_full);
  }
}

}  // namespace localization
}  // namespace senseAD
