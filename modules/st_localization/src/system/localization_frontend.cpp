/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */

#include "system/localization_frontend.hpp"

#include <algorithm>
#include <deque>
#include <memory>
#include <string>

#include "ad_common/data_type/base.hpp"
#include "common/coordinate_converter.hpp"
#include "common/utility.hpp"
#include "eval/evaluator_gnss_status.hpp"
#include "eval/evaluator_localization_status.hpp"
#include "eval/evaluator_localziation.hpp"
#include "eval/evaluator_motion.hpp"
#include "gnss/gnss_locator.hpp"
#include "ins/ins_locator.hpp"
#include "localization/common/log.hpp"
#include "msf/msf_fusion/msf_fusion.hpp"
#include "system/backend/localization_backend_gnss.hpp"
#include "system/localization_backend.hpp"
#include "system/localization_visualizer.hpp"

namespace senseAD {
namespace localization {

LocalizationFrontend::~LocalizationFrontend() {
  if (thread_ && thread_->joinable()) thread_->join();
}

adLocStatus_t LocalizationFrontend::Init(const LocalizationParam& param) {
  param_ = param;

  // init gt related locator and evaluator
  if (InitGtLocatorAndEvaluator() != LOC_SUCCESS) {
    LC_LERROR(SYSTEM) << "Failed to init GT locator and evaluator.";
    return LOC_LOCALIZATION_ERROR;
  }

  // init frontend locator
  if (InitLocator() != LOC_SUCCESS) {
    LC_LERROR(SYSTEM) << "Failed to init frontend locator.";
    return LOC_LOCALIZATION_ERROR;
  }

  // init initilizer
  if (InitInitializer() != LOC_SUCCESS) {
    LC_LERROR(SYSTEM) << "failed to init frontend Initializer";
    return LOC_LOCALIZATION_ERROR;
  }

  // init data buffer
  if (OnInitDataBuffer() != LOC_SUCCESS) {
    LC_LERROR(SYSTEM) << "Failed to init frontend data buffer.";
    return LOC_LOCALIZATION_ERROR;
  }

  // init thread
  thread_.reset(new std::thread(&LocalizationFrontend::Run, this));
  if (nullptr == thread_) {
    LC_LERROR(SYSTEM) << "failed to create frontend thread";
    return LOC_LOCALIZATION_ERROR;
  }
  SetThreadName();

  return LOC_SUCCESS;
}

adLocStatus_t LocalizationFrontend::OnInitDataBuffer() {
  // init publish data buffer
  {
    std::lock_guard<std::mutex> lock(window_mutex_);
    window_states_.set_capacity(param_.common_param.window_size);
  }
  frc_timestamp_buffer_.set_capacity(frc_buffer_size_);

  // init subscribe data buffer
  auto status = InitDataBuffer();
  if (status != LOC_SUCCESS) return status;
  return LOC_SUCCESS;
}

adLocStatus_t LocalizationFrontend::Restart() {
  // reset related variable
  frontend_time_ = 0;
  frontend_frame_rate_ = -1;
  frc_timestamp_buffer_.clear();
  frc_total_count_ = 0;
  frc_total_frame_rates_ = 0;
  last_get_timestamp_ = 0;
  {
    std::lock_guard<std::mutex> lock(window_mutex_);
    window_states_.clear();
  }

  // restart sub data
  if (ResetSubData() != LOC_SUCCESS) {
    LC_LERROR(SYSTEM) << "Failed to reset frontend sub data.";
    return LOC_LOCALIZATION_ERROR;
  }

  // restart locator
  if (InitLocator() != LOC_SUCCESS) {
    LC_LERROR(SYSTEM) << "Failed to restart frontend locator.";
    return LOC_LOCALIZATION_ERROR;
  }
  // resatrt initializer
  if (InitInitializer() != LOC_SUCCESS) {
    LC_LERROR(SYSTEM) << "Failed to restart frontend Initializer";
    return LOC_LOCALIZATION_ERROR;
  }

  return LOC_SUCCESS;
}

bool LocalizationFrontend::CheckNeedRestart() {
  if (initializer_->GetInitStage() != InitStage::INIT_DONE) return false;

  // check for frontend
  if (CheckFrontendNeedRestart()) return true;

  // check for gnss backend
  if (loc_backends_.count(GNSS)) {
    auto gnss_backend = std::dynamic_pointer_cast<LocalizationBackendGNSS>(
        loc_backends_.at(GNSS));
    if (gnss_backend->CheckNeedRestart()) return true;
  }

  // TODO(xxx): check for other backends

  return false;
}

adLocStatus_t LocalizationFrontend::SwitchOriginProc() {
  // convert enu position to new origin
  {
    PointLLH_t enu_origin;
    CoordinateConverter::GetInstance()->GetOrigin(&enu_origin);
    std::lock_guard<std::mutex> lock(window_mutex_);
    for (auto& nav_state : window_states_) {
      nav_state.origin = enu_origin;
      auto& pose = nav_state.pose;
      CoordinateConverter::GetInstance()->ConvertENUToNewOrigin(
          pose.translation(), &pose.translation());
    }
  }
  {
    std::lock_guard<std::mutex> lock(gt_state_mutex_);
    auto& pose = nav_gt_state_.pose;
    CoordinateConverter::GetInstance()->ConvertENUToNewOrigin(
        pose.translation(), &pose.translation());
  }
  {
    std::lock_guard<std::mutex> lock(gnss_state_mutex_);
    auto& pose = nav_gnss_state_.pose;
    CoordinateConverter::GetInstance()->ConvertENUToNewOrigin(
        pose.translation(), &pose.translation());
  }

  // reinit for evaluator related
  if (param_.ci_param.enable_evaluation) {
    static uint64_t origin_id = 1;
    std::string tmp = "_" + std::to_string(origin_id);
    ++origin_id;
    std::string setting_type = "GT" + tmp;
    gt_evaluator_->Init(param_.ci_param.results_save_dir, setting_type,
                        param_.ci_param.testcase_id);
    setting_type = "RTK_GNSS" + tmp;
    gt_rtk_evaluator_->Init(param_.ci_param.results_save_dir, setting_type,
                            param_.ci_param.testcase_id);
    setting_type = "RTK_GNSS_STATUS" + tmp;
    gt_rtk_status_evaluator_->Init(param_.ci_param.results_save_dir,
                                   setting_type, param_.ci_param.testcase_id);
    setting_type = param_.common_param.front_locator_type + "_" +
                   param_.common_param.back_locator_type + tmp;
    evaluator_->Init(param_.ci_param.results_save_dir, setting_type,
                     param_.ci_param.testcase_id);
    setting_type = "STATUS" + tmp;
    status_evaluator_->Init(param_.ci_param.results_save_dir, setting_type,
                            param_.ci_param.testcase_id);
    setting_type = "INIT" + tmp;
    init_evaluator_->Init(param_.ci_param.results_save_dir, setting_type,
                          param_.ci_param.testcase_id);
  }

  // switch origin sub process
  if (SwitchOriginSubProc() != LOC_SUCCESS) {
    LC_LERROR(SYSTEM) << "Failed to switch origin sub process.";
    return LOC_LOCALIZATION_ERROR;
  }

  return LOC_SUCCESS;
}

bool LocalizationFrontend::CheckNeedSwitchOrigin() const {
  // TODO(xxx): check for frontend
  if (initializer_->GetInitStage() != InitStage::INIT_DONE) return false;

  // check for gnss backend
  if (loc_backends_.count(GNSS)) {
    auto gnss_backend = std::dynamic_pointer_cast<LocalizationBackendGNSS>(
        loc_backends_.at(GNSS));
    if (gnss_backend->CheckNeedSwitchOrigin()) return true;
  }

  // TODO(xxx): check for other backends

  return false;
}

void LocalizationFrontend::SetMSFFusion(
    std::shared_ptr<msf::MSFFusion> msf_fusion) {
  msf_fusion_ = msf_fusion;
}

void LocalizationFrontend::SetLocalizationBackends(
    const std::map<LocatorType, std::shared_ptr<LocalizationBackend>>& bes) {
  loc_backends_ = bes;
}

void LocalizationFrontend::SetLocalizationVisualizer(
    std::shared_ptr<LocalizationVisualizer> visualizer) {
  loc_visualizer_ = visualizer;
}

std::shared_ptr<Initialization> LocalizationFrontend::GetInitilizer() {
  return initializer_;
}

adLocStatus_t LocalizationFrontend::GtLocatorForEval(uint64_t timestamp,
                                                     const Ins& ins) {
  static auto ins_data = std::make_shared<Ins>();
  *ins_data = ins;
  if (gt_locator_->Process(timestamp, ins_data, nullptr) != LOC_SUCCESS) {
    LC_LERROR(SYSTEM) << "gt locator process failed";
    return LOC_LOCALIZATION_ERROR;
  }

  NavState nav_state;
  gt_locator_->GetState(&nav_state);
  {
    std::lock_guard<std::mutex> lock_gt(gt_state_mutex_);
    nav_gt_state_ = nav_state;
  }
  // write GT evaluation results
  if (gt_evaluator_) {
    gt_evaluator_->WriteResult(timestamp * kNanoSecToSec,
                               nav_state.pose.matrix());
  }
  if (gt_motion_evaluator_) {
    MotionEvalData motion_data;
    motion_data.linear_speed = nav_state.linear_speed;
    // TODO(fy): get GT angular speed and acceleration
    // motion_data.angular_speed = nav_state.angular_speed;
    // motion_data.linear_acceleration = nav_state.linear_acceleration;
    gt_motion_evaluator_->WriteResult(timestamp * kNanoSecToSec, motion_data);
  }

  return LOC_SUCCESS;
}

adLocStatus_t LocalizationFrontend::GtRtkLocatorForEval(uint64_t timestamp,
                                                        const Gnss& gnss) {
  static auto gnss_data = std::make_shared<Gnss>();
  *gnss_data = gnss;

  NavState nav_state;
  if (GNSSLocator::ProcessForGtMode(timestamp, gnss_data, &nav_state) !=
      LOC_SUCCESS) {
    LC_LERROR(SYSTEM) << "gt rtk locator process failed.";
    return LOC_LOCALIZATION_ERROR;
  }
  {
    std::lock_guard<std::mutex> lock_gnss(gnss_state_mutex_);
    nav_gnss_state_ = nav_state;
  }

  // write RTK evaluation results
  if (gt_rtk_evaluator_) {
    gt_rtk_evaluator_->WriteResult(timestamp * kNanoSecToSec,
                                   nav_state.pose.matrix());
  }
  if (gt_rtk_status_evaluator_) {
    gt_rtk_status_evaluator_->WriteResult(
        timestamp * kNanoSecToSec, static_cast<int16_t>(gnss_data->status));
  }

  return LOC_SUCCESS;
}

uint64_t LocalizationFrontend::GetFrontendTime() const {
  return frontend_time_;
}

adLocStatus_t LocalizationFrontend::GetGtState(NavState* gt_state,
                                               NavState* gt_rtk_state) const {
  if (nullptr == gt_state || nullptr == gt_rtk_state) return LOC_NULL_PTR;
  {
    std::lock_guard<std::mutex> lock(gt_state_mutex_);
    *gt_state = nav_gt_state_;
  }
  {
    std::lock_guard<std::mutex> lock(gnss_state_mutex_);
    *gt_rtk_state = nav_gnss_state_;
  }
  return LOC_SUCCESS;
}

adLocStatus_t LocalizationFrontend::QueryPoseByTime(uint64_t timestamp,
                                                    NavState* state) const {
  if (nullptr == state) return LOC_NULL_PTR;
  std::lock_guard<std::mutex> guard(window_mutex_);

  // time check
  if (window_states_.size() < 2) {
    LC_LDEBUG(SYSTEM) << "not enough valid pose window";
    return LOC_INVALID;
  }
  if (window_states_.front().timestamp > timestamp) {
    LC_LDEBUG(SYSTEM) << "query time delay window too much";
    return LOC_TIME_DELAY;
  }
  // at most 100ms ahead
  static constexpr uint64_t ahead_gap = 1e8;
  if (window_states_.back().timestamp + ahead_gap < timestamp) {
    LC_LDEBUG(SYSTEM) << "query time ahead window too much";
    return LOC_TIME_AHEAD;
  }

  // TODO(wangxiaofeng): may change it to vector for index search
  if (window_states_.back().timestamp >= timestamp) {
    // interpolate
    auto last_iter = window_states_.begin();
    auto next_iter = window_states_.end();
    --next_iter;
    size_t last_ind = 0;
    size_t next_ind = window_states_.size() - 1;
    while (last_ind <= next_ind) {
      if (last_iter->timestamp == timestamp) {
        (*state) = (*last_iter);
        return LOC_SUCCESS;
      }
      if (next_iter->timestamp == timestamp) {
        (*state) = (*next_iter);
        return LOC_SUCCESS;
      }
      if (last_iter->timestamp < timestamp) {
        ++last_iter;
        ++last_ind;
      }
      if (next_iter->timestamp > timestamp) {
        --next_iter;
        --next_ind;
      }
    }

    // interpolation failure is not allowed
    double factor = 1.0 * (timestamp - next_iter->timestamp) /
                    (last_iter->timestamp - next_iter->timestamp);
    NavStateInterp(*next_iter, *last_iter, factor, state);
  } else {
    // extrapolate
    static constexpr size_t min_extrapolate_diff = 10;
    if (window_states_.size() < min_extrapolate_diff) {
      LC_LDEBUG(SYSTEM)
          << "query time ahead, but not enough pose to extrapolate";
      return LOC_TIME_AHEAD;
    }

    NavState latest_state, last_state, ahead_state;
    size_t ind = 0;
    auto iter = window_states_.rbegin();
    for (; iter != window_states_.rend(); ++iter, ++ind) {
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
    // TODO(wangxiaofeng): others doesn't matter at now ?
    ahead_state.pose =
        latest_state.pose * (last_state.pose.inverse() * latest_state.pose);

    // interpolation failure is not allowed
    double factor = 1.0 * (timestamp - latest_state.timestamp) /
                    (ahead_state.timestamp - latest_state.timestamp);
    NavStateInterp(latest_state, ahead_state, factor, state);
  }

  return LOC_SUCCESS;
}

adLocStatus_t LocalizationFrontend::GetLatestState(NavState* nav_state) const {
  if (nullptr == nav_state) {
    LC_LERROR(SYSTEM) << "nullptr";
    return LOC_NULL_PTR;
  }
  std::unique_lock<std::mutex> locker(window_mutex_);
  if (window_states_.empty()) {
    LC_LINFO_EVERY(SYSTEM, 20) << "window states is empty!";
    return LOC_LOCALIZATION_ERROR;
  }
  *nav_state = window_states_.back();
  return LOC_SUCCESS;
}

adLocStatus_t LocalizationFrontend::GetNavStateInfo(NavStateInfo* info) const {
  NavState cur_state;
  std::unique_lock<std::mutex> locker(window_mutex_);
  if (window_states_.empty()) {
    LC_LDEBUG_EVERY(SYSTEM, 20) << "window states is empty!";
    return LOC_LOCALIZATION_ERROR;
  }
  const auto& back_state = window_states_.back();
  if (last_get_timestamp_ == 0 ||
      (back_state.timestamp * 1e-9 - last_get_timestamp_ * 1e-9) > 0.1) {
    cur_state = back_state;  // get the latest state
  } else {
    auto iter = window_states_.rbegin();
    for (; iter != window_states_.rend(); ++iter) {
      if (iter->timestamp <= last_get_timestamp_) break;
    }
    if (iter != window_states_.rbegin()) --iter;
    cur_state = *iter;  // get the first state not published
  }
  locker.unlock();
  last_get_timestamp_ = cur_state.timestamp;

  // convert nav state info
  return ConvertNavStateInfo(cur_state, info);
}

void LocalizationFrontend::Run() {
  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    // check finish request
    if (CheckFinishRequest()) break;

    // check pause request
    if (CheckPauseRequest()) {
      SetPause();
      while (IsPaused()) WaitReleasePaused(1000);
    }

    // frontend process core
    FrontendProcess();
  }
  SetFinish();
}

void LocalizationFrontend::FrontendProcess() {
  // initilize
  if (Initialize() != InitStage::INIT_DONE) {
    NavState nav_state;
    // for evaluate localization output
    if (init_evaluator_ && GetLatestState(&nav_state) == LOC_SUCCESS) {
      if (nav_state.timestamp > last_init_evaluator_time_) {
        last_init_evaluator_time_ = nav_state.timestamp;
        init_evaluator_->WriteResult(nav_state.timestamp * kNanoSecToSec,
                                     nav_state.pose.matrix());
      }
    }

    // for visualizer
    if (loc_visualizer_) SetCurrentNavStateInfoForVisual();
    return;
  }

  // frontend data process
  bool odom_continue_to_process = true;
  while (odom_continue_to_process) {
    // replay locator process
    NavState nav_state;
    if (LOC_SUCCESS != LocatorProcess(&nav_state, &odom_continue_to_process)) {
      break;
    }

    // frontend processed frame rate check
    frontend_frame_rate_ = FrameRateCheck(nav_state.timestamp);

    // store nav state into window states
    const auto& s_nav_state = StoreState(nav_state);

    // for evaluate localization output
    if (evaluator_ && status_evaluator_ && s_nav_state.timestamp != 0) {
      SaveLocalizationEvalData(s_nav_state);
    }

    // for visualizer
    if (loc_visualizer_) SetCurrentNavStateInfoForVisual();
  }
}

adLocStatus_t LocalizationFrontend::InitGtLocatorAndEvaluator() {
  // create and init GT locator
  gt_locator_ = std::make_shared<INSLocator>();
  if (nullptr == gt_locator_) {
    LC_LERROR(SYSTEM) << "Failed to create gt locator.";
    return LOC_ALLOC_MEMORY_FAILED;
  }
  if (gt_locator_->Init(param_) != LOC_SUCCESS) {
    LC_LERROR(SYSTEM) << "Failed to init gt locator.";
    return LOC_LOCALIZATION_ERROR;
  }

  if (param_.ci_param.enable_evaluation) {
    // create and init localization output evaluator
    std::string setting_type = param_.common_param.front_locator_type + "_" +
                               param_.common_param.back_locator_type;
    evaluator_ = std::make_shared<EvaluatorLocalization>();
    if (nullptr == evaluator_) {
      LC_LERROR(SYSTEM) << "Failed to create localization evaluator.";
      return LOC_ALLOC_MEMORY_FAILED;
    }
    if (evaluator_->Init(param_.ci_param.results_save_dir, setting_type,
                         param_.ci_param.testcase_id) != LOC_SUCCESS) {
      LC_LERROR(SYSTEM) << "Failed to init localization evaluator.";
      return LOC_LOCALIZATION_ERROR;
    }

    // create and init status evaluator
    setting_type = "STATUS";
    status_evaluator_ = std::make_shared<EvaluatorLocalizationStatus>();
    if (nullptr == status_evaluator_) {
      LC_LERROR(SYSTEM) << "Failed to create status evaluator.";
      return LOC_ALLOC_MEMORY_FAILED;
    }
    if (status_evaluator_->Init(param_.ci_param.results_save_dir, setting_type,
                                param_.ci_param.testcase_id) != LOC_SUCCESS) {
      LC_LERROR(SYSTEM) << "Failed to init status evaluator.";
      return LOC_LOCALIZATION_ERROR;
    }

    // create and init initialization evaluator
    setting_type = "INIT";
    init_evaluator_ = std::make_shared<EvaluatorLocalization>();
    if (nullptr == init_evaluator_) {
      LC_LERROR(SYSTEM) << "Failed to create initialization evaluator.";
      return LOC_ALLOC_MEMORY_FAILED;
    }
    init_evaluator_->Init(param_.ci_param.results_save_dir, setting_type,
                          param_.ci_param.testcase_id);

    // create and init GT evaluator
    setting_type = "GT";
    gt_evaluator_ = std::make_shared<EvaluatorLocalization>();
    if (nullptr == gt_evaluator_) {
      LC_LERROR(SYSTEM) << "Failed to create gt evaluator.";
      return LOC_ALLOC_MEMORY_FAILED;
    }
    if (gt_evaluator_->Init(param_.ci_param.results_save_dir, setting_type,
                            param_.ci_param.testcase_id) != LOC_SUCCESS) {
      LC_LERROR(SYSTEM) << "Failed to init gt evaluator.";
      return LOC_LOCALIZATION_ERROR;
    }

    // create and init GT motion evaluator
    setting_type = "GT_MOTION";
    gt_motion_evaluator_ = std::make_shared<EvaluatorMotion>();
    if (nullptr == gt_motion_evaluator_) {
      LC_LERROR(SYSTEM) << "Failed to create gt motion evaluator.";
      return LOC_ALLOC_MEMORY_FAILED;
    }
    if (gt_motion_evaluator_->Init(param_.ci_param.results_save_dir,
                                   setting_type, param_.ci_param.testcase_id) !=
        LOC_SUCCESS) {
      LC_LERROR(SYSTEM) << "Failed to init gt motion evaluator.";
      return LOC_LOCALIZATION_ERROR;
    }

    // create and init GT-RTK evaluator
    setting_type = "RTK_GNSS";
    gt_rtk_evaluator_ = std::make_shared<EvaluatorLocalization>();
    if (nullptr == gt_rtk_evaluator_) {
      LC_LERROR(SYSTEM) << "Failed to create rtk evaluator.";
      return LOC_ALLOC_MEMORY_FAILED;
    }
    if (gt_rtk_evaluator_->Init(param_.ci_param.results_save_dir, setting_type,
                                param_.ci_param.testcase_id) != LOC_SUCCESS) {
      LC_LERROR(SYSTEM) << "Failed to init rtk evaluator.";
      return LOC_LOCALIZATION_ERROR;
    }
    setting_type = "RTK_GNSS_STATUS";
    gt_rtk_status_evaluator_ = std::make_shared<EvaluatorGnssStatus>();
    if (nullptr == gt_rtk_status_evaluator_) {
      LC_LERROR(SYSTEM) << "Failed to create rtk status evaluator.";
      return LOC_ALLOC_MEMORY_FAILED;
    }
    if (gt_rtk_status_evaluator_->Init(
            param_.ci_param.results_save_dir, setting_type,
            param_.ci_param.testcase_id) != LOC_SUCCESS) {
      LC_LERROR(SYSTEM) << "Failed to init rtk status evaluator.";
      return LOC_LOCALIZATION_ERROR;
    }
  }
  return LOC_SUCCESS;
}

adLocStatus_t LocalizationFrontend::TimeOrderCheck(uint64_t timestamp) {
  // time order check
  if (timestamp < frontend_time_) {
    LC_LERROR_EVERY_SEC(SYSTEM, 1) << "frontend data time disorder !!! "
                                   << timestamp << " " << frontend_time_;
    return LOC_LOCALIZATION_ERROR;
  }
  return LOC_SUCCESS;
}

int64_t LocalizationFrontend::FrameRateCheck(uint64_t timestamp) {
  frc_timestamp_buffer_.push_back(timestamp);
  if (frc_timestamp_buffer_.size() < frc_buffer_size_) return -1;

  // calculate frame rate
  double time_gap = frc_timestamp_buffer_.back() * 1.0e-9 -
                    frc_timestamp_buffer_.front() * 1.0e-9;
  int64_t frame_rate = static_cast<int64_t>(frc_buffer_size_ / time_gap + 0.5);
  ++frc_total_count_;
  frc_total_frame_rates_ += frame_rate;
  int64_t ave_frame_rate = frc_total_frame_rates_ / frc_total_count_;
  LC_LDEBUG_EVERY_SEC(SYSTEM, 1)
      << "current frame rate: " << frame_rate << " HZ"
      << ", average frame rate: " << ave_frame_rate << " HZ";
  return frame_rate;
}

NavState LocalizationFrontend::StoreState(const NavState& newest_nav_state) {
  // store history position information
  PointLLH_t enu_origin;
  CoordinateConverter::GetInstance()->GetOrigin(&enu_origin);
  static constexpr uint64_t window_gap = 1e7;
  static NavState default_state{};

  bool window_empty = false;
  uint64_t last_standard_time = 0;
  NavState s_ns;
  {
    std::lock_guard<std::mutex> wind_lock(window_mutex_);
    window_empty = window_states_.empty();
    s_ns = window_states_.back();
    last_standard_time = s_ns.timestamp;
  }

  if (newest_nav_state.timestamp <= last_standard_time) {
    return default_state;
  }

  double time_gap =
      (static_cast<double>(newest_nav_state.timestamp) - last_standard_time) /
      static_cast<double>(window_gap);
  // Compensate the lost state
  if (window_empty || time_gap >= 100) {
    if (time_gap >= 100) {
      LC_LERROR_EVERY_SEC(SYSTEM, 1) << "nav state timegap larger than 1 sec,"
                                     << "timestamp jump possible!";
    }
    auto nav_state = newest_nav_state;
    nav_state.timestamp = static_cast<uint64_t>(std::round(nav_state.timestamp *
                                                           1.0 / window_gap)) *
                          window_gap;
    nav_state.origin = enu_origin;
    std::lock_guard<std::mutex> wind_lock(window_mutex_);
    window_states_.clear();
    window_states_.push_back(nav_state);
    return nav_state;
  }

  int interp_num = static_cast<int>(std::round(time_gap));
  NavState smooth_ns = newest_nav_state;
  SmoothNavStateOutput(newest_nav_state, &smooth_ns);
  if (interp_num >= 1) {
    std::list<NavState> interp_nav_states;
    const NavState& e_ns = smooth_ns;
    for (int i = 1; i <= interp_num; ++i) {
      NavState n_state;
      uint64_t c_time = last_standard_time + window_gap * i;
      NavStateInterp(s_ns, e_ns, i / time_gap, &n_state);
      n_state.timestamp = c_time;
      n_state.state_source = e_ns.state_source;
      n_state.origin = enu_origin;
      interp_nav_states.emplace_back(n_state);
    }

    std::lock_guard<std::mutex> wind_lock(window_mutex_);
    for (auto& item : interp_nav_states) {
      window_states_.push_back(std::move(item));
    }
    return window_states_.back();
  }

  return default_state;  // default value, timestamp is 0
}

void LocalizationFrontend::NavStateInterp(const NavState& s_ns,
                                          const NavState& e_ns, double factor,
                                          NavState* ns) const {
  ns->nav_status = e_ns.nav_status;
  ns->origin = e_ns.origin;
  // ns->pose = SE3d::SE3Interpolation(s_ns.pose, e_ns.pose, factor); old sophus
  ns->pose = Utility::SE3interpolate(s_ns.pose, e_ns.pose, factor);
  ns->linear_speed =
      s_ns.linear_speed + (e_ns.linear_speed - s_ns.linear_speed) * factor;
  ns->linear_acceleration =
      s_ns.linear_acceleration +
      (e_ns.linear_acceleration - s_ns.linear_acceleration) * factor;
  ns->angular_speed =
      s_ns.angular_speed + (e_ns.angular_speed - s_ns.angular_speed) * factor;
  // TODO(wangxiaofeng) How about covariance ?
  ns->pose_cov = e_ns.pose_cov;
}

adLocStatus_t LocalizationFrontend::SmoothNavStateOutput(
    const NavState& newest_ns, NavState* smooth_ns) const {
  constexpr size_t base_num = 5;
  NavState lastest_state;
  NavState last_state;
  {
    std::lock_guard<std::mutex> wind_lock(window_mutex_);
    if (window_states_.size() < base_num) return LOC_LOCALIZATION_ERROR;

    // predict future state according to latest localization results
    lastest_state = window_states_.back();
    last_state = window_states_[window_states_.size() - base_num];
  }
  double factor = 1.0 * (newest_ns.timestamp - last_state.timestamp) /
                  (lastest_state.timestamp - last_state.timestamp);
  if (factor <= 1.0) {
    LC_LFATAL(SYSTEM) << "impossible factor for interpolation";
    return LOC_LOCALIZATION_ERROR;
  }
  NavState predict_state;
  predict_state.timestamp = newest_ns.timestamp;
  NavStateInterp(last_state, lastest_state, factor, &predict_state);

  // smmoth the final state for balance prediceted and MSF state
  smooth_ns->timestamp = newest_ns.timestamp;
  smooth_ns->state_source = newest_ns.state_source;
  factor = 0.5;
  NavStateInterp(predict_state, newest_ns, factor, smooth_ns);

  return LOC_SUCCESS;
}

adLocStatus_t LocalizationFrontend::ConvertNavStateInfo(
    const NavState& nav_state, NavStateInfo* info) const {
  if (nullptr == info) {
    LC_LERROR(SYSTEM) << "info nullptr";
    return LOC_NULL_PTR;
  }

  // set timestamp
  info->measurement_time_ns = nav_state.timestamp;

  // set fusion status
  info->nav_status = GetNavStatusEnum(nav_state.nav_status);

  // set lla origin position
  info->origin_position_lla = nav_state.origin;

  // Fill in the latest odometry information
  // Local horizontal position
  Eigen::Vector3d t = nav_state.pose.translation();
  info->position.x = t(0);
  info->position.y = t(1);
  info->position.z = t(2);
  PointENU_t pos_enu;
  pos_enu.x = t(0);
  pos_enu.y = t(1);
  pos_enu.z = t(2);
  PointLLH_t pos_lla;
  // LLA position
  CoordinateConverter::GetInstance()->ENU2LLA(pos_enu, &pos_lla);
  info->position_lla.lat = pos_lla.lat;
  info->position_lla.lon = pos_lla.lon;
  info->position_lla.height = pos_lla.height;
  // attitude
  // Eigen::Vector3d ypr = nav_state.pose.so3().toYPR(); old Sophus
  Eigen::Vector3d ypr =
      nav_state.pose.so3().unit_quaternion().toRotationMatrix().eulerAngles(
          2, 1, 0);
  info->euler_angle.roll = ypr(2);
  info->euler_angle.pitch = ypr(1);
  info->euler_angle.yaw = ypr(0);
  // velocity, acceleration, angular speed
  // global velocity
  Eigen::Vector3d v_g = nav_state.pose.so3().matrix() * nav_state.linear_speed;
  info->global_linear_velocity = {
      v_g(0),
      v_g(1),
      v_g(2),
  };
  // global acceleration
  Eigen::Vector3d a_g =
      nav_state.pose.so3().matrix() * nav_state.linear_acceleration;
  info->global_linear_acceleration = {
      a_g(0),
      a_g(1),
      a_g(2),
  };
  // local vehicle frame velocity
  info->linear_velocity = {nav_state.linear_speed(0), nav_state.linear_speed(1),
                           nav_state.linear_speed(2)};
  // local vehicle frame acceleration
  info->linear_acceleration = {
      nav_state.linear_acceleration(0),
      nav_state.linear_acceleration(1),
      nav_state.linear_acceleration(2),
  };
  // local vehicle frame angular velocity
  info->angular_velocity = {
      nav_state.angular_speed(0),
      nav_state.angular_speed(1),
      nav_state.angular_speed(2),
  };
  // position uncertainty (car-center coordinate)
  auto rot = nav_state.pose.so3().inverse().matrix();
  Eigen::Matrix3d tra_cov =
      rot * nav_state.pose_cov.topLeftCorner(3, 3) * rot.transpose();
  info->position_std.x = std::sqrt(tra_cov(0, 0));
  info->position_std.y = std::sqrt(tra_cov(1, 1));
  info->position_std.z = std::sqrt(tra_cov(2, 2));
  // and attitude uncertainty
  info->attitude_std.x = std::sqrt(nav_state.pose_cov(3, 3));
  info->attitude_std.y = std::sqrt(nav_state.pose_cov(4, 4));
  info->attitude_std.z = std::sqrt(nav_state.pose_cov(5, 5));

  return LOC_SUCCESS;
}

void LocalizationFrontend::SetCurrentNavStateInfoForVisual() {
  if (!loc_visualizer_) return;
  NavState cur_state;
  {
    std::lock_guard<std::mutex> locker(window_mutex_);
    if (!window_states_.empty()) cur_state = window_states_.back();
  }
  if (cur_state.timestamp == 0) return;

  NavStateInfo nav_state_info;
  ConvertNavStateInfo(cur_state, &nav_state_info);
  loc_visualizer_->SetLocalizationInfo(nav_state_info,
                                       multi_source_nav_status_);
}

void LocalizationFrontend::SaveLocalizationEvalData(
    const NavState& nav_state) const {
  // save nav state
  double timestamp = nav_state.timestamp * kNanoSecToSec;
  evaluator_->WriteResult(timestamp, nav_state.pose.matrix());

  // save nav status
  static auto TypeCast = [](const NavStatus& nav_status) {
    return static_cast<int8_t>(GetNavStatusNum(nav_status));
  };

  NavStatusEvalData eval_data;
  // system nav status
  eval_data.nav_status = static_cast<int8_t>(nav_state.nav_status);
  // multi source nav status
  if (multi_source_nav_status_.size() == 4) {
    eval_data.frontend_status = TypeCast(multi_source_nav_status_[0]);
    eval_data.backend_status = TypeCast(multi_source_nav_status_[1]);
    eval_data.msf_status = TypeCast(multi_source_nav_status_[2]);
    eval_data.output_status = TypeCast(multi_source_nav_status_[3]);
  }
  // backend locator nav status
  if (backends_status_.count(GNSS))
    eval_data.gnss_status = TypeCast(backends_status_.at(GNSS));
  if (backends_status_.count(SMM))
    eval_data.smm_status = TypeCast(backends_status_.at(SMM));

  status_evaluator_->WriteResult(timestamp, eval_data);
}

}  // namespace localization
}  // namespace senseAD
