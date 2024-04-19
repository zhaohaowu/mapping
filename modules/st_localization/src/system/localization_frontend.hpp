/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <atomic>
#include <deque>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include <boost/circular_buffer.hpp>

#include "common/thread_base.hpp"
#include "initialization/initialization.hpp"  // using InitStage
#include "localization/data_type/base.hpp"
#include "localization/data_type/navstate_info.hpp"
#include "localization/data_type/odomstate_info.hpp"
#include "localization/data_type/smart_ptr.hpp"
#include "localization/localization_param.hpp"

namespace senseAD {
namespace localization {

namespace msf {
class MSFFusion;
}
class INSLocator;
class GNSSLocator;
class EvaluatorLocalization;
class EvaluatorLocalizationStatus;
class EvaluatorGnssStatus;
class EvaluatorMotion;
class LocalizationBackend;
class LocalizationVisualizer;

// struct holds the system localization frontend running in internal thread,
// contains initialize, frontend localize, msf fusion, localization output etc.
class LocalizationFrontend : public ThreadBase {
 public:
  DEFINE_PTR(LocalizationFrontend)

  LocalizationFrontend() = default;
  LocalizationFrontend(const LocalizationFrontend&) = delete;
  LocalizationFrontend& operator=(const LocalizationFrontend&) = delete;
  virtual ~LocalizationFrontend();

  // @brief: frontend init
  adLocStatus_t Init(const LocalizationParam& param);

  adLocStatus_t Restart();

  bool CheckNeedRestart();

  adLocStatus_t SwitchOriginProc();

  bool CheckNeedSwitchOrigin() const;

  // @brief: set/get instance pointer
  void SetMSFFusion(std::shared_ptr<msf::MSFFusion> msf_fusion);
  void SetLocalizationBackends(
      const std::map<LocatorType, std::shared_ptr<LocalizationBackend>>& bes);
  void SetLocalizationVisualizer(
      std::shared_ptr<LocalizationVisualizer> visualizer);
  std::shared_ptr<Initialization> GetInitilizer();

  // @brief: gt locator process for eval
  adLocStatus_t GtLocatorForEval(uint64_t timestamp, const Ins& ins);
  adLocStatus_t GtRtkLocatorForEval(uint64_t timestamp, const Gnss& gnss);

  // @brief: get frontend processed latest time
  uint64_t GetFrontendTime() const;

  // @brief: get gt state
  adLocStatus_t GetGtState(NavState* gt_state, NavState* gt_rtk_state) const;

  // @brief: query frontend pose by time
  adLocStatus_t QueryPoseByTime(uint64_t timestamp, NavState* state) const;

  // @brief: causion! this interface just for publish call
  adLocStatus_t GetNavStateInfo(NavStateInfo* info) const;

  adLocStatus_t GetLatestState(NavState* nav_state) const;

 protected:
  virtual bool CheckFrontendNeedRestart() = 0;

  virtual adLocStatus_t InitLocator() = 0;

  virtual adLocStatus_t InitInitializer() = 0;

  virtual adLocStatus_t InitDataBuffer() = 0;

  virtual adLocStatus_t ResetSubData() = 0;

  virtual adLocStatus_t SwitchOriginSubProc() = 0;

  virtual InitStage Initialize() = 0;

  virtual NavStatus GetCurrentNavStatus(const NavState& nav_state) = 0;

  virtual adLocStatus_t LocatorProcess(NavState* nav_state,
                                       bool* continue_to_process) = 0;

 protected:
  adLocStatus_t OnInitDataBuffer();

  // @brief: thread main
  void Run();

  void FrontendProcess();

  // helper function

  adLocStatus_t InitGtLocatorAndEvaluator();

  adLocStatus_t TimeOrderCheck(uint64_t timestamp);

  int64_t FrameRateCheck(uint64_t timestamp);

  // store and get localization state related

  NavState StoreState(const NavState& newest_nav_state);

  void NavStateInterp(const NavState& s_ns, const NavState& e_ns, double factor,
                      NavState* ns) const;

  adLocStatus_t SmoothNavStateOutput(const NavState& newest_ns,
                                     NavState* smooth_ns) const;

  void SetCurrentNavStateInfoForVisual();

  adLocStatus_t ConvertNavStateInfo(const NavState& nav_state,
                                    NavStateInfo* info) const;

  void SaveLocalizationEvalData(const NavState& nav_state) const;

 protected:
  // Localization config parameters
  LocalizationParam param_;

  // output nav states
  mutable std::mutex window_mutex_;
  boost::circular_buffer<NavState> window_states_;
  mutable std::atomic<uint64_t> last_get_timestamp_{0};  // for loc publish

  // frontend related variable
  std::atomic<uint64_t> frontend_time_{0};  // frontend unique order time
  int64_t frontend_frame_rate_{-1};         // frontend process frame rate
  // for frontend frame rate check
  static constexpr size_t frc_buffer_size_ = 100;
  boost::circular_buffer<uint64_t> frc_timestamp_buffer_;
  int64_t frc_total_count_{0}, frc_total_frame_rates_{0};

  // gt and gnss state for SMM using
  mutable std::mutex gt_state_mutex_, gnss_state_mutex_;
  NavState nav_gt_state_, nav_gnss_state_;

  // GT locator and evaluator for eval
  std::shared_ptr<INSLocator> gt_locator_;
  std::shared_ptr<EvaluatorLocalization> gt_evaluator_;
  std::shared_ptr<EvaluatorMotion> gt_motion_evaluator_;
  std::shared_ptr<EvaluatorLocalization> gt_rtk_evaluator_;
  std::shared_ptr<EvaluatorGnssStatus> gt_rtk_status_evaluator_;

  // nav_state/status evaluator
  std::shared_ptr<EvaluatorLocalization> evaluator_;
  std::shared_ptr<EvaluatorLocalization> init_evaluator_;
  std::shared_ptr<EvaluatorLocalizationStatus> status_evaluator_;
  std::vector<NavStatus> multi_source_nav_status_;
  std::map<LocatorType, NavStatus> backends_status_;

  // external modules
  std::shared_ptr<Initialization> initializer_;  // initializer instance
  std::shared_ptr<msf::MSFFusion> msf_fusion_;   // msf instance, shallow copy
  // backend instances, shallow copy
  std::map<LocatorType, std::shared_ptr<LocalizationBackend>> loc_backends_;
  // localization visualizer instance, shallow copy
  std::shared_ptr<LocalizationVisualizer> loc_visualizer_;

  // for init evaluator
  uint64_t last_init_evaluator_time_{0};
};

template <typename Locator, typename InputData>
class FrontendAdapter {
 public:
  DEFINE_PTR(FrontendAdapter)

  FrontendAdapter() = default;
  virtual ~FrontendAdapter() = default;

  // @brief: init circular buffer capacity
  void InitCapacity() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    data_list_.set_capacity(kDataSizeUpperLimit);
  }

  // @brief: set input sensor data
  void SetInputData(uint64_t timestamp, const InputData& in_data) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    data_list_.push_back(std::make_pair(timestamp, in_data));
  }

  // @brief: clear data buffer
  void ClearData() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    data_list_.clear();
  }

  // @brief: get locator instance
  std::shared_ptr<Locator> GetLocator() const { return locator_; }

 protected:
  static constexpr size_t kDataSizeUpperLimit = 20;

  std::mutex data_mutex_;
  boost::circular_buffer<std::pair<uint64_t, InputData>> data_list_;

  std::shared_ptr<Locator> locator_;  // localtor instance
};

}  // namespace localization
}  // namespace senseAD
