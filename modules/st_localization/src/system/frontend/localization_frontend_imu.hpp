/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <atomic>
#include <list>
#include <map>
#include <memory>
#include <string>
#include <utility>

#include "imu/imu_locator.hpp"
#include "localization/data_type/base.hpp"
#include "localization/data_type/imu.hpp"
#include "localization/data_type/ins.hpp"
#include "localization/data_type/smart_ptr.hpp"
#include "localization/data_type/vehicle_info.hpp"
#include "localization/localization_param.hpp"
#include "system/localization_frontend.hpp"

namespace senseAD {
namespace localization {

class HybridStaticDetector;
class INSLocator;
class EvaluatorStatic;

class LocalizationFrontendIMU : public LocalizationFrontend,
                                public FrontendAdapter<IMULocator, Imu> {
 public:
  DEFINE_PTR(LocalizationFrontendIMU)

  LocalizationFrontendIMU() = default;
  LocalizationFrontendIMU(const LocalizationFrontendIMU&) = delete;
  LocalizationFrontendIMU& operator=(const LocalizationFrontendIMU&) = delete;
  virtual ~LocalizationFrontendIMU() = default;

  void SetInsData(uint64_t timestamp, const Ins& ins);

  void SetCanData(uint64_t timestamp, const VehicleInfo& can);

 protected:
  // @brief: check whether need system restart
  bool CheckFrontendNeedRestart() final;

  adLocStatus_t InitLocator() final;

  adLocStatus_t InitInitializer() final;

  adLocStatus_t InitDataBuffer() final;

  void SetThreadName() final;

  adLocStatus_t ResetSubData() final;

  adLocStatus_t SwitchOriginSubProc() final;

  InitStage Initialize() final;

  NavStatus GetCurrentNavStatus(const NavState& nav_state) final;

  adLocStatus_t LocatorProcess(NavState* nav_state,
                               bool* continue_to_process) final;

 private:
  adLocStatus_t InsAidedFastInitialization();
  adLocStatus_t FastInitialization();

  NavStatus CheckFrontendStatus(std::string* reason);
  NavStatus CheckBackendStatus(std::string* reason);
  NavStatus CheckMSFStatus(const NavState& nav_state, std::string* reason);
  NavStatus CheckOutputStatus(std::string* reason);

  void AddZUPTConstraints(const NavState& nav_state, bool parking = false);

 private:
  // cnt of fatal accuracy
  uint64_t fatal_accuracy_cnt_{0};
  std::atomic<NavStatus> latest_nav_status_{NavStatus::INVALID};

  // record last init state timestamp for disorder check
  std::atomic<uint64_t> last_init_state_time_{0};

  // fast initialization related variable
  std::shared_ptr<INSLocator> ins_init_locator_;  // ins locator instance
  // buffer ins datas, for fast initialization
  std::mutex ins_mutex_;
  boost::circular_buffer<std::pair<uint64_t, Ins>> ins_data_list_;

  // for localization status check
  std::map<NavStatus, int64_t> status_counter_{{NavStatus::FATAL_ACCURACY, 0},
                                               {NavStatus::LOW_ACCURACY, 0},
                                               {NavStatus::MID_ACCURACY, 0},
                                               {NavStatus::HIGH_ACCURACY, 0}};

  // static detect related variable
  bool update_fix_state_{true};
  NavState fixed_nav_state_;
  uint64_t last_zupt_timestamp_{0};
  std::shared_ptr<HybridStaticDetector>
      static_detector_;  // static detect instance
  std::shared_ptr<EvaluatorStatic>
      static_evaluator_;  // static evaluator instance
};

}  // namespace localization
}  // namespace senseAD
