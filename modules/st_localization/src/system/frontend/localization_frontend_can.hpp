/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>

#include "can/can_locator.hpp"
#include "localization/data_type/base.hpp"
#include "localization/data_type/smart_ptr.hpp"
#include "localization/data_type/vehicle_info.hpp"
#include "localization/localization_param.hpp"
#include "system/localization_frontend.hpp"

namespace senseAD {
namespace localization {

class LocalizationFrontendCAN
    : public LocalizationFrontend,
      public FrontendAdapter<CANLocator, VehicleInfo> {
 public:
  DEFINE_PTR(LocalizationFrontendCAN)

  LocalizationFrontendCAN() = default;
  LocalizationFrontendCAN(const LocalizationFrontendCAN&) = delete;
  LocalizationFrontendCAN& operator=(const LocalizationFrontendCAN&) = delete;
  virtual ~LocalizationFrontendCAN() = default;

 protected:
  // @brief: check whether need system restart
  bool CheckFrontendNeedRestart() { return false; }

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
};

}  // namespace localization
}  // namespace senseAD
