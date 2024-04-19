/*
 * Copyright (C) 2023 by SenseTime Group Limited. All rights reserved.
 * Tang Zhuan <tangzhuan@senseauto.com>
 */

#pragma once

#include <memory>
#include <utility>

#include "msf/msf_fusion/msf_fusion.hpp"
#include "msf/msf_fusion/msf_fusion_can/msf_fusion_can.hpp"
#include "msf/msf_fusion/msf_fusion_imu/msf_fusion_imu.hpp"

namespace senseAD {
namespace localization {
namespace msf {

class MSFFusionFactory {
 public:
  template <typename... Args>
  static std::unique_ptr<MSFFusion> CreateMSFFusion(
      LocatorType type, std::shared_ptr<BaseLocator> locator, Args&&... args) {
    std::unique_ptr<MSFFusion> fusion = nullptr;
    if (type == IMU) {
      fusion.reset(new MSFFusionIMU(locator, std::forward<Args>(args)...));
    } else if (type == CAN) {
      fusion.reset(new MSFFusionCAN(locator, std::forward<Args>(args)...));
    }
    return fusion;
  }
};

}  // namespace msf
}  // namespace localization
}  // namespace senseAD
