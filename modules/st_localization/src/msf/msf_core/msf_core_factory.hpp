/*
 * Copyright (C) 2023 by SenseTime Group Limited. All rights reserved.
 * Tang Zhuan <tangzhuan@senseauto.com>
 */

#pragma once

#include <memory>
#include <utility>

#include "msf/msf_core/msf_core.hpp"
#include "msf/msf_core/msf_core_can/msf_core_can.hpp"
#include "msf/msf_core/msf_core_imu/msf_core_imu.hpp"

namespace senseAD {
namespace localization {
namespace msf {

class MSFCoreFactory {
 public:
  template <typename T, typename... Args>
  static std::unique_ptr<T> CreateMSFCore(LocatorType type,
                                          std::shared_ptr<BaseLocator> locator,
                                          Args&&... args) {
    std::unique_ptr<T> core = nullptr;
    if (type == IMU) {
      core.reset(new MSFCoreIMU(locator, std::forward<Args>(args)...));
    }
    return core;
  }
  template <typename T, typename... Args>
  static std::unique_ptr<T> CreateMSFCore(
      LocatorType type, std::shared_ptr<BaseLocator> locator) {
    std::unique_ptr<T> core = nullptr;
    if (type == CAN) {
      core.reset(new MSFCoreCAN(locator));
    }
    return core;
  }
};

}  // namespace msf
}  // namespace localization
}  // namespace senseAD
