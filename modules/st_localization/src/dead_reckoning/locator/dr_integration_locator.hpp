/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Du Jiankui <dujiankui@senseauto.com>
 */

#pragma once

#include <memory>

#include "dead_reckoning/dead_reckoning_locator.hpp"

namespace senseAD {
namespace localization {
namespace dr {

class DRIntegrationLocator : public DRLocator {
 public:
  DRIntegrationLocator() = default;
  virtual ~DRIntegrationLocator() = default;

  adLocStatus_t Init(const LocalizationParam& param) final;

  adLocStatus_t GetState(OdomState* odom_state,
                         double* confidence = nullptr) final;

  OdomStatus GetCurrentLocatorStatus() final;

  adLocStatus_t Process(uint64_t timestamp, std::shared_ptr<Imu> raw_imu) final;

 private:
};

}  // namespace dr
}  // namespace localization
}  // namespace senseAD
