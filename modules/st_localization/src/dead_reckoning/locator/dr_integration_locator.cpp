/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Du Jiankui <dujiankui@senseauto.com>
 */

#include "dead_reckoning/locator/dr_integration_locator.hpp"

namespace senseAD {
namespace localization {
namespace dr {

adLocStatus_t DRIntegrationLocator::Init(const LocalizationParam& param) {
  // TODO(xx): implement it!
  return LOC_SUCCESS;
}

adLocStatus_t DRIntegrationLocator::GetState(OdomState* odom_state,
                                             double* confidence) {
  // TODO(xx): implement it!
  return LOC_SUCCESS;
}

OdomStatus DRIntegrationLocator::GetCurrentLocatorStatus() {
  // TODO(xx): implement it!
  return OdomStatus::INVALID;
}

adLocStatus_t DRIntegrationLocator::Process(uint64_t timestamp,
                                            std::shared_ptr<Imu> raw_imu) {
  // TODO(xx): implement it!
  return LOC_SUCCESS;
}

}  // namespace dr
}  // namespace localization
}  // namespace senseAD
