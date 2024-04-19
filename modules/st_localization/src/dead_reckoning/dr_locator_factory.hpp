/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Fang Yu <fangyu1@senseauto.com>
 */
#pragma once

#include <memory>
#include <string>

#include "dead_reckoning/locator/dr_filtering_locator.hpp"
#include "dead_reckoning/locator/dr_integration_locator.hpp"
#include "dead_reckoning/locator/dr_optim_locator.hpp"

namespace senseAD {
namespace localization {
namespace dr {

class DRLocatorFactory {
 public:
  static std::unique_ptr<DRLocator> CreateDRLocator(std::string type) {
    std::unique_ptr<DRLocator> locator;
    if (type == "INTEGRATION") {
      // trivial intergration based DR locator
      locator.reset(new DRIntegrationLocator());
    } else if (type == "FILTERING") {
      // kalman filter based DR locator
      locator.reset(new DRFilteringLocator());
    } else if (type == "OPTIM") {
      // slide window optimization based DR locator
      locator.reset(new DROptimLocator());
    } else {
      LC_LERROR(DR) << "wrong DRLocatorType given: " << type;
      locator = nullptr;
    }
    return locator;
  }
};

}  // namespace dr
}  // namespace localization
}  // namespace senseAD
