/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： dr_fusion.h
 *   author     ： nihongjie
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <memory>
#include <string>

#include "adf-lite/include/base.h"
#include "common_onboard/adapter/onboard_lite/onboard_lite.h"
#include "modules/location/dr_fusion/lib/dr_fusion.h"

namespace hozon {
namespace perception {
namespace common_onboard {

class DrFusionLite : public OnboardLite {
 public:
  DrFusionLite() = default;
  ~DrFusionLite() = default;
  int32_t AlgInit() override;
  void AlgRelease() override {}

 private:
  // recieve in-process data and interprocess data
  int32_t send_dr(Bundle* input);
  // send in-process data and interprocess data
  int32_t receive_dr(Bundle* input);
  int32_t receive_inspva(Bundle* input);

 private:
  std::unique_ptr<hozon::mp::loc::DrFusion> dr_fusion_ = nullptr;
};

REGISTER_EXECUTOR_CLASS("DrFusionLite", DrFusionLite);
}  //  namespace common_onboard
}  //  namespace perception
}  //  namespace hozon
