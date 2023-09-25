/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： ins_fusion.h
 *   author     ： nihongjie
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <memory>
#include <string>

#include "adf-lite/include/base.h"
#include "common_onboard/adapter/onboard_lite/onboard_lite.h"
#include "modules/location/ins_fusion/lib/ins_fusion.h"

namespace hozon {
namespace perception {
namespace common_onboard {

class InsFusionLite : public OnboardLite {
 public:
  InsFusionLite() = default;
  ~InsFusionLite() = default;
  int32_t AlgInit() override;
  void AlgRelease() override {}

 private:
  // recieve in-process data and interprocess data
  int32_t send_ins(Bundle* input);
  // send in-process data and interprocess data
  int32_t receive_ins(Bundle* input);
  int32_t receive_inspva(Bundle* input);

 private:
  std::unique_ptr<hozon::mp::loc::InsFusion> ins_fusion_ = nullptr;
};

REGISTER_EXECUTOR_CLASS("InsFusionLite", InsFusionLite);
}  //  namespace common_onboard
}  //  namespace perception
}  //  namespace hozon
