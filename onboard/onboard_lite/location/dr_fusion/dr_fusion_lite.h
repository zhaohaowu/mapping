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
#include "depend/nos/x86_2004/include/adf-lite/include/executor.h"
#include "depend/nos/x86_2004/include/adf/include/node_proto_register.h"
#include "modules/location/dr_fusion/lib/dr_fusion.h"

namespace hozon {
namespace perception {
namespace common_onboard {

using hozon::netaos::adf_lite::Bundle;
class DrFusionLite : public hozon::netaos::adf_lite::Executor {
 public:
  DrFusionLite() = default;
  ~DrFusionLite() = default;
  int32_t AlgInit() override;
  void AlgRelease() override;

 private:
  // recieve in-process data and interprocess data
  int32_t send_dr(Bundle* input);
  // send in-process data and interprocess data
  int32_t receive_dr(Bundle* input);
  int32_t receive_ins_fusion(Bundle* input);

 private:
  std::unique_ptr<hozon::mp::loc::DrFusion> dr_fusion_ = nullptr;
};

REGISTER_ADF_CLASS(DrFusionLite, DrFusionLite);
}  //  namespace common_onboard
}  //  namespace perception
}  //  namespace hozon
