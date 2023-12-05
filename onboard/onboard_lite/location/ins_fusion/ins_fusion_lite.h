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
#include "depend/nos/x86_2004/include/adf-lite/include/executor.h"
#include "depend/nos/x86_2004/include/adf/include/node_proto_register.h"
#include "modules/location/ins_fusion/lib/ins_fusion.h"

namespace hozon {
namespace perception {
namespace common_onboard {

using hozon::netaos::adf_lite::Bundle;
using hozon::netaos::adf_lite::g_class_loader;

class InsFusionLite : public hozon::netaos::adf_lite::Executor {
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

REGISTER_ADF_CLASS(InsFusionLite, InsFusionLite);
}  //  namespace common_onboard
}  //  namespace perception
}  //  namespace hozon
