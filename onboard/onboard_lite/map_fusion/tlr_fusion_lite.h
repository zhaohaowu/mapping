/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： tlr_fusion_lite.h
 *   author     ： taoshaoyuan
 *   date       ： 2023.12
 ******************************************************************************/
#pragma once

#include <adf-lite/include/base.h>
#include <depend/nos/x86_2004/include/adf-lite/include/executor.h>
#include <depend/nos/x86_2004/include/adf/include/node_proto_register.h>

#include <memory>
#include <mutex>

#include "modules/map_fusion/include/map_fusion/tlr_fusion.h"

namespace hozon {
namespace perception {
namespace common_onboard {

class TlrFusionLite : public hozon::netaos::adf_lite::Executor {
 public:
  TlrFusionLite() = default;
  ~TlrFusionLite() = default;

  int32_t AlgInit() override;
  void AlgRelease() override;

 private:
  int32_t OnTlr(hozon::netaos::adf_lite::Bundle* input);
  std::shared_ptr<hozon::mp::mf::TlrFusion> tlr_fusion_ = nullptr;
};

REGISTER_ADF_CLASS(TlrFusionLite, TlrFusionLite);

}  // namespace common_onboard
}  // namespace perception
}  // namespace hozon
