/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: Hozon
 *Date: 2023-09-09
 *****************************************************************************/

#pragma once

#include <memory>

#include "depend/nos/x86_2004/include/adf-lite/include/executor.h"
#include "depend/nos/x86_2004/include/adf/include/node_proto_register.h"
#include "modules/local_mapping/local_mapping.h"

namespace hozon {
namespace perception {
namespace common_onboard {

using hozon::mp::lm::LMapApp;
using hozon::netaos::adf_lite::Bundle;

class LocalMappingOnboard : public hozon::netaos::adf_lite::Executor {
 public:
  LocalMappingOnboard() = default;
  ~LocalMappingOnboard() = default;

  int32_t AlgInit() override;
  void AlgRelease() override;

  int32_t OnLaneLine(Bundle* input);

  int32_t OnDr(Bundle* input);

  // int32_t OnLocation(Bundle* input);

  int32_t OnIns(Bundle* input);

  int32_t OnImage(Bundle* input);

  // int32_t OnRoadEdge(Bundle* input);

  int32_t LocalMapPublish(Bundle* output);

  int32_t LocalMapLocationPublish(Bundle* output);

 private:
  std::shared_ptr<LMapApp> lmap_;
};

// REGISTER_ADF_CLASS(LocalMappingOnboard, LocalMappingOnboard);

}  // namespace common_onboard
}  // namespace perception
}  // namespace hozon
