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
namespace mp {
namespace lm {

using adf_lite_Bundle = hozon::netaos::adf_lite::Bundle;

class LocalMappingOnboard : public hozon::netaos::adf_lite::Executor {
 public:
  LocalMappingOnboard() = default;
  ~LocalMappingOnboard() override = default;

  int32_t AlgInit() override;
  void AlgRelease() override;

  int32_t OnLaneLine(adf_lite_Bundle* input);

  // int32_t OnDr(adf_lite_Bundle* input);

  int32_t Onlocalization(adf_lite_Bundle* input);

  int32_t OnIns(adf_lite_Bundle* input);

  int32_t OnImage(adf_lite_Bundle* input);

  // int32_t OnRoadEdge(Bundle* input);

  int32_t LocalMapPublish(adf_lite_Bundle* output);

  int32_t LocalMapLocationPublish(adf_lite_Bundle* output);

 private:
  std::shared_ptr<LMapApp> lmap_ = nullptr;
  std::shared_ptr<hozon::mapping::LocalMap> result = nullptr;
};

// REGISTER_ADF_CLASS(LocalMappingOnboard, LocalMappingOnboard);

}  // namespace lm
}  // namespace mp
}  // namespace hozon
