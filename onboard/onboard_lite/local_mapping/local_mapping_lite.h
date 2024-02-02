/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: Hozon
 *Date: 2023-09-09
 *****************************************************************************/

#pragma once

#include <memory>

#include "depend/nos/x86_2004/include/adf-lite/include/executor.h"
#include "depend/nos/x86_2004/include/adf/include/node_proto_register.h"
#include "depend/perception-base/base/state_machine/state_machine_info.h"
#include "depend/perception-lib/lib/fault_manager/fault_manager.h"
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

  int32_t OnPerception(adf_lite_Bundle* input);

  int32_t OnPerceptionObj(adf_lite_Bundle* input);

  int32_t OnRunningMode(adf_lite_Bundle* input);

  int32_t Onlocalization(adf_lite_Bundle* input);

  int32_t OnIns(adf_lite_Bundle* input);

  int32_t OnImage(adf_lite_Bundle* input);

  int32_t PublishLocalMap();

  std::shared_ptr<LMapApp> lmap_ = nullptr;
  std::shared_ptr<hozon::mapping::LocalMap> result = nullptr;
};

// REGISTER_ADF_CLASS(LocalMappingOnboard, LocalMappingOnboard);

}  // namespace lm
}  // namespace mp
}  // namespace hozon
