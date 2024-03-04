/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: Hozon
 *Date: 2023-09-09
 *****************************************************************************/

#pragma once

#include <memory>

#include "adf-lite/include/base.h"
#include "depend/nos/x86_2004/include/adf-lite/include/executor.h"
#include "depend/nos/x86_2004/include/adf/include/node_proto_register.h"
#include "depend/perception-base/base/state_machine/state_machine_info.h"
#include "depend/perception-lib/lib/fault_manager/fault_manager.h"
#include "modules/local_mapping/lib/laneline/tracker/bev_lane_tracker/datalogger/load_data_singleton.h"
#include "modules/local_mapping/local_mapping.h"
// #include
// "perception-common-onboard/common_onboard/adapter/onboard_lite/onboard_lite.h"
#include "depend/perception-base/base/frame/measurement_frame.h"
#include "onboard/onboard_lite/local_mapping/measurement_message.h"

namespace hozon {
namespace mp {
namespace lm {

using adf_lite_Bundle = hozon::netaos::adf_lite::Bundle;
namespace perception_base = hozon::perception::base;
namespace perception_lib = hozon::perception::lib;

class LocalMappingOnboard : public hozon::netaos::adf_lite::Executor {
 public:
  LocalMappingOnboard() = default;
  ~LocalMappingOnboard() override = default;

  int32_t AlgInit() override;
  void AlgRelease() override;

  int32_t OnRunningMode(adf_lite_Bundle* input);

  int32_t OnDr(adf_lite_Bundle* input);

  int32_t Onlocalization(adf_lite_Bundle* input);

  int32_t OnIns(adf_lite_Bundle* input);

  int32_t OnImage(adf_lite_Bundle* input);

  int32_t OnPerception(adf_lite_Bundle* input);

  int32_t PublishPostLaneLine(
      std::shared_ptr<const hozon::perception::measurement::MeasurementPb>
          pbdata,
      std::shared_ptr<const perception_base::FusionFrame> fusion_msg);

  int32_t PublishLocalMap();

 private:
  std::shared_ptr<LMapApp> lmap_ = nullptr;
};

// REGISTER_ADF_CLASS(LocalMappingOnboard, LocalMappingOnboard);

}  // namespace lm
}  // namespace mp
}  // namespace hozon
