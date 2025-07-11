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
#include "modules/laneline_postprocess/app/laneline_postprocess.h"
#include "modules/laneline_postprocess/app/roadedge_postprocess.h"
#include "modules/laneline_postprocess/lib/laneline/tracker/bev_lane_tracker/datalogger/load_data_singleton.h"
// #include
// "perception-common-onboard/common_onboard/adapter/onboard_lite/onboard_lite.h"
#include "perception-lib/lib/location_manager/location_manager.h"

using LocationManager = hozon::perception::lib::LocationManager;
using Location = hozon::perception::base::Location;

namespace hozon {
namespace mp {
namespace environment_onboard {

using adf_lite_Bundle = hozon::netaos::adf_lite::Bundle;
namespace perception_base = hozon::perception::base;
namespace perception_lib = hozon::perception::lib;

// namespace perception_env = hozon::perception::;

class EnvBevfusionOnboard : public hozon::netaos::adf_lite::Executor {
 public:
  EnvBevfusionOnboard() = default;
  ~EnvBevfusionOnboard() override = default;

  int32_t AlgInit() override;
  void AlgRelease() override;
  // 接收running_mode通道，用于行泊切换
  int32_t OnRunningMode(adf_lite_Bundle* input);
  int32_t ReceiveDetectLaneLine(adf_lite_Bundle* input);

  int32_t ReceiveDr(adf_lite_Bundle* input);

 private:
  std::unique_ptr<environment::LanePostProcess> lane_postprocessor_;
  std::unique_ptr<environment::RoadEdgePostProcess> roadedge_postprocessor_;
  std::shared_ptr<perception_base::Location> location_msg_;

  double frame_proc_maxtime_ = 0.0;
  int frame_proc_num = 0;
  int frame_overtime_nums = 0;
};

REGISTER_ADF_CLASS(EnvBevfusionOnboard, EnvBevfusionOnboard);

}  // namespace environment_onboard
}  // namespace mp
}  // namespace hozon
