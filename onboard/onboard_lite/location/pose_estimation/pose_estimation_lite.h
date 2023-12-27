/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： pose_estimation_lite.h
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <adf-lite/include/base.h>

#include <memory>
#include <string>

#include "depend/nos/x86_2004/include/adf-lite/include/executor.h"
#include "depend/nos/x86_2004/include/adf/include/node_proto_register.h"
#include "modules/location/pose_estimation/lib/pose_estimation.h"
#include "onboard/onboard_lite/phm_comment_lite/proto/running_mode.pb.h"

namespace hozon {
namespace perception {
namespace common_onboard {

using hozon::mp::loc::MapMatching;
using hozon::netaos::adf_lite::Bundle;

class PoseEstimationLite : public hozon::netaos::adf_lite::Executor {
 public:
  PoseEstimationLite() = default;
  ~PoseEstimationLite() = default;

  int32_t AlgInit() override;
  void AlgRelease() override;

 private:
  void RegistMessageType() const;
  void RegistProcessFunc();

  int32_t OnIns(Bundle* input);
  //  void OnLocation(const std::shared_ptr<location::HafLocation> &msg);
  int32_t OnPerception(Bundle* input);
  // void OnMarkPole(const std::shared_ptr<::perception::Roadmarking> &msg);
  int32_t OnPoseEstimation(Bundle* input);
  int32_t OnRunningMode(Bundle* input);

 private:
  std::unique_ptr<MapMatching> pose_estimation_ = nullptr;
};

REGISTER_ADF_CLASS(PoseEstimationLite, PoseEstimationLite);

}  // namespace common_onboard
}  // namespace perception
}  // namespace hozon
