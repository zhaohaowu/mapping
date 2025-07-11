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
#include "modules/location/pose_estimation/lib/pose_estimation.h"

namespace hozon {
namespace perception {
namespace common_onboard {

using hozon::mp::loc::pe::PoseEstimation;
using hozon::netaos::adf_lite::Bundle;

class PoseEstimationLite : public hozon::netaos::adf_lite::Executor {
 public:
  PoseEstimationLite() = default;
  ~PoseEstimationLite() = default;

  int32_t AlgInit() override;
  void AlgRelease() override;

 private:
  void ExtractCmParameter(const std::string& yamlpath);
  void RegistMessageType() const;
  void RegistProcessFunc();

  int32_t OnIns(Bundle* input);
  int32_t OnLocation(Bundle* input);
  int32_t OnPerception(Bundle* input);
  int32_t OnPoseEstimation(Bundle* input);
  int32_t OnRunningMode(Bundle* input);

 private:
  std::unique_ptr<PoseEstimation> pose_estimation_ = nullptr;
  std::string kPerceptionTopic_;
  std::string kinsFusionTopic_;
  std::string kFcTopic_;
  std::string kRunningModeTopic_;
};

REGISTER_ADF_CLASS(PoseEstimationLite, PoseEstimationLite);

}  // namespace common_onboard
}  // namespace perception
}  // namespace hozon
