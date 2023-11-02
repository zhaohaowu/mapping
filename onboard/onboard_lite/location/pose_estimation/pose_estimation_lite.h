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

#include "common_onboard/adapter/onboard_lite/onboard_lite.h"
#include "modules/location/pose_estimation/lib/pose_estimation.h"

namespace hozon {
namespace perception {
namespace common_onboard {

using hozon::mp::loc::MapMatching;

class PoseEstimationLite : public OnboardLite {
 public:
  PoseEstimationLite() = default;
  ~PoseEstimationLite() = default;

  int32_t AlgInit() override;
  void AlgRelease() override;

 private:
  void RegistLog() const;
  void RegistMessageType() const;
  void RegistProcessFunc();

  int32_t OnIns(Bundle* input);
  //  void OnLocation(const std::shared_ptr<location::HafLocation> &msg);
  int32_t OnPerception(Bundle* input);
  // void OnMarkPole(const std::shared_ptr<::perception::Roadmarking> &msg);
  int32_t OnPoseEstimation(Bundle* input);

 private:
  std::unique_ptr<MapMatching> pose_estimation_ = nullptr;
};

REGISTER_EXECUTOR_CLASS("PoseEstimationLite", PoseEstimationLite);

}  // namespace common_onboard
}  // namespace perception
}  // namespace hozon
