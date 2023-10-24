/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： fusion_center_lite.h
 *   author     ： lilanxing
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <adf-lite/include/base.h>
#include <common_onboard/adapter/onboard_lite/onboard_lite.h>

#include <memory>
#include <string>

#include "modules/location/fusion_center/lib/fusion_center.h"

namespace hozon {
namespace perception {
namespace common_onboard {

using hozon::mp::loc::fc::FusionCenter;

class FusionCenterLite : public OnboardLite {
 public:
  FusionCenterLite() = default;
  ~FusionCenterLite() = default;

  int32_t AlgInit() override;
  void AlgRelease() override;

 private:
  void RegistLog() const;
  void RegistMessageType() const;
  void RegistProcessFunc();
  int32_t OnInsFusion(Bundle* input);

 private:
  std::unique_ptr<FusionCenter> fusion_center_ = nullptr;
};

REGISTER_EXECUTOR_CLASS(FusionCenterLite, FusionCenterLite);

}  // namespace common_onboard
}  // namespace perception
}  // namespace hozon
