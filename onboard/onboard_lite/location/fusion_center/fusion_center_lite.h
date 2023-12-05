/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： fusion_center_lite.h
 *   author     ： lilanxing
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <adf-lite/include/base.h>
// #include <common_onboard/adapter/onboard_lite/onboard_lite.h>

#include <memory>
#include <string>

#include "depend/nos/x86_2004/include/adf-lite/include/executor.h"
#include "depend/nos/x86_2004/include/adf/include/node_proto_register.h"
#include "modules/location/coord_adapter/lib/coord_adapter.h"
#include "modules/location/fusion_center/lib/fusion_center.h"

namespace hozon {
namespace perception {
namespace common_onboard {

using hozon::mp::loc::ca::CoordAdapter;
using hozon::mp::loc::fc::FusionCenter;
using hozon::netaos::adf_lite::Bundle;

class FusionCenterLite : public hozon::netaos::adf_lite::Executor {
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
  int32_t OnDrFusion(Bundle* input);
  int32_t OnLocalMap(Bundle* input);
  int32_t OnPoseEstimation(Bundle* input);

 private:
  std::unique_ptr<FusionCenter> fusion_center_ = nullptr;
  std::unique_ptr<CoordAdapter> coord_adapter_ = nullptr;
  bool init_dr_ = false;
};

REGISTER_ADF_CLASS(FusionCenterLite, FusionCenterLite);

}  // namespace common_onboard
}  // namespace perception
}  // namespace hozon
