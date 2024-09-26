/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： fusion_center_lite.h
 *   author     ： lilanxing
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <adf-lite/include/base.h>

#include <memory>

#include "depend/nos/x86_2004/include/adf-lite/include/executor.h"
#include "modules/location/fusion_center/lib/fusion_center.h"

namespace hozon {
namespace mp {
namespace loc {

using hozon::mp::loc::fc::FusionCenter;
using hozon::netaos::adf_lite::Bundle;

class FusionCenterLite : public hozon::netaos::adf_lite::Executor {
 public:
  FusionCenterLite() = default;
  ~FusionCenterLite() override = default;

  int32_t AlgInit() override;
  void AlgRelease() override;

 private:
  void RegistMessageType() const;
  void RegistProcessFunc();
  int32_t OnInsFusion(Bundle* input);
  int32_t OnDr(Bundle* input);
  int32_t OnImu(Bundle* input);
  int32_t OnChassis(Bundle* input);
  int32_t OnMm(Bundle* input);
  int32_t OnRunningMode(Bundle* input);

 private:
  std::unique_ptr<FusionCenter> fusion_center_ = nullptr;
};

REGISTER_ADF_CLASS(FusionCenterLite, FusionCenterLite);

}  // namespace loc
}  // namespace mp
}  // namespace hozon
