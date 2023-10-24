/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： location.h
 *   author     ： lilanxing
 *   date       ： 2023.10
 ******************************************************************************/

#pragma once

#include <memory>

#include "modules/location/coord_adapter/lib/coord_adapter.h"
#include "modules/location/dr_fusion/lib/dr_fusion.h"
#include "modules/location/fusion_center/lib/fusion_center.h"
#include "modules/location/ins_fusion/lib/ins_fusion.h"
// #include "modules/location/pose_estimation/lib/pose_estimation.h"

namespace hozon {
namespace mp {
namespace loc {

using hozon::mp::loc::ca::CoordAdapter;
using hozon::mp::loc::DrFusion;
using hozon::mp::loc::InsFusion;
using hozon::mp::loc::fc::FusionCenter;

/**
 * @brief Class assemble all location modules for receiving sensors
 *        and sending poses.
 */
class Localization {
 public:
  Localization() = default;
  ~Localization() = default;

  bool Init();

  // receive origin dr
  void OnDr(const hozon::dead_reckoning::DeadReckoning& dr_node);
  void OnOriginIns(const hozon::soc::ImuIns& origin_ins);
  void OnInspva(const hozon::localization::HafNodeInfo& inspva_node);
  void OnImu(const hozon::soc::ImuIns& imuins);
  void OnLocalMap(const hozon::mapping::LocalMap& local_map);
  bool GetCurrentLocalization(
      hozon::localization::Localization* const location);

 private:
  void DrFusionPoseProcess(int dr_state);

 private:
  std::unique_ptr<CoordAdapter> coord_adapter_ = nullptr;
  std::unique_ptr<DrFusion> dr_fusion_ = nullptr;
  std::unique_ptr<InsFusion> ins_fusion_ = nullptr;
  std::unique_ptr<FusionCenter> fc_ = nullptr;
};

}  // namespace loc
}  // namespace mp
}  // namespace hozon
