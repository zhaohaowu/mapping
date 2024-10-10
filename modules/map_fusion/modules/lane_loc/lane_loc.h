/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： lane_loc.h
 *   author     ： zhaohaowu
 *   date       ： 2024.07
 ******************************************************************************/
#pragma once
#include <memory>
#include <vector>

#include "Eigen/Dense"
#include "Eigen/src/Geometry/Transform.h"
#include "map/hdmap/hdmap_common.h"
#include "modules/map_fusion/modules/lane_loc/base_lane_loc.h"
#include "modules/map_fusion/modules/lane_loc/detect_turn_state.h"
#include "modules/map_fusion/modules/lane_loc/fusion_lane_loc.h"
#include "modules/map_fusion/modules/lane_loc/measure_lane_loc.h"

namespace hozon {
namespace mp {
namespace mf {
namespace lane_loc {

class LaneLoc {
 public:
  LaneLoc();
  ~LaneLoc() = default;
  bool UpdateGroups(Groups* groups);
  std::shared_ptr<Eigen::Vector3d> GetRefPoint(
      const HafNodeInfoConstPtr& ins_ptr);
  SectionPtr GetPerSection(const Groups& groups);
  std::vector<hdmap::LaneInfoConstPtr> GetLdmapLanes(
      const HafNodeInfoConstPtr& ins_pose_ptr);
  SectionPtr GetMapSection(
      const std::vector<hdmap::LaneInfoConstPtr>& ldmap_lanes,
      const Eigen::Vector3d& ref_point, const Eigen::Affine3d& T_V_W);
  void FilterPerSection(Section* per_section_ptr);
  LaneLocInfo GetLaneLocInfo(int fusion_lane_index,
                             const SectionPtr& map_section_ptr,
                             const Eigen::Vector3d& ref_point,
                             const Eigen::Affine3d& T_V_W);

 private:
  std::shared_ptr<MeasureLaneLoc> measure_lane_loc_ptr_ = nullptr;
  std::shared_ptr<DetectTurnState> detect_turn_state_ptr_ = nullptr;
  std::shared_ptr<FusionLaneLoc> fusion_lane_loc_ptr_ = nullptr;
};

}  // namespace lane_loc
}  // namespace mf
}  // namespace mp
}  // namespace hozon

using LaneLocPtr = std::unique_ptr<hozon::mp::mf::lane_loc::LaneLoc>;
