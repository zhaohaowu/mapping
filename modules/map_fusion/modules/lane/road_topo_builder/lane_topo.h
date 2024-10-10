/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： lane_topo.h
 *   author     ： likuan
 *   date       ： 2024.08
 ******************************************************************************/
#pragma once

#include <depend/common/math/vec2d.h>

#include <algorithm>
#include <cfloat>
#include <deque>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

// #include "modules/map_fusion/base/group.h"
#include "modules/map_fusion/base/interface_option.h"
#include "modules/map_fusion/common/calc_util.h"
#include "modules/map_fusion/common/common_data.h"
#include "modules/map_fusion/data_manager/location_data_manager.h"
#include "modules/map_fusion/modules/lane/road_topo_builder/utils.h"
#include "modules/util/include/util/mapping_log.h"

namespace hozon {
namespace mp {
namespace mf {

class LaneTopoConstruct {
 public:
  LaneTopoConstruct() = default;

  ~LaneTopoConstruct() = default;

  void Init(const LaneFusionProcessOption& conf);

  void Clear();

  void ConstructTopology(std::vector<Group::Ptr>* groups);

 private:
  bool LeftRightTopoProcess(std::vector<Group::Ptr>* groups);

  void ForwardTopoProcess(const Group::Ptr& curr_group,
                          const Group::Ptr& next_group,
                          bool* is_any_next_lane_exist,
                          bool* is_all_next_lane_exist);

  void BackwardTopoProcess(const Group::Ptr& curr_group,
                           const Group::Ptr& next_group,
                           bool* is_any_next_lane_exist,
                           bool* is_all_next_lane_exist);

  void SmoothCenterline(std::vector<Group::Ptr>* groups);

  void SetLaneStatus(std::vector<Group::Ptr>* groups);

  void UpdateLaneBoundaryId(const Group::Ptr& curr_group);

  bool ContainEgoLane(std::vector<Group::Ptr>* groups, int next_grp_index);

  bool IsGroupsNoEgoLane(std::vector<Group::Ptr>* groups, int curr_group_index);

  bool IsGroupNoEgoLane(const Group::Ptr& group);

  void DelLaneNextStrIdInGroup(const Group::Ptr& curr_group);

  void DelLanePrevStrIdInGroup(const Group::Ptr& curr_group);

  void BuildVirtualLaneAfter(const Group::Ptr& curr_group,
                             const Group::Ptr& next_group);

  void BuildVirtualLaneBefore(const Group::Ptr& curr_group,
                              const Group::Ptr& next_group);

  void BuildVirtualLaneLeftRight(const Group::Ptr& curr_group,
                                 const Group::Ptr& next_group);

  void EraseIntersectLane(Group::Ptr curr_group, Group::Ptr next_group);

  bool DistanceInferenceLane(const LineSegment& left_line,
                             const LineSegment& right_line);

  bool Distanceline(const LineSegment& left_line, float line_front_x,
                    float line_front_y);

 private:
  LaneFusionProcessOption conf_;
};

using LaneTopoConstructPtr = std::unique_ptr<LaneTopoConstruct>;

}  // namespace mf
}  // namespace mp
}  // namespace hozon
