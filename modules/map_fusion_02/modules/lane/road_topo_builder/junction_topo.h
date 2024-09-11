/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： junction_topo.h
 *   author     ： likuan
 *   date       ： 2024.08
 ******************************************************************************/
#pragma once

#include <depend/common/math/vec2d.h>

#include <Eigen/Core>
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "Eigen/src/Core/Matrix.h"
#include "modules/map_fusion_02/base/element_base.h"
#include "modules/map_fusion_02/base/group.h"
#include "modules/map_fusion_02/base/interface_option.h"
#include "modules/map_fusion_02/base/junction.h"
#include "modules/map_fusion_02/common/calc_util.h"
#include "modules/map_fusion_02/common/common_data.h"
#include "modules/map_fusion_02/data_manager/junction_status_manager.h"
#include "modules/map_fusion_02/modules/lane/road_topo_builder/utils.h"
#include "modules/util/include/util/mapping_log.h"

namespace hozon {
namespace mp {
namespace mf {

class JunctionTopoConstruct {
 public:
  JunctionTopoConstruct() = default;

  ~JunctionTopoConstruct() = default;

  void Init(const LaneFusionProcessOption& conf);

  void Clear();

  void ConstructTopology(double stamp, std::vector<Group::Ptr>* groups,
                         const std::shared_ptr<std::vector<KinePosePtr>>& path,
                         const KinePosePtr& curr_pose);

 private:
  void UpdatePathInCurrPose(const std::vector<KinePosePtr>& path,
                            const KinePose& curr_pose);

  void FindSatisefyNextLane(const Group::Ptr& next_group, float dist_to_slice,
                            std::vector<Lane::Ptr>* satisfy_next_lane);

  void FindBestNextLane(const Group::Ptr& next_group, float dist_to_slice,
                        Lane::Ptr* best_next_lane);

  void GenerateLane(const Lane::Ptr& lane_next,
                    const Lane::Ptr& transition_lane);

  void GenerateAllSatisfyTransitionLane(
      const Lane::Ptr& lane_in_curr, std::vector<Lane::Ptr>* virtual_lanes,
      const std::vector<Lane::Ptr>& history_satisfy_lane_,
      float dist_to_next_group_slice);

  void BuildVirtualGroup(const std::vector<Lane::Ptr>& virtual_lanes,
                         std::vector<Group::Ptr>* group_virtual, double stamp);

  void GenerateTransitionLaneToBefore(const Lane::Ptr& lane_in_curr,
                                      const Lane::Ptr& transition_lane);

  void GenerateTransitionLaneToAfter(const Lane::Ptr& lane_in_curr,
                                     const Lane::Ptr& lane_in_next,
                                     const Lane::Ptr& transition_lane);

  void GenerateTransitionLaneToPo(const Lane::Ptr& lane_in_curr,
                                  const Lane::Ptr& lane_in_next,
                                  const Lane::Ptr& transition_lane, int type);

  void GenerateTransitionLaneGeo(const Lane::Ptr& lane_in_curr,
                                 const Lane::Ptr& lane_in_next,
                                 const Lane::Ptr& transition_lane);

  void FindNearestLaneToHisVehiclePosition(const Group::Ptr& curr_group,
                                           Lane::Ptr* ego_curr_lane);

  void FindGroupNextLane(const Group::Ptr& curr_group,
                         const Group::Ptr& next_group);

  void NextGroupLaneConnect(
      const Group::Ptr& curr_group, const Group::Ptr& next_group,
      const std::map<int, std::vector<int>>& curr_group_next_lane);

  void BuildVirtualGroup2(const std::vector<Lane::Ptr>& virtual_lanes,
                          std::vector<Group::Ptr>* group_virtual, double stamp);

  void AvoidSplitMergeLane(std::vector<Group::Ptr>* groups);

  void ErasePrevRelation(const Group::Ptr& curr_group, int curr_erase_index,
                         const Group::Ptr& next_group, int next_index);

 private:
  const float kOffsetThreshold = 3.5 * 0.7;  // 0.7个3.5米车道宽度
  float incross_before_virtual_lane_length_;

  LaneFusionProcessOption conf_;
  Lane::Ptr history_best_lane_ = nullptr;
  Lane::Ptr ego_curr_lane_ = nullptr;
  std::vector<Pose> path_in_curr_pose_;

  // isCross变量
  bool is_connect_ = false;
  int next_lane_left_ = -1000;
  int next_lane_right_ = -1000;
  Eigen::Vector3f along_path_dis_;
  std::set<std::string>
      next_satisefy_lane_seg_;  // 记录路口后满足连接条件的所有lane名称
};

using JunctionTopoConstructPtr = std::unique_ptr<JunctionTopoConstruct>;

}  // namespace mf
}  // namespace mp
}  // namespace hozon
