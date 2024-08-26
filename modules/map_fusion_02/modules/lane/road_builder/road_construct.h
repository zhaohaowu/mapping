/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： lane_construct.h
 *   author     ： cuijiayu
 *   date       ： 2024.08
 ******************************************************************************/
#pragma once

#include <depend/common/math/vec2d.h>

#include <algorithm>
#include <deque>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/map_fusion_02/base/interface_option.h"
#include "modules/map_fusion_02/base/processor.h"
#include "modules/map_fusion_02/common/calc_util.h"
#include "modules/map_fusion_02/common/common_data.h"
#include "modules/map_fusion_02/data_manager/location_data_manager.h"
#include "modules/map_fusion_02/modules/lane/road_builder/broken_point_search.h"
#include "modules/util/include/util/mapping_log.h"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d.hpp"

using hozon::common::math::Vec2d;

namespace hozon {
namespace mp {
namespace mf {

class RoadConstruct : ProcessorBase {
 public:
  RoadConstruct() = default;

  ~RoadConstruct() = default;

  bool Init(const LaneFusionProcessOption& conf);

  bool ConstructRoad(const std::vector<CutPoint>& cutpoints,
                     std::deque<Line::Ptr> lines,
                     const std::shared_ptr<std::vector<KinePosePtr>>& path,
                     const KinePosePtr& curr_pose,
                     const ElementMap::Ptr& ele_map);

  std::vector<Eigen::Vector3f> GetDistPoints();

  std::vector<Group::Ptr> GetGroups();

  void Clear() override;

 private:
  void BuildKDtrees(std::deque<Line::Ptr>* lines);

  void UpdatePathInCurrPose(const std::vector<KinePosePtr>& path,
                            const KinePose& curr_pose);

  float DistByKDtree(const Eigen::Vector3f& ref_point, const LineSegment& line);

  float GetDistPointLane(const Eigen::Vector3f& point_a,
                         const Eigen::Vector3f& point_b,
                         const Eigen::Vector3f& point_c);

  void FitLaneline(const ElementMap::Ptr& ele_map, int id_1, int id_2,
                   int near_line);

  void BuildGroups(const std::vector<CutPoint>& cutpoints,
                   std::deque<Line::Ptr>* lines, const ElementMap::Ptr& ele_map,
                   std::vector<Group::Ptr>* groups);

  void CreatGroupsFromCutPoints(const std::vector<CutPoint>& cutpoints,
                                std::vector<Group::Ptr>* groups);

  void SplitPtsToGroup(std::deque<Line::Ptr>* lines,
                       const ElementMap::Ptr& ele_map,
                       std::vector<Group::Ptr>* groups);

  void SplitLinesToGroup(std::deque<Line::Ptr>* lines, const Group::Ptr& grp);

  void SplitOccsToGroup(const ElementMap::Ptr& ele_map, const Group::Ptr& grp);

  void SplitModelEdgesToGroup(const ElementMap::Ptr& ele_map,
                              const Group::Ptr& grp);

  void GenRoadEdges(std::vector<Group::Ptr>* groups);

  void GenLanesInGroups(std::vector<Group::Ptr>* groups,
                        const ElementMap::Ptr& ele_map, double stamp);

  void GenGroupAllLanes(const Group::Ptr& grp);

  void FilterGroupBadLane(const Group::Ptr& grp);

  void GenGroupName(const Group::Ptr& grp, int grp_id, double stamp);

  void EgoLineTrajectory(const Group::Ptr& grp, const ElementMap::Ptr& ele_map);

  void SetBrokenId(std::vector<Group::Ptr>* groups);

  void GenLanesInGroups(std::vector<Group::Ptr>* groups,
                        std::map<Id, OccRoad::Ptr> occ_roads, double stamp);

  void CollectGroupPossibleLanes(const Group::Ptr& grp,
                                 std::vector<Lane::Ptr>* possible_lanes);

  bool FilterGroupBadLane(const std::vector<Lane::Ptr>& possible_lanes,
                          const Group::Ptr& grp);

  bool MatchLRLane(const Group::Ptr& grp);

  bool OptiPreNextLaneBoundaryPoint(std::vector<Group::Ptr>* groups);

  bool GenLaneCenterLine(std::vector<Group::Ptr>* groups);

  int FindEgoGroup(std::vector<Group::Ptr>* groups);

  void EraseEgoGroupWithNoEgoLane(std::vector<Group::Ptr>* groups);

  void FitCenterLine(const Lane::Ptr& lane);

  void RemoveNullGroup(std::vector<Group::Ptr>* groups);

  void SmoothCenterline(std::vector<Group::Ptr>* groups);

  std::vector<Point> SigmoidFunc(const std::vector<Point>& centerline,
                                 float sigma);

 private:
  LaneFusionProcessOption conf_;
  std::map<int, std::shared_ptr<cv::flann::Index>> KDTrees_;
  std::map<int, std::shared_ptr<std::vector<cv::Point2f>>> line_points_;

  std::vector<Group::Ptr> groups_;

  std::vector<Pose> path_in_curr_pose_;
  std::vector<Eigen::Vector3f> distpoits_;
  bool ego_line_exist_ = false;              // todo need?
  EgoLane ego_line_id_;                      // todo need?
  std::vector<double> predict_line_params_;  // todo need? 三次样条
};

using RoadConstructPtr = std::unique_ptr<RoadConstruct>;

}  // namespace mf
}  // namespace mp
}  // namespace hozon
