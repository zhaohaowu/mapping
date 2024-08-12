/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： lane_construct.h
 *   author     ： cuijiayu
 *   date       ： 2024.08
 ******************************************************************************/
#pragma once

#include <depend/common/math/vec2d.h>

#include <deque>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "modules/util/include/util/mapping_log.h"
#include "modules/map_fusion_02/common/calc_util.h"
#include "modules/map_fusion_02/common/common_data.h"
#include "modules/map_fusion_02/modules/lane/road_builder/broken_point_search.h"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d.hpp"

using hozon::common::math::Vec2d;

namespace hozon {
namespace mp {
namespace mf {

struct RoadConstructConf {
  // 用于分割GroupSegment的分割线的1/2长度
  float half_slice_length = 0;
  // 从线聚合到lane的宽度阈值
  float min_lane_width = 0;
  float max_lane_width = 0;
};

class RoadConstruct {
 public:
  explicit RoadConstruct(const RoadConstructConf& conf) : conf_(conf) {}

  ~RoadConstruct() = default;

  bool ConstructLane(const std::vector<CutPoint>& cutpoints,
                     std::deque<Line::Ptr> lines,
                     const std::shared_ptr<std::vector<KinePosePtr>>& path,
                     const KinePosePtr& curr_pose,
                     const KinePosePtr& last_pose,
                     const ElementMap::Ptr& ele_map);

  void GetDistPoints(std::vector<Eigen::Vector3f>* distpoints);

  void GetGroups(std::vector<Group::Ptr>* groups);

 private:
  void BuildKDtrees(std::deque<Line::Ptr>* lines);

  void UpdatePathInCurrPose(const std::vector<KinePosePtr>& path,
                            const KinePose& curr_pose);

  void BuildGroupSegments(const std::vector<CutPoint>& cutpoints,
                          std::deque<Line::Ptr>* lines,
                          std::vector<GroupSegment::Ptr>* group_segments,
                          const ElementMap::Ptr& ele_map);

  void CreateGroupSegFromCutPoints(
      const std::vector<CutPoint>& cutpoints,
      std::vector<GroupSegment::Ptr>* segments);

  void SplitPtsToGroupSeg(std::deque<Line::Ptr>* lines,
                          std::vector<GroupSegment::Ptr>* segments);

  void GenLaneSegInGroupSeg(std::vector<GroupSegment::Ptr>* segments);

  float DistByKDtree(const Eigen::Vector3f& ref_point,
                     const LineSegment& line);

  float GetDistPointLane(const Eigen::Vector3f& point_a,
                         const Eigen::Vector3f& point_b,
                         const Eigen::Vector3f& point_c);

  void EgoLineTrajectory(std::vector<GroupSegment::Ptr>* grp_segment,
                         const ElementMap::Ptr& ele_map);

  void FitLaneline(const ElementMap::Ptr& ele_map, int id_1, int id_2,
                   int near_line);

  std::vector<double> FitLaneline(const std::vector<Point>& centerline);

  void BuildGroups(const ElementMap::Ptr& ele_map,
                   const std::vector<GroupSegment::Ptr>& group_segments,
                   std::vector<Group::Ptr>* groups);

  void UniteGroupSegmentsToGroups(
      double stamp, const std::vector<GroupSegment::Ptr>& group_segments,
      std::vector<Group::Ptr>* groups);

  void GenLanesInGroups(std::vector<Group::Ptr>* groups,
                        std::map<Id, OccRoad::Ptr> occ_roads,
                        double stamp);

  void CollectGroupPossibleLanes(const Group::Ptr& grp,
                                 std::vector<Lane::Ptr>* possible_lanes);

  bool FilterGroupBadLane(const std::vector<Lane::Ptr>& possible_lanes,
                          const Group::Ptr& grp);

  bool MatchLRLane(const Group::Ptr& grp);

  bool OptiPreNextLaneBoundaryPoint(std::vector<Group::Ptr>* groups);

  bool GenLaneCenterLine(std::vector<Group::Ptr>* groups);

  void FitCenterLine(const Lane::Ptr& lane);

  std::vector<double> FitLanelinefront(
      const std::vector<Point>& centerline);

  void RemoveNullGroup(std::vector<Group::Ptr>* groups);

  void SmoothCenterline(std::vector<Group::Ptr>* groups);

  std::vector<Point> SigmoidFunc(const std::vector<Point>& centerline,
                                      float sigma);

  RoadConstructConf conf_;
  std::map<int, std::shared_ptr<cv::flann::Index>> KDTrees_;
  std::map<int, std::shared_ptr<std::vector<cv::Point2f>>> line_points_;

  std::vector<GroupSegment::Ptr> group_segments_;
  std::vector<Group::Ptr> groups_;

  std::vector<Pose> path_in_curr_pose_;
  std::vector<Eigen::Vector3f> distpoits_;
  bool ego_line_exist_ = false;              // todo need?
  EgoLane ego_line_id_;                 // todo need?
  std::vector<double> predict_line_params_;  // todo need? 三次样条
};

using RoadConstructPtr = std::unique_ptr<RoadConstruct>;

}  // namespace mf
}  // namespace mp
}  // namespace hozon
