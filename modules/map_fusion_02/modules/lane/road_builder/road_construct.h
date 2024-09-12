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

  std::deque<Line::Ptr> GetFitOcc();

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

  void ExtendLineToAlignSlice(const Group::Ptr& grp);

  void SplitLinesToGroup(std::deque<Line::Ptr>* lines, const Group::Ptr& grp);

  void SplitOccsToGroup(const std::map<Id, OccRoad::Ptr>& occ_roads,
                        const Group::Ptr& grp);

  void SplitModelEdgesToGroup(const std::map<Id, RoadEdge::Ptr>& road_edges,
                              const Group::Ptr& grp);

  void GenRoadEdges(std::vector<Group::Ptr>* groups);

  void RemoveInvalGroups(std::vector<Group::Ptr>* groups);

  void CheckRoadInval(std::vector<Group::Ptr>* groups);

  void CheckMidGroupLaneInval(std::vector<Group::Ptr>* groups);

  void UpdateRoadEdgeWithLines(const Group::Ptr& grp,
                               RoadEdge::Ptr* road_edge_left,
                               RoadEdge::Ptr* road_edge_right);

  void UpdateRoadEdgeWithModelEdges(const Group::Ptr& grp,
                                    RoadEdge::Ptr* road_edge_left,
                                    RoadEdge::Ptr* road_edge_right);

  void UpdateRoadEdgeWithOccs(const Group::Ptr& grp,
                              RoadEdge::Ptr* road_edge_left,
                              RoadEdge::Ptr* road_edge_right);

  void FindNearestCaditate(const std::vector<EdgeSegment::Ptr>& edge_segments,
                           RoadEdge::Ptr* candidate_road_edge_left,
                           RoadEdge::Ptr* candidate_road_edge_right);

  void UpdateWithCandidate(RoadEdge::Ptr* road_edge_left,
                           RoadEdge::Ptr* road_edge_right,
                           const RoadEdge::Ptr& candidate_road_edge_left,
                           const RoadEdge::Ptr& candidate_road_edge_right);

  void GenLanesInGroups(std::vector<Group::Ptr>* groups,
                        const ElementMap::Ptr& ele_map, double stamp);

  void GenGroupAllLanes(const Group::Ptr& grp);

  void FilterGroupBadLane(const Group::Ptr& grp);

  void GenGroupName(const ElementMap::Ptr& ele_map,
                    std::vector<Group::Ptr>* groups);

  void EgoLineTrajectory(const Group::Ptr& grp, const ElementMap::Ptr& ele_map);

  void SetBrokenId(std::vector<Group::Ptr>* groups);

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

  float Dist2Path(const Eigen::Vector3f& point);

  void BuildOccGroups(const ElementMap::Ptr& ele_map,
                      std::vector<Group::Ptr>* groups);

  void FitUnusedOccRoads(const ElementMap::Ptr& ele_map);

  void CheckBestOccRoad(double* good_k, double* good_b, double* good_y,
                        double* max_r_squared, int* good_id);

  void FitLineTLS(const std::vector<Eigen::Vector3d>& points, double* k,
                  double* b, double* r_squared);

  void GenNewOccRoads(const double& good_k, const double& good_b,
                      const double& good_y, const int& good_id);

  void SearchOccCutPoints();

 private:
  LaneFusionProcessOption conf_;
  std::map<int, std::shared_ptr<cv::flann::Index>> KDTrees_;
  std::map<int, std::shared_ptr<std::vector<cv::Point2f>>> line_points_;

  std::vector<Group::Ptr> groups_;

  std::vector<Pose> path_in_curr_pose_;
  std::vector<Eigen::Vector3f> distpoits_;
  bool ego_line_exist_ = false;              // todo need?
  std::vector<double> predict_line_params_;  // todo need? 三次样条

  std::map<Id, OccRoad::Ptr> unused_occ_roads_;
  std::deque<Line::Ptr> unused_occ_road_fitlines_;
  std::vector<CutPoint> unused_occ_ctps_;
};

using RoadConstructPtr = std::unique_ptr<RoadConstruct>;

}  // namespace mf
}  // namespace mp
}  // namespace hozon
