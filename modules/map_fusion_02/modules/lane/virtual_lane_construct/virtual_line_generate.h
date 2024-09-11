/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： virtual_line_generate.h
 *   author     ： zhangzhike
 *   date       ： 2024.08
 ******************************************************************************/

#pragma once
#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "common/math/vec2d.h"
#include "modules/map_fusion_02/base/element_base.h"
#include "modules/map_fusion_02/base/element_map.h"
#include "modules/map_fusion_02/base/group.h"
#include "modules/map_fusion_02/base/processor.h"
#include "modules/map_fusion_02/common/calc_util.h"
#include "modules/map_fusion_02/common/common_data.h"
#include "modules/util/include/util/mapping_log.h"

namespace hozon {
namespace mp {
namespace mf {

struct GroupVirtualLine {
  std::vector<LineSegment::Ptr> lines;
  std::vector<int> lines_type;  // 1 line, 2 road_edge, 3 occ
};

class VirtualLineGen : ProcessorBase {
 public:
  VirtualLineGen() = default;
  ~VirtualLineGen() = default;
  bool Init(const LaneFusionProcessOption& conf);
  bool ConstructVirtualLine(std::vector<Group::Ptr>* groups);
  void Clear() override;

 private:
  LaneFusionProcessOption conf_;
  void ExtendLineToAlignSlice(std::vector<Group::Ptr>* groups);
  void SameLineNumVirtualBuild(const Group::Ptr& curr_group,
                               std::vector<LineSegment::Ptr>* line_virtual_vec,
                               const std::vector<float>& line_dis,
                               const LineSegment::Ptr& left_line,
                               const LineSegment::Ptr& right_line,
                               float lane_width, int left_type, int right_type);
  void MergeSceneVirtualBuild(const Group::Ptr& curr_group,
                              std::vector<LineSegment::Ptr>* line_virtual_vec,
                              const std::vector<float>& line_dis,
                              const LineSegment::Ptr& left_line,
                              const LineSegment::Ptr& right_line,
                              float lane_width, int left_type, int right_type);
  void SplitSceneVirtualBuild(const Group::Ptr& curr_group,
                              std::vector<LineSegment::Ptr>* line_virtual_vec,
                              const std::vector<float>& line_dis,
                              const LineSegment::Ptr& left_line,
                              const LineSegment::Ptr& right_line,
                              float lane_width, int left_type, int right_type);
  void SlicePointVirtualLine(const Group::Ptr& curr_group,
                             std::vector<LineSegment::Ptr>* line_virtual_vec,
                             const std::vector<float>& start_between_lr,
                             const std::vector<float>& line_dis,
                             const LineSegment::Ptr& left_line,
                             const LineSegment::Ptr& right_line,
                             float lane_width, int left_type, int right_type);
  void NeighborLineStSlP(const Eigen::Vector3f& start_po,
                         const Eigen::Vector3f& start_pr,
                         const Eigen::Vector3f& left_line_front_pt,
                         const Eigen::Vector3f& right_line_front_pt,
                         const std::vector<float>& start_grp_lines2slice,
                         std::vector<float>* start_between_lr);
  float GetTwoBoundayDis(const LineSegment::Ptr& left_boundary,
                         const LineSegment::Ptr& right_boundary);
  float GetTwoBoundayDis(const RoadEdge::Ptr& left_boundary,
                         const RoadEdge::Ptr& right_boundary);
  std::vector<float> GetRoadedgeLineFrontEndDis(
      const RoadEdge::Ptr& road_edge, const LineSegment::Ptr& boundary);
  std::vector<float> GetTwoBoundayFrontBackDis(
      const LineSegment::Ptr& left_boundary,
      const LineSegment::Ptr& right_boundary);
  std::vector<float> GetTwoBoundayFrontBackDis(
      const RoadEdge::Ptr& left_road_edge,
      const RoadEdge::Ptr& right_road_edge);
  float perpendicular_distance(const Eigen::Vector3f& A,
                               const Eigen::Vector3f& B,
                               const Eigen::Vector3f& P);
  float GetEntranceLaneWidth(
      const std::vector<LineSegment::Ptr>& bev_lanelines);
  float GetCurrGroupLaneWidth(const Group::Ptr& curr_grp);
  // 重载不同数据类型的同车道数构建
  void SameLineNumVirtualBuild(const Group::Ptr& curr_group,
                               std::vector<LineSegment::Ptr>* line_virtual_vec,
                               const std::vector<float>& line_dis,
                               const LineSegment::Ptr& left_line,
                               const LineSegment::Ptr& right_line,
                               float lane_width);
  void SameLineNumVirtualBuild(const Group::Ptr& curr_group,
                               std::vector<LineSegment::Ptr>* line_virtual_vec,
                               const std::vector<float>& line_dis,
                               const RoadEdge::Ptr& road_edge,
                               const LineSegment::Ptr& line, float lane_width);
  void SameLineNumVirtualBuild(const Group::Ptr& curr_group,
                               std::vector<LineSegment::Ptr>* line_virtual_vec,
                               const std::vector<float>& line_dis,
                               const RoadEdge::Ptr& road_left,
                               const RoadEdge::Ptr& road_right,
                               float lane_width);
  // 重载不同数据类型的合流group车道线构建
  void MergeSceneVirtualBuild(const Group::Ptr& curr_group,
                              std::vector<LineSegment::Ptr>* line_virtual_vec,
                              const std::vector<float>& line_dis,
                              const LineSegment::Ptr& left_line,
                              const LineSegment::Ptr& right_line,
                              float lane_width);
  void MergeSceneVirtualBuild(const Group::Ptr& curr_group,
                              std::vector<LineSegment::Ptr>* line_virtual_vec,
                              const std::vector<float>& line_dis,
                              const RoadEdge::Ptr& road_edge,
                              const LineSegment::Ptr& line, float lane_width);
  void MergeSceneVirtualBuild(const Group::Ptr& curr_group,
                              std::vector<LineSegment::Ptr>* line_virtual_vec,
                              const std::vector<float>& line_dis,
                              const RoadEdge::Ptr& road_left,
                              const RoadEdge::Ptr& road_right,
                              float lane_width);
  // 重载不同数据类型的分流group车道线构建
  void SplitSceneVirtualBuild(const Group::Ptr& curr_group,
                              std::vector<LineSegment::Ptr>* line_virtual_vec,
                              const std::vector<float>& line_dis,
                              const LineSegment::Ptr& left_line,
                              const LineSegment::Ptr& right_line,
                              float lane_width);
  void SplitSceneVirtualBuild(const Group::Ptr& curr_group,
                              std::vector<LineSegment::Ptr>* line_virtual_vec,
                              const std::vector<float>& line_dis,
                              const RoadEdge::Ptr& road_edge,
                              const LineSegment::Ptr& line, float lane_width);
  void SplitSceneVirtualBuild(const Group::Ptr& curr_group,
                              std::vector<LineSegment::Ptr>* line_virtual_vec,
                              const std::vector<float>& line_dis,
                              const RoadEdge::Ptr& road_left,
                              const RoadEdge::Ptr& road_right,
                              float lane_width);
  std::vector<float> StartSlicePrevLineIntersecPointProjec(
      const Group::Ptr& prev_group,
      const Group::Ptr& curr_group);  // line在slice上交点在slice上的投影
  std::vector<float> EndSliceNextLineIntersecPointProjec(
      const Group::Ptr& curr_group,
      const Group::Ptr& next_group);  // line在slice上交点在slice上的投影
  LineSegment::Ptr RoadEdgecvtLine(const RoadEdge::Ptr& road_edge);
  void SlicePointVirtualLine(const Group::Ptr& curr_group,
                             std::vector<LineSegment::Ptr>* line_virtual_vec,
                             const std::vector<float>& start_between_lr,
                             const std::vector<float>& end_between_lr,
                             const std::vector<float>& line_dis,
                             const RoadEdge::Ptr& road_left,
                             const RoadEdge::Ptr& road_right, float lane_width);
  void SlicePointVirtualLine(const Group::Ptr& curr_group,
                             std::vector<LineSegment::Ptr>* line_virtual_vec,
                             const std::vector<float>& start_between_lr,
                             const std::vector<float>& end_between_lr,
                             const std::vector<float>& line_dis,
                             const RoadEdge::Ptr& road,
                             const LineSegment::Ptr& line, float lane_width,
                             int is_line_left);
  void SlicePointVirtualLine(const Group::Ptr& curr_group,
                             std::vector<LineSegment::Ptr>* line_virtual_vec,
                             const std::vector<float>& start_between_lr,
                             const std::vector<float>& end_between_lr,
                             const std::vector<float>& line_dis,
                             const LineSegment::Ptr& left_line,
                             const LineSegment::Ptr& right_line,
                             float lane_width);
  float Pt2BaselineDis(const LineSegment::Ptr& base_line,
                       const Eigen::Vector3f& vec_popr,
                       const Eigen::Vector3f& po, float length);
  float Pt2BaselineDis(const RoadEdge::Ptr& base_line,
                       const Eigen::Vector3f& vec_popr,
                       const Eigen::Vector3f& po, float length);
  void NeighborLineStSlP(const Eigen::Vector3f& start_po,
                         const Eigen::Vector3f& start_pr,
                         const Eigen::Vector3f& end_po,
                         const Eigen::Vector3f& end_pr,
                         const Eigen::Vector3f& left_line_front_pt,
                         const Eigen::Vector3f& right_line_front_pt,
                         const Eigen::Vector3f& left_line_back_pt,
                         const Eigen::Vector3f& right_line_back_pt,
                         const std::vector<float>& start_grp_lines2slice,
                         const std::vector<float>& end_grp_lines2slice,
                         std::vector<float>* start_between_lr,
                         std::vector<float>* end_between_lr);
  void TranslateLine(LineSegment* line_virtual, Point tmp,
                     const LineSegment::Ptr& left_line,
                     const Group::Ptr& curr_group);
  void TranslateRoad(LineSegment* line_virtual, Point tmp,
                     const RoadEdge::Ptr& road_edge,
                     const Group::Ptr& curr_group);
  void TranslateLineBack(LineSegment* line_virtual, Point tmp,
                         const LineSegment::Ptr& left_line,
                         const Group::Ptr& curr_group);
  void TranslateRoadBack(LineSegment* line_virtual, Point tmp,
                         const RoadEdge::Ptr& road_edge,
                         const Group::Ptr& curr_group);
  void TranslateAndRotateRoad(LineSegment* line_virtual, Point start_pt,
                              Point end_pt, const RoadEdge::Ptr& road_edge,
                              const Group::Ptr& curr_group);
  void TranslateAndRotateLine(LineSegment* line_virtual, Point start_pt,
                              Point end_pt, const LineSegment::Ptr& line,
                              const Group::Ptr& curr_group);
  void GenGroupAllLanes(const Group::Ptr& grp);
  float DistPointNew(const Point& ref_point, const LineSegment& lineSegment);
  void ComputeCenterPoints(Lane::Ptr lane);

  int virtual_line_id_ = 3000;
  float averge_exit_lane_width_ = 3.5;

  std::vector<std::unordered_set<std::string>> groups_lane_name_;
  std::unordered_map<std::string, GroupVirtualLine> groups_lines_;
};

using VirtualLineGenPtr = std::unique_ptr<VirtualLineGen>;

}  // namespace mf
}  // namespace mp
}  // namespace hozon
