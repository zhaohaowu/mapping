/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： pose_estimation.h
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <adsfi_proto/viz/geometry_msgs.pb.h>
#include <adsfi_proto/viz/nav_msgs.pb.h>
#include <adsfi_proto/viz/sensor_msgs.pb.h>
#include <adsfi_proto/viz/visualization_msgs.pb.h>
#include <google/protobuf/util/json_util.h>
#include <yaml-cpp/yaml.h>

#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "depend/proto/localization/node_info.pb.h"
#include "depend/proto/map/map.pb.h"
#include "modules/location/pose_estimation/lib/fault/fault.h"
#include "modules/location/pose_estimation/lib/hd_map/hd_map.h"
#include "modules/location/pose_estimation/lib/pose_estimate/pose_estimate.h"
#include "proto/local_mapping/local_map.pb.h"

namespace hozon {
namespace mp {
namespace loc {

using hozon::localization::HafNodeInfo;
using PtrNodeInfo = std::shared_ptr<::hozon::localization::HafNodeInfo>;
using LineSegment = std::pair<std::string, V3>;  // LineSegment: {id, end_point}

// MM故障，后续可补充
struct MMFault {
  bool map_lane_match_error = false;  // 感知与地图车道线差距过大
  bool valid_estimate_last_error = false;  // mm正常优化一段时间后，优化失败了
  bool valid_estimate = false;           // mm是否正常优化
  bool pecep_lane_error = false;         // 无有效感知车道线
  bool map_lane_error = false;           // 无有效地图车道线
  bool fc_exceed_curb_error = false;     // fc超出高精地图路沿
  bool fc_offset_onelane_error = false;  // fc偏移一个车道
};
struct ControlPointInfo {
  V3 last_start_point_v{0.0, 0.0, 0.0};
  V3 last_end_point_v{0.0, 0.0, 0.0};
  size_t last_control_points_size = 0;
};

struct EigenMatrixHash {
  std::size_t operator()(const Eigen::Matrix<double, 3, 1>& matrix) const {
    return std::hash<double>()(matrix(0)) ^ std::hash<double>()(matrix(1)) ^
           std::hash<double>()(matrix(2));
  }
};
struct ControlPointInfoEqual {
  bool operator()(const ControlPointInfo& lhs,
                  const ControlPointInfo& rhs) const {
    return lhs.last_start_point_v.isApprox(rhs.last_start_point_v) &&
           lhs.last_end_point_v.isApprox(rhs.last_end_point_v) &&
           lhs.last_control_points_size == rhs.last_control_points_size;
  }
};
struct ControlPointInfoHash {
  std::size_t operator()(const ControlPointInfo& cpt_info) const {
    std::size_t hashValue = 0;
    hashValue ^= EigenMatrixHash()(cpt_info.last_start_point_v);
    hashValue ^= EigenMatrixHash()(cpt_info.last_end_point_v);
    hashValue ^= std::hash<size_t>()(cpt_info.last_control_points_size);
    return hashValue;
  }
};

class MapMatching {
 public:
  MapMatching();
  ~MapMatching() = default;
  bool Init(const std::string& config_file);
  hozon::mp::loc::Map<hozon::hdmap::Map> SetHdMap(
      const std::vector<hozon::hdmap::LaneInfoConstPtr>& lane_ptr_vec,
      const Eigen::Vector3d& ref_point);
  void ProcData(const SE3& T_input,
                const std::shared_ptr<hozon::localization::Localization>& fc,
                const hozon::mapping::LocalMap& perception,
                const std::vector<hozon::hdmap::LaneInfoConstPtr>& lanes,
                const Eigen::Vector3d& ref_point, double ins_height,
                int sys_status);
  bool CheckIsRampRoad(const hozon::common::PointENU& utm_pos);
  void MergeMapLanes(
      const HdMap& hd_map, const SE3& T_W_V, const ValidPose& T_fc,
      std::unordered_map<std::string, std::vector<ControlPoint>>* const
          merged_map_lines,
      std::unordered_map<std::string, std::vector<ControlPoint>>* const
          merged_fcmap_lines,
      std::unordered_map<std::string, std::vector<ControlPoint>>* const
          merged_map_edges);
  void FilterPercpLane(
      const std::shared_ptr<Perception>& perception,
      std::list<std::list<LaneLinePerceptionPtr>>* const percep_lanelines);
  void MergeMapLines(const std::shared_ptr<MapBoundaryLine>& boundary_lines,
                     const SE3& T,
                     std::unordered_map<std::string, std::vector<ControlPoint>>*
                         merged_fcmap_lines,
                     std::unordered_map<std::string, std::vector<ControlPoint>>*
                         merged_map_lines);
  void MergeMapEdges(const std::shared_ptr<MapRoadEdge>& road_edges,
                     const SE3& T,
                     std::unordered_map<std::string, std::vector<ControlPoint>>*
                         merged_map_edges);
  void Traversal(const V3& root_start_point, std::vector<std::string> line_ids,
                 int loop);
  void EdgesTraversal(const V3& root_start_point,
                      std::vector<std::string> line_ids, int loop);
  void FilterPercpLaneline(const std::list<LaneLinePerceptionPtr>& lanelines,
                           std::list<LaneLinePerceptionPtr>* const out);
  PtrNodeInfo getMmNodeInfo();
  PtrNodeInfo generateNodeInfo(const Sophus::SE3d& T_W_V, uint64_t sec,
                               uint64_t nsec, const bool& has_err,
                               const Eigen::Vector3d& ref_point,
                               double ins_height, int sys_status,
                               bool is_big_curvature_frame,
                               double lane_width_check_coeff);
  void RvizFunc(const std::unordered_map<
                    std::string, std::vector<ControlPoint>>& merged_map_lines,
                const Sophus::SE3d& T_input, const Sophus::SE3d& T_output);
  void SetWarnInfo(int warn_info) { warn_info_ = warn_info; }

 private:
  std::shared_ptr<hozon::mp::loc::MapMatch> map_match_;
  std::shared_ptr<hozon::mp::loc::MmFault> mm_fault_;
  std::unordered_set<std::string> edge_visited_id_;
  std::unordered_set<std::string> visited_id_;
  std::map<V3, std::vector<LineSegment>, PointV3Comp<V3>>
      edges_map_;  // map: {{start_point, LineSegment}}
  std::vector<std::vector<std::string>> linked_edges_id_;
  std::vector<std::vector<std::vector<std::string>>>
      multi_linked_edges_;  // 内层vector是一组前后继链接的车道线
  std::map<V3, std::vector<LineSegment>, PointV3Comp<V3>>
      lines_map_;  // map: {{start_point, LineSegment}}
  std::vector<std::vector<std::string>> linked_lines_id_;
  std::vector<std::vector<std::vector<std::string>>>
      multi_linked_lines_;  // 内层vector是一组前后继链接的车道线
  std::unordered_set<ControlPointInfo, ControlPointInfoHash,
                     ControlPointInfoEqual>
      lane_control_pointInfo_;
  std::unordered_set<ControlPointInfo, ControlPointInfoHash,
                     ControlPointInfoEqual>
      edge_control_pointInfo_;
  std::mutex mm_output_lck_;
  MMFault mmfault_;
  PtrNodeInfo mm_node_info_;
  int warn_info_ = 0;
};

}  // namespace loc
}  // namespace mp
}  // namespace hozon
