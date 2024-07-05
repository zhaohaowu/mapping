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

#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <list>
#include <utility>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <map>

#include "depend/proto/localization/node_info.pb.h"
#include "depend/proto/map/map.pb.h"
#include "modules/location/pose_estimation/lib/hd_map/hd_map.h"
#include "modules/location/pose_estimation/lib/pose_estimate/pose_estimate.h"
#include "modules/location/pose_estimation/lib/fault/fault.h"

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
  void ProcData(bool use_rviz, const SE3& T_input,
                const std::shared_ptr<hozon::localization::Localization>& fc,
                const hozon::perception::TransportElement& perception,
                const std::vector<hozon::hdmap::LaneInfoConstPtr>& lanes,
                const Eigen::Vector3d& ref_point, double ins_height,
                int sys_status);
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
  VP SetRvizMergeMapLines(
      std::unordered_map<std::string, std::vector<ControlPoint>>
          merged_map_lines,
      const SE3& T02_W_V);
  PtrNodeInfo getMmNodeInfo();
  PtrNodeInfo generateNodeInfo(const Sophus::SE3d& T_W_V, uint64_t sec,
                               uint64_t nsec, const bool& has_err,
                               const Eigen::Vector3d& ref_point,
                               double ins_height, int sys_status,
                               bool is_big_curvature_frame,
                               double lane_width_check_coeff);
  void RvizFunc(uint64_t cur_sec, uint64_t cur_nsec,
                const hozon::mp::loc::Connect& connect, const SE3& T_output,
                const VP& rviz_merged_map_lines);
  void setPoints(const PerceptionLaneLineList& line_list, const SE3& T_W_V,
                 VP* points);
  void pubPoints(const VP& points, const uint64_t& sec, const uint64_t& nsec,
                 const std::string& krviz_topic);
  void pubOdomPoints(const std::string& topic, const SE3& T, uint64_t sec,
                     uint64_t nsec);
  void PubTfAndPath(const SE3& T, uint64_t sec, uint64_t nsec);
  void PubLocState(const SE3& T, uint64_t sec, uint64_t nsec, int loc_state);
  void pubTimeAndInsStatus(const SE3& T, uint64_t sec, uint64_t nsec,
                           int ins_state);
  void setConnectPercepPoints(const Connect& connect, const SE3& T_W_V,
                              VP* points);
  void pubConnectPercepPoints(const VP& points, uint64_t sec, uint64_t nsec);
  void setOriginConnectPercepPoints(const Connect& connect, const SE3& T_W_V,
                                    VP* points);
  void pubOriginConnectPercepPoints(const VP& points, uint64_t sec,
                                    uint64_t nsec);
  void setOriginConnectMapPoints(const Connect& connect, VP* points);
  void pubOriginConnectMapPoints(const VP& points, uint64_t sec, uint64_t nsec);
  void setConnectMapPoints(const Connect& connect, VP* points);
  void pubConnectMapPoints(const VP& points, uint64_t sec, uint64_t nsec);

  adsfi_proto::viz::Marker RoadEdgeToMarker(
      const VP& points, const std::string& id, bool is_points, bool is_center,
      uint64_t sec, uint64_t nsec, bool is_edge = false, float point_size = 1);
  adsfi_proto::viz::Marker laneToMarker(const VP& points, const std::string& id,
                                        bool is_points, bool is_center,
                                        uint64_t sec, uint64_t nsec,
                                        bool is_boundary = false,
                                        float point_size = 1);
  adsfi_proto::viz::Marker lineIdToMarker(const V3& point,
                                          const std::string& id, uint64_t sec,
                                          uint64_t nsec);

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

  const std::string kTopicMmTf = "/mm/tf";
  const std::string kTopicMmInterTf = "/mm/tf_inter";
  const std::string kTopicMmCarPath = "/mm/car_path";
  const std::string kTopicMmInterCarPath = "/mm/car_path_inter";
  const std::string kTopicMmFrontPoints = "/mm/front_point";
  const std::string kTopicMmMergedMapLaneLinePoints =
      "/mm/merged_map_lane_line_points";
  const std::string kTopicInsOdom = "/mm/ins_odom";
  const std::string kTopicMmOdom = "/mm/mm_odom";
  const std::string kTopicDrOdom = "/mm/dr_odom";
  const std::string kTopicFcOdom = "/mm/fc_odom";
  const std::string kTopicInputOdom = "/mm/input_odom";
  const std::string kTopicMmMatchPoints = "/mm/link";
  const std::string KTopicMmHdMap = "/mm/hd_map";
  const std::string kTopicMmTimeStamp = "/mm/time_stamp";
  const std::string kTopicMmPerceptionPointsEdge = "/mm/perception_point_edge";
  const std::string kTopicMmMapPointsEdge = "/mm/map_point_edge";
  const std::string kTopicLocstate = "/fc/locstate";
  const std::string kTopicMmConnectPercepPoints = "/mm/connect_per_point";
  const std::string kTopicMmConnectMapPoints = "/mm/connect_map_point";
  const std::string kTopicMmOriginConnectPercepPoints =
      "/mm/origin_connect_per_point";
  const std::string kTopicMmOriginConnectMapPoints =
      "/mm/origin_connect_map_point";
};

}  // namespace loc
}  // namespace mp
}  // namespace hozon
