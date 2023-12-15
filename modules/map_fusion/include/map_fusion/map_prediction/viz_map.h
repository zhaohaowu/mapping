/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： viz_map.h
 *   author     ： zhangshuo
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <adsfi_proto/internal/node_info.pb.h>
#include <adsfi_proto/location/location.pb.h>
// #include <adsfi_proto/map/local_map.pb.h>
#include <adsfi_proto/perception/lanes.pb.h>
#include <adsfi_proto/viz/visualization_msgs.pb.h>

#include <cstdint>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "map/hdmap/hdmap.h"
#include "map/hdmap/hdmap_common.h"

// #include "Eigen/src/Core/Matrix.h"
// #include "map_fusion/map_service/map_table.h"
#include "util/geo.h"
#include "util/rviz_agent/rviz_agent.h"

namespace hozon {
namespace mp {
namespace mf {

using Vec2d = common::math::Vec2d;

class VizMap {
 public:
  VizMap() = default;
  ~VizMap() = default;

  int Init();

 public:
  void VizLocalMapLaneLine(const std::shared_ptr<hozon::hdmap::Map>& msg);
  void StoreLaneLine(
      const std::shared_ptr<hozon::hdmap::Map>& msg,
      std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>&
          localmap_lanelines);
  static void StoreLeftLine(const hozon::hdmap::Lane& lane,
                            std::vector<Eigen::Vector3d>* left_line_point);
  static void StoreRightLine(const hozon::hdmap::Lane& lane,
                             std::vector<Eigen::Vector3d>* right_line_point);
  //   void PointsToMarker(const double stamp,
  //                       const std::vector<Eigen::Vector3d>& points,
  //                       adsfi_proto::viz::Marker* marker, double color_type);
  void VizHqMapRoad(const std::vector<Eigen::Vector3d>& edge);
  static void LaneLineToMarker(
      const std::pair<uint32_t, std::vector<Eigen::Vector3d>>& lane_line,
      adsfi_proto::viz::Marker* marker);
  static void RoadEdgeToMarker(const std::vector<Eigen::Vector3d>& road_edge,
                               adsfi_proto::viz::Marker* marker);
  void VizAddSideLaneLine(
      const std::vector<std::vector<Eigen::Vector3d>>& addLaneLines);
  static void AddLanelineToMarker(const std::vector<Eigen::Vector3d>& lane_line,
                                  adsfi_proto::viz::Marker* marker);
  void VizAddAheadLaneLine(
      const std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>&
          aheadLaneLines);
  static void AheadLanelineToMarker(
      const std::pair<uint32_t, std::vector<Eigen::Vector3d>>& lane_line,
      adsfi_proto::viz::Marker* marker);
  void VizLaneID(const std::shared_ptr<hozon::hdmap::Map>& local_msg);
//   void VizHDLaneID(
//       const std::unordered_map<std::string, LaneInfo>& lanes_in_range);
  void VizCompanLane(
      const std::vector<std::vector<Eigen::Vector3d>>& compan_lines);
  static void ComLaneLineToMarker(const std::vector<Eigen::Vector3d>& lane_line,
                                  adsfi_proto::viz::Marker* marker);
  static void VizLocalMsg(const std::shared_ptr<hozon::hdmap::Map>& local_msg,
                          const Eigen::Vector3d& pose);
  void VizCenterLane(const std::vector<Vec2d>& cent_points);

 private:
  const std::string viz_localmap_line_ = "/mp/localmap_line";
  const std::string viz_hqmap_road_ = "/mp/hqmap_line";
  const std::string viz_add_line_ = "/mp/side_line";
  const std::string viz_ahead_line_ = "/mp/ahead_line";
  const std::string viz_lane_id_ = "/mp/lane_id";
  const std::string viz_hd_lane_id_ = "/mp/hd_lane_id";
  const std::string viz_com_lane_id_ = "/mp/com_lane_line_";
  const std::string viz_center_lane_ = "/mp/center_lane";

  const std::string kFrameIdLocalEnu = "local_enu";
  bool viz_flag_ = true;
  bool viz_ = true;
  int id_ = 0;
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
