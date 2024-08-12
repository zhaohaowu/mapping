/***************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： geo_optimization_rviz.cc
 *   author     ： ouyanghailin
 *   date       ： 2024.08
 ******************************************************************************/

#include "modules/map_fusion_02/rviz/geo_optimization_rviz.h"

namespace hozon {
namespace mp {
namespace mf {

void GeoOptimizationViz::PointsToMarker(
    const double stamp, const std::vector<Eigen::Vector3d>& points,
    const std::vector<float>& color_type, adsfi_proto::viz::Marker* marker) {
  if (marker == nullptr) {
    return;
  }
  static int id = 0;
  marker->Clear();
  marker->mutable_header()->set_frameid("map");
  marker->mutable_header()->mutable_timestamp()->set_sec(
      static_cast<uint32_t>(stamp));
  marker->mutable_header()->mutable_timestamp()->set_nsec(
      static_cast<uint32_t>(stamp));
  marker->set_ns("ns_local_map_lane");
  marker->set_id(id++);
  marker->set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
  marker->set_action(adsfi_proto::viz::MarkerAction::ADD);
  marker->mutable_pose()->mutable_position()->set_x(0);
  marker->mutable_pose()->mutable_position()->set_y(0);
  marker->mutable_pose()->mutable_position()->set_z(0);
  marker->mutable_pose()->mutable_orientation()->set_x(0.);
  marker->mutable_pose()->mutable_orientation()->set_y(0.);
  marker->mutable_pose()->mutable_orientation()->set_z(0.);
  marker->mutable_pose()->mutable_orientation()->set_w(1.);
  marker->mutable_scale()->set_x(0.2);
  marker->mutable_scale()->set_y(0.2);
  marker->mutable_scale()->set_z(0.2);
  marker->mutable_lifetime()->set_sec(0);
  marker->mutable_lifetime()->set_nsec(150000000);
  adsfi_proto::viz::ColorRGBA color;
  color.set_a(1.0);
  color.set_r(color_type[0]);
  color.set_g(color_type[1]);
  color.set_b(color_type[2]);
  marker->mutable_color()->CopyFrom(color);
  for (const auto& point : points) {
    auto* pt = marker->add_points();
    pt->set_x(point.x());
    pt->set_y(point.y());
    pt->set_z(point.z());
  }
  if (marker->points().empty()) {
    HLOG_WARN << "empty lane line";
  }
}

void GeoOptimizationViz::LineIdToMarker(const double stamp,
                                        const Eigen::Vector3d& point,
                                        const std::string& id,
                                        adsfi_proto::viz::Marker* marker) {
  if (marker == nullptr) {
    return;
  }
  static int id_set = 0;
  marker->mutable_header()->set_frameid("map");
  marker->mutable_header()->mutable_timestamp()->set_sec(
      static_cast<uint32_t>(stamp));
  marker->mutable_header()->mutable_timestamp()->set_nsec(
      static_cast<uint32_t>(stamp));
  marker->set_id(id_set++);
  marker->set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
  marker->set_action(adsfi_proto::viz::MarkerAction::ADD);
  marker->mutable_pose()->mutable_orientation()->set_x(0.);
  marker->mutable_pose()->mutable_orientation()->set_y(0.);
  marker->mutable_pose()->mutable_orientation()->set_z(0.);
  marker->mutable_pose()->mutable_orientation()->set_w(1.);
  const double text_size = 3;
  marker->mutable_scale()->set_z(text_size);
  marker->mutable_lifetime()->set_sec(0);
  marker->mutable_lifetime()->set_nsec(200000000);
  marker->mutable_color()->set_a(1.0);
  marker->mutable_color()->set_r(1);
  marker->mutable_color()->set_g(0.1);
  marker->mutable_color()->set_b(0);
  marker->mutable_pose()->mutable_position()->set_x(point.x());
  marker->mutable_pose()->mutable_position()->set_y(point.y());
  marker->mutable_pose()->mutable_position()->set_z(point.z());
  auto* text = marker->mutable_text();
  *text = "ID: " + id;
}

void GeoOptimizationViz::VizElementMap(ElementMap::Ptr element_map,
                                      const Eigen::Affine3d& T) {
  HLOG_FATAL << "map_fusion02 geo viz elementmap";
  adsfi_proto::viz::MarkerArray markers;
  std::vector<Eigen::Vector3d> lane_points;
  auto& element_map_info = element_map->map_info;
  auto& lane_boundaries = element_map->lane_boundaries;
  for (auto& lane_boundary : lane_boundaries) {
    auto& lane_boundary_nodes = lane_boundary.second->nodes;
    for (auto& point : lane_boundary_nodes) {
      Eigen::Vector3d point_local(point->point.x(), point->point.y(), point->point.z());
      Eigen::Vector3d point_enu = T * point_local;
      lane_points.emplace_back(point_enu);
    }
    adsfi_proto::viz::Marker marker;
    std::vector<float> color = color_palette.find('r')->second;
    PointsToMarker(element_map_info.stamp, lane_points, color, &marker);
    if (marker.points().size() >= 2) {
      markers.add_markers()->CopyFrom(marker);
    }
    if (!lane_points.empty()) {
      adsfi_proto::viz::Marker marker_id;
      auto id = std::to_string(lane_boundary.first);
      LineIdToMarker(element_map_info.stamp, lane_points[0],
                     id, &marker_id);
      markers.add_markers()->CopyFrom(marker_id);
    }
  }
  RVIZ_AGENT.Publish(KTopicRoadRecognitionElementMap, markers);
}

// void GeoOptimizationViz::VizLocalMap(
//     bool road_recognition_rviz_switch,
//     const std::shared_ptr<hozon::mapping::LocalMap>& local_map,
//     const std::string& viz_topic, const Eigen::Isometry3d& T_U_V) {
//   if ((!road_recognition_rviz_switch) || (!RVIZ_AGENT.Ok())) {
//     return;
//   }
//   int ret = RVIZ_AGENT.Register<adsfi_proto::viz::TransformStamped>(
//       kTopicRoadRecognitionTf);
//   ret = RVIZ_AGENT.Register<adsfi_proto::viz::Path>(
//       kTopicRoadRecognitionLocation);
//   std::vector<std::string> topic_vec{
//       KTopicRoadRecognitionLocalMap, KTopicRoadRecognitionTopoMapRoad,
//       KTopicRoadRecognitionTopoMapLane, KTopicRoadRecognitionElementMap,
//       KTopicRoadRecognitionLineLable};
//   ret = RVIZ_AGENT.Register<adsfi_proto::viz::MarkerArray>(topic_vec);
//   adsfi_proto::viz::MarkerArray markers;
//   for (const auto& i : local_map->lane_lines()) {
//     std::vector<Eigen::Vector3d> lane_points;
//     for (const auto& point : i.points()) {
//       Eigen::Vector3d point_local(point.x(), point.y(), point.z());
//       Eigen::Vector3d point_enu = T_U_V * point_local;
//       lane_points.emplace_back(point_enu);
//     }
//     adsfi_proto::viz::Marker marker;
//     std::vector<float> color = color_palette.find('c')->second;
//     PointsToMarker(local_map->header().publish_stamp(), lane_points, color,
//                    &marker);
//     if (marker.points().size() >= 2) {
//       markers.add_markers()->CopyFrom(marker);
//     }
//     if (!lane_points.empty()) {
//       adsfi_proto::viz::Marker marker_id;
//       auto id = std::to_string(i.lanepos());
//       LineIdToMarker(local_map->header().publish_stamp(), lane_points[0], id,
//                      &marker_id);
//       markers.add_markers()->CopyFrom(marker_id);
//     }
//   }

//   for (const auto& i : local_map->road_edges()) {
//     std::vector<Eigen::Vector3d> lane_points;
//     for (const auto& point : i.points()) {
//       Eigen::Vector3d point_local(point.x(), point.y(), point.z());
//       Eigen::Vector3d point_enu = T_U_V * point_local;
//       lane_points.emplace_back(point_enu);
//     }
//     adsfi_proto::viz::Marker marker;
//     std::vector<float> color = color_palette.find('b')->second;
//     PointsToMarker(local_map->header().publish_stamp(), lane_points, color,
//                    &marker);
//     if (marker.points().size() >= 2) {
//       markers.add_markers()->CopyFrom(marker);
//     }
//     if (!lane_points.empty()) {
//       adsfi_proto::viz::Marker marker_id;
//       auto id = std::to_string(i.lanepos());
//       LineIdToMarker(local_map->header().publish_stamp(), lane_points[0], id,
//                      &marker_id);
//       markers.add_markers()->CopyFrom(marker_id);
//     }
//   }

//   for (const auto& i : local_map->stop_lines()) {
//     std::vector<Eigen::Vector3d> lane_points;
//     Eigen::Vector3d point_left(i.left_point().x(), i.left_point().y(), 0);
//     Eigen::Vector3d point_right(i.right_point().x(), i.right_point().y(), 0);
//     lane_points.emplace_back(T_U_V * point_left);
//     lane_points.emplace_back(T_U_V * point_right);
//     adsfi_proto::viz::Marker marker;
//     std::vector<float> color = color_palette.find('o')->second;
//     PointsToMarker(local_map->header().publish_stamp(), lane_points, color,
//                    &marker);
//     markers.add_markers()->CopyFrom(marker);
//   }

//   for (const auto& i : local_map->arrows()) {
//     if (i.points().point_size() < 1) {
//       continue;
//     }
//     std::vector<Eigen::Vector3d> arrow_points;
//     for (const auto& point_arrow : i.points().point()) {
//       Eigen::Vector3d arrow_pt(point_arrow.x(), point_arrow.y(),
//                                point_arrow.z());
//       arrow_points.emplace_back(T_U_V * arrow_pt);
//     }
//     arrow_points.emplace_back(arrow_points[0]);
//     adsfi_proto::viz::Marker marker;
//     std::vector<float> color = color_palette.find('w')->second;
//     PointsToMarker(local_map->header().publish_stamp(), arrow_points, color,
//                    &marker);
//     markers.add_markers()->CopyFrom(marker);
//   }

//   for (const auto& i : local_map->cross_walks()) {
//     if (i.points().point_size() < 1) {
//       continue;
//     }
//     std::vector<Eigen::Vector3d> zebra_points;
//     for (const auto& zebra_pt : i.points().point()) {
//       Eigen::Vector3d pt(zebra_pt.x(), zebra_pt.y(), zebra_pt.z());
//       zebra_points.emplace_back(T_U_V * pt);
//     }
//     zebra_points.emplace_back(zebra_points[0]);
//     adsfi_proto::viz::Marker marker;
//     std::vector<float> color = color_palette.find('c')->second;
//     PointsToMarker(local_map->header().publish_stamp(), zebra_points, color,
//                    &marker);
//     markers.add_markers()->CopyFrom(marker);
//   }

//   RVIZ_AGENT.Publish(viz_topic, markers);
// }

}  // namespace mf
}  // namespace mp
}  // namespace hozon
