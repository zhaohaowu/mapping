/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： viz_map.cc
 *   author     ： zhangshuo
 *   date       ： 2023.09
 ******************************************************************************/

#include "map_fusion/map_prediction/viz_map.h"
#include <gflags/gflags.h>

#include <cmath>
#include <fstream>
#include <string>
#include <vector>

#include "Eigen/src/Core/Matrix.h"
#include "adsfi_proto/viz/sensor_msgs.pb.h"
#include "adsfi_proto/viz/visualization_msgs.pb.h"
#include "common/utm_projection/coordinate_convertor.h"
#include "map_fusion/map_service/map_proto_maker.h"
#include "util/mapping_log.h"

// NOLINTBEGIN
DEFINE_string(viz_addr_mp, "ipc:///tmp/rviz_agent_mp",
              "RvizAgent working address, may like "
              "ipc:///tmp/sample_rviz_agent or "
              "inproc://sample_rviz_agent or "
              "tcp://127.0.0.1:9100");
// NOLINTEND

namespace hozon {
namespace mp {
namespace mf {

double NowInSeconds() {
  timespec ts{};
  clock_gettime(CLOCK_REALTIME, &ts);
  double secs =
      static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
  return secs;
}

int VizMap::Init() {
  if (!viz_ || !RVIZ_AGENT.Ok()) {
    return 0;
  }

  int ret =
      RVIZ_AGENT.Register<adsfi_proto::viz::MarkerArray>(viz_localmap_line_);
  if (ret < 0) {
    HLOG_ERROR << "RvizAgent register" << viz_localmap_line_;
  }

  ret = RVIZ_AGENT.Register<adsfi_proto::viz::MarkerArray>(viz_hqmap_road_);
  if (ret < 0) {
    HLOG_ERROR << "RvizAgent register" << viz_hqmap_road_;
  }

  ret = RVIZ_AGENT.Register<adsfi_proto::viz::MarkerArray>(viz_add_line_);
  if (ret < 0) {
    HLOG_ERROR << "RvizAgent register" << viz_add_line_;
  }

  ret = RVIZ_AGENT.Register<adsfi_proto::viz::PointCloud>(viz_ahead_line_);
  if (ret < 0) {
    HLOG_ERROR << "RvizAgent register" << viz_ahead_line_;
  }

  ret = RVIZ_AGENT.Register<adsfi_proto::viz::MarkerArray>(viz_lane_id_);
  if (ret < 0) {
    HLOG_ERROR << "RvizAgent register" << viz_lane_id_;
  }

  ret = RVIZ_AGENT.Register<adsfi_proto::viz::MarkerArray>(viz_hd_lane_id_);
  if (ret < 0) {
    HLOG_ERROR << "RvizAgent register" << viz_hd_lane_id_;
  }

  ret = RVIZ_AGENT.Register<adsfi_proto::viz::MarkerArray>(viz_hd_lane_id_);
  if (ret < 0) {
    HLOG_ERROR << "RvizAgent register" << viz_hd_lane_id_;
  }

  ret = RVIZ_AGENT.Register<adsfi_proto::viz::MarkerArray>(viz_com_lane_id_);
  if (ret < 0) {
    HLOG_ERROR << "RvizAgent register" << viz_com_lane_id_;
  }

  ret = RVIZ_AGENT.Register<adsfi_proto::viz::MarkerArray>(viz_center_lane_);
  if (ret < 0) {
    HLOG_ERROR << "RvizAgent register" << viz_center_lane_;
  }

  // ret = RVIZ_AGENT.Register<adsfi_proto::viz::MarkerArray>("TuoPu");
  // if (ret < 0) {
  //   HLOG_WARN << "RvizAgent register "
  //             << "tuopu"
  //             << " failed";
  // }
  // ret = RVIZ_AGENT.Register<adsfi_proto::viz::MarkerArray>("TuoPu2");
  // if (ret < 0) {
  //   HLOG_WARN << "RvizAgent register "
  //             << "tuopu"
  //             << " failed";
  // }
  // ret = RVIZ_AGENT.Register<adsfi_proto::viz::MarkerArray>("TuoPu3");
  // if (ret < 0) {
  //   HLOG_WARN << "RvizAgent register "
  //             << "tuopu"
  //             << " failed";
  // }
  // ret = RVIZ_AGENT.Register<adsfi_proto::viz::MarkerArray>("TuoPu4");
  // if (ret < 0) {
  //   HLOG_WARN << "RvizAgent register "
  //             << "tuopu"
  //             << " failed";
  // }

  return 0;
}

void VizMap::VizLocalMapLaneLine(
    const std::shared_ptr<hozon::hdmap::Map>& msg) {
  // 开始进行可视化操作
  if (viz_flag_) {
    Init();
    HLOG_ERROR << "Register Successfull!";
    viz_flag_ = false;
  }

  if (!RVIZ_AGENT.Ok()) {
    return;
  }

  std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>
      localmap_lanelines;
  // 存储边线
  StoreLaneLine(msg, localmap_lanelines);

  adsfi_proto::viz::MarkerArray markers;
  for (const auto& lane_line : localmap_lanelines) {
    adsfi_proto::viz::Marker marker;
    LaneLineToMarker(lane_line, &marker);
    if (!marker.points().empty()) {
      markers.add_markers()->CopyFrom(marker);
    }
  }

  RVIZ_AGENT.Publish(viz_localmap_line_, markers);
}

void VizMap::StoreLaneLine(
    const std::shared_ptr<hozon::hdmap::Map>& msg,
    std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>&
        localmap_lanelines) {
  // 存储边线
  for (const auto& lane : msg->lane()) {
    // 存储左边线
    std::vector<Eigen::Vector3d> left_line_point;
    StoreLeftLine(lane, &left_line_point);
    if (!left_line_point.empty()) {
      localmap_lanelines.emplace_back(id_, left_line_point);
      id_ -= 1;
    }
    // 存储右边线
    // if (!lane.right_neighbor_forward_lane_id().empty()) {
    //   continue;
    // }
    std::vector<Eigen::Vector3d> right_line_point;
    StoreRightLine(lane, &right_line_point);
    if (!right_line_point.empty()) {
      localmap_lanelines.emplace_back(id_, right_line_point);
      id_ -= 1;
    }
  }
}

void VizMap::StoreLeftLine(const hozon::hdmap::Lane& lane,
                           std::vector<Eigen::Vector3d>* left_line_point) {
  for (const auto& left_line : lane.left_boundary().curve().segment()) {
    for (const auto& point : left_line.line_segment().point()) {
      Eigen::Vector3d point_enu(point.x(), point.y(), point.z());
      double x = point_enu.x();
      double y = point_enu.y();
      if (std::isnan(x) || std::isnan(y)) {
        HLOG_ERROR << "have nan!!!";
        continue;
      }
      left_line_point->emplace_back(point_enu);
    }
  }
}

void VizMap::StoreRightLine(const hozon::hdmap::Lane& lane,
                            std::vector<Eigen::Vector3d>* right_line_point) {
  for (const auto& right_line : lane.right_boundary().curve().segment()) {
    for (const auto& point : right_line.line_segment().point()) {
      Eigen::Vector3d point_enu(point.x(), point.y(), point.z());
      // int zone = 51;
      double x = point_enu.x();
      double y = point_enu.y();
      if (std::isnan(x) || std::isnan(y)) {
        HLOG_ERROR << "have nan!!!";
        continue;
      }
      right_line_point->emplace_back(point_enu);
    }
  }
}

#if 0
void VizMap::PointsToMarker(const double stamp,
                            const std::vector<Eigen::Vector3d>& points,
                            adsfi_proto::viz::Marker* marker,
                            double color_type) {
  static int id = 0;
  marker->Clear();
  marker->mutable_header()->set_frameid("map");
  marker->mutable_header()->mutable_timestamp()->set_sec(stamp);
  marker->mutable_header()->mutable_timestamp()->set_nsec(stamp);
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
  marker->mutable_lifetime()->set_nsec(200000000);

  adsfi_proto::viz::ColorRGBA color;
  color.set_a(1.0);
  color.set_r(0.0);
  color.set_g(1.0);
  color.set_b(0.0);

  marker->mutable_color()->CopyFrom(color);
  for (const auto& point : points) {
    auto pt = marker->add_points();
    pt->set_x(point.x());
    pt->set_y(point.y());
    pt->set_z(point.z());
  }

  if (marker->points().empty()) {
    HLOG_WARN << "empty lane line";
  }
}
#endif

void VizMap::LaneLineToMarker(
    const std::pair<uint32_t, std::vector<Eigen::Vector3d>>& lane_line,
    adsfi_proto::viz::Marker* marker) {
  static int id = 0;
  marker->Clear();
  marker->mutable_header()->set_frameid("map");
  marker->set_ns("ns_co_local_map_lane_line");
  marker->set_id(id++);
  marker->set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);  // LINE_STRIP
  marker->set_action(adsfi_proto::viz::MarkerAction::ADD);
  marker->mutable_pose()->mutable_orientation()->set_x(0.);
  marker->mutable_pose()->mutable_orientation()->set_y(0.);
  marker->mutable_pose()->mutable_orientation()->set_z(0.);
  marker->mutable_pose()->mutable_orientation()->set_w(1.);
  marker->mutable_scale()->set_x(
      0.1);  // 这里lane_line.width()的值都为0，手动改为了0.1
  marker->mutable_lifetime()->set_sec(0);
  //  marker.mutable_lifetime()->set_nsec(200000000);
  marker->mutable_lifetime()->set_nsec(200000000);

  adsfi_proto::viz::ColorRGBA color;
  color.set_a(1.0);

  color.set_r(0);
  color.set_g(1);
  color.set_b(0);
  marker->mutable_color()->CopyFrom(color);
  // 插入新填充的点
  if (lane_line.second.empty()) {
    return;
  }
  for (const auto& line : lane_line.second) {
    auto* predict_pt = marker->add_points();
    predict_pt->set_x(line.x());
    predict_pt->set_y(line.y());
    predict_pt->set_z(0);
  }
}

void VizMap::VizHqMapRoad(const std::vector<Eigen::Vector3d>& edge) {
  if (!RVIZ_AGENT.Ok()) {
    return;
  }

  adsfi_proto::viz::MarkerArray markers;
  adsfi_proto::viz::Marker marker;
  RoadEdgeToMarker(edge, &marker);
  if (!marker.points().empty()) {
    markers.add_markers()->CopyFrom(marker);
  }
  RVIZ_AGENT.Publish(viz_hqmap_road_, markers);
}

void VizMap::RoadEdgeToMarker(const std::vector<Eigen::Vector3d>& road_edge,
                              adsfi_proto::viz::Marker* marker) {
  if (marker == nullptr) {
    return;
  }

  static int id = 0;
  marker->Clear();
  marker->mutable_header()->set_frameid("map");
  marker->set_ns("ns_co_hq_map_road_line");
  marker->set_id(id++);
  marker->set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
  marker->set_action(adsfi_proto::viz::MarkerAction::ADD);
  marker->mutable_pose()->mutable_orientation()->set_x(0.);
  marker->mutable_pose()->mutable_orientation()->set_y(0.);
  marker->mutable_pose()->mutable_orientation()->set_z(0.);
  marker->mutable_pose()->mutable_orientation()->set_w(1.);
  marker->mutable_scale()->set_x(
      0.7);  // 这里lane_line.width()的值都为0，手动改为了0.1
  marker->mutable_lifetime()->set_sec(0);
  //  marker.mutable_lifetime()->set_nsec(200000000);
  marker->mutable_lifetime()->set_nsec(200000000);
  adsfi_proto::viz::ColorRGBA color;
  color.set_a(1.0);

  color.set_r(0);
  color.set_g(0);
  color.set_b(1);
  color.set_a(0.8);
  marker->mutable_color()->CopyFrom(color);
  // 插入新填充的点
  if (road_edge.empty()) {
    return;
  }
  for (const auto& road : road_edge) {
    auto* predict_pt = marker->add_points();
    predict_pt->set_x(road.x());
    predict_pt->set_y(road.y());
    predict_pt->set_z(0);
  }
}

void VizMap::VizAddSideLaneLine(
    const std::vector<std::vector<Eigen::Vector3d>>& addLaneLines) {
  if (!RVIZ_AGENT.Ok()) {
    return;
  }
  // 可视化预测的左右车道线
  adsfi_proto::viz::MarkerArray markers;
  for (const auto& add_lane : addLaneLines) {
    adsfi_proto::viz::Marker marker;
    AddLanelineToMarker(add_lane, &marker);
    if (!marker.points().empty()) {
      markers.add_markers()->CopyFrom(marker);
    }
  }
  RVIZ_AGENT.Publish(viz_add_line_, markers);
}

void VizMap::AddLanelineToMarker(const std::vector<Eigen::Vector3d>& lane_line,
                                 adsfi_proto::viz::Marker* marker) {
  static int id = 0;
  marker->Clear();
  marker->mutable_header()->set_frameid("map");
  marker->set_ns("ns_co_local_map_add_lane_line");
  marker->set_id(id++);
  marker->set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
  marker->set_action(adsfi_proto::viz::MarkerAction::ADD);
  marker->mutable_pose()->mutable_orientation()->set_x(0.);
  marker->mutable_pose()->mutable_orientation()->set_y(0.);
  marker->mutable_pose()->mutable_orientation()->set_z(0.);
  marker->mutable_pose()->mutable_orientation()->set_w(1.);
  marker->mutable_scale()->set_x(
      0.1);  // 这里lane_line.width()的值都为0，手动改为了0.1
  marker->mutable_lifetime()->set_sec(0);
  //  marker->mutable_lifetime()->set_nsec(200000000);
  marker->mutable_lifetime()->set_nsec(200000000);
  adsfi_proto::viz::ColorRGBA color;
  color.set_a(1.0);

  color.set_r(1);
  color.set_g(0);
  color.set_b(0);
  marker->mutable_color()->CopyFrom(color);
  // 插入新填充的点
  if (lane_line.empty()) {
    return;
  }
  for (const auto& add_line : lane_line) {
    auto* predict_pt = marker->add_points();
    predict_pt->set_x(add_line.x());
    predict_pt->set_y(add_line.y());
    predict_pt->set_z(0);
  }
}

void VizMap::VizAddAheadLaneLine(
    const std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>&
        aheadLaneLines) {
  if (!RVIZ_AGENT.Ok()) {
    return;
  }
  // 开始可视化前方的车道线
  adsfi_proto::viz::MarkerArray markers;
  for (const auto& ahead_lane : aheadLaneLines) {
    adsfi_proto::viz::Marker marker;
    AheadLanelineToMarker(ahead_lane, &marker);
    if (!marker.points().empty()) {
      markers.add_markers()->CopyFrom(marker);
    }
  }
  RVIZ_AGENT.Publish(viz_add_line_, markers);
}

void VizMap::AheadLanelineToMarker(
    const std::pair<uint32_t, std::vector<Eigen::Vector3d>>& lane_line,
    adsfi_proto::viz::Marker* marker) {
  static int id = 0;
  marker->Clear();
  marker->mutable_header()->set_frameid("map");
  marker->set_ns("ns_co_local_map_add_lane_line");
  marker->set_id(id++);
  marker->set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
  marker->set_action(adsfi_proto::viz::MarkerAction::ADD);
  marker->mutable_pose()->mutable_orientation()->set_x(0.);
  marker->mutable_pose()->mutable_orientation()->set_y(0.);
  marker->mutable_pose()->mutable_orientation()->set_z(0.);
  marker->mutable_pose()->mutable_orientation()->set_w(1.);
  marker->mutable_scale()->set_x(
      0.1);  // 这里lane_line.width()的值都为0，手动改为了0.1
  marker->mutable_lifetime()->set_sec(0);
  //  marker.mutable_lifetime()->set_nsec(500000000);
  marker->mutable_lifetime()->set_nsec(200000000);
  adsfi_proto::viz::ColorRGBA color;
  color.set_a(1.0);

  color.set_r(0.75);
  color.set_g(0.75);
  color.set_b(0.);
  marker->mutable_color()->CopyFrom(color);
  // 插入新填充的点
  if (lane_line.second.empty()) {
    return;
  }
  for (const auto& ahead_line : lane_line.second) {
    auto* predict_pt = marker->add_points();
    predict_pt->set_x(ahead_line.x());
    predict_pt->set_y(ahead_line.y());
    predict_pt->set_z(0);
  }
}

void VizMap::VizLaneID(const std::shared_ptr<hozon::hdmap::Map>& local_msg) {
  if (!RVIZ_AGENT.Ok()) {
    return;
  }
  // 对车道ID进行可视化
  adsfi_proto::viz::MarkerArray ID_markers;
  for (const auto& lane : local_msg->lane()) {
    Eigen::Vector3d point_utm(0, 0, 0);
    if (!lane.left_boundary().curve().segment().empty() && !lane.left_boundary()
                                                                .curve()
                                                                .segment()[0]
                                                                .line_segment()
                                                                .point()
                                                                .empty()) {
      auto point =
          lane.left_boundary().curve().segment()[0].line_segment().point()[0];
      point_utm << point.x(), point.y(), point.z();
    }

    if (lane.id().id().empty()) {
      HLOG_ERROR << "VizLaneID have empty lane id!";
      continue;
    }

    adsfi_proto::viz::Marker marker;
    int64_t laneId = std::stoll(lane.id().id());
    marker.mutable_header()->set_frameid("map");
    int remainder = static_cast<int>(laneId / 10000);
    marker.set_ns("ns_prior_map_lane" + std::to_string(remainder));
    marker.set_id(static_cast<int>(laneId % 10000));
    marker.set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
    marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
    marker.mutable_pose()->mutable_position()->set_x(point_utm.x());
    marker.mutable_pose()->mutable_position()->set_y(point_utm.y());
    marker.mutable_pose()->mutable_position()->set_z(point_utm.z());
    // if (std::isnan(point_utm.x()) || std::isnan(point_utm.y()) ||
    // std::isnan(point_utm.z()) ||
    //     std::isinf(point_utm.x()) || std::isinf(point_utm.y()) ||
    //     std::isinf(point_utm.z())) {
    //   HLOG_ERROR << "=== " << point_utm.x() << ", " << point_utm.y() << ", "
    //   << point_utm.z();
    // }

    const double text_size = 1.5;
    marker.mutable_scale()->set_z(text_size);
    marker.mutable_pose()->mutable_orientation()->set_x(0.);
    marker.mutable_pose()->mutable_orientation()->set_y(0.);
    marker.mutable_pose()->mutable_orientation()->set_z(0.);
    marker.mutable_pose()->mutable_orientation()->set_w(1.);
    marker.set_text(lane.id().id());
    marker.mutable_scale()->set_x(1.0);
    marker.mutable_scale()->set_y(1.0);
    marker.mutable_scale()->set_z(1.0);
    marker.mutable_lifetime()->set_sec(0);
    marker.mutable_lifetime()->set_nsec(200000000);
    marker.mutable_color()->set_a(1.0);
    marker.mutable_color()->set_r(1.0);
    marker.mutable_color()->set_g(1.0);
    marker.mutable_color()->set_b(1.0);
    auto* text = marker.mutable_text();
    *text = "ID: " + lane.id().id();

    ID_markers.add_markers()->CopyFrom(marker);
  }
  RVIZ_AGENT.Publish(viz_lane_id_, ID_markers);
}

// void VizMap::VizHDLaneID(
//     const std::unordered_map<std::string, LaneInfo>& lanes_in_range) {
//   if (!RVIZ_AGENT.Ok()) {
//     return;
//   }
//   // 对车道ID进行可视化
//   adsfi_proto::viz::MarkerArray ID_markers;
//   for (const auto& lane : lanes_in_range) {
//     Eigen::Vector3d point_enu(0, 0, 0);
//     if (!lane.second.left_line.empty()) {
//       auto point = lane.second.left_line.front();
//       point_enu << point.x(), point.y(), point.z();
//     }
//     if (lane.first.empty()) {
//       HLOG_ERROR << "VizLaneID have empty lane id!";
//       continue;
//     }

//     adsfi_proto::viz::Marker marker;
//     int64_t laneId = std::stoll(lane.first);
//     marker.mutable_header()->set_frameid("map");
//     int remainder = static_cast<int>(laneId / 10000);
//     marker.set_ns("ns_hd_map_lane" + std::to_string(remainder));
//     marker.set_id(static_cast<int>(laneId % 10000));
//     marker.set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
//     marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
//     marker.mutable_pose()->mutable_position()->set_x(point_enu.x());
//     marker.mutable_pose()->mutable_position()->set_y(point_enu.y());
//     marker.mutable_pose()->mutable_position()->set_z(point_enu.z());
//     // if (std::isnan(point_utm.x()) || std::isnan(point_utm.y()) ||
//     // std::isnan(point_utm.z()) ||
//     //     std::isinf(point_utm.x()) || std::isinf(point_utm.y()) ||
//     //     std::isinf(point_utm.z())) {
//     //   HLOG_ERROR << "=== " << point_utm.x() << ", " << point_utm.y() << ",
//     "
//     //   << point_utm.z();
//     // }

//     const double text_size = 2.0;
//     marker.mutable_scale()->set_z(text_size);
//     marker.mutable_pose()->mutable_orientation()->set_x(0.);
//     marker.mutable_pose()->mutable_orientation()->set_y(0.);
//     marker.mutable_pose()->mutable_orientation()->set_z(0.);
//     marker.mutable_pose()->mutable_orientation()->set_w(1.);
//     marker.set_text(lane.first);
//     marker.mutable_scale()->set_x(1.0);
//     marker.mutable_scale()->set_y(1.0);
//     marker.mutable_scale()->set_z(1.0);
//     marker.mutable_lifetime()->set_sec(0);
//     marker.mutable_lifetime()->set_nsec(0);
//     marker.mutable_color()->set_a(1.0);
//     marker.mutable_color()->set_r(0.0);
//     marker.mutable_color()->set_g(1.0);
//     marker.mutable_color()->set_b(0.0);
//     auto* text = marker.mutable_text();
//     *text = "ID: " + lane.first;

//     ID_markers.add_markers()->CopyFrom(marker);
//   }
//   RVIZ_AGENT.Publish(viz_hd_lane_id_, ID_markers);
// }

void VizMap::VizCompanLane(
    const std::vector<std::vector<Eigen::Vector3d>>& compan_lines) {
  if (!RVIZ_AGENT.Ok()) {
    return;
  }
  if (compan_lines.empty()) {
    return;
  }
  adsfi_proto::viz::MarkerArray markers;
  for (const auto& lane_line : compan_lines) {
    adsfi_proto::viz::Marker marker;
    ComLaneLineToMarker(lane_line, &marker);
    if (!marker.points().empty()) {
      markers.add_markers()->CopyFrom(marker);
    }
  }
  RVIZ_AGENT.Publish(viz_com_lane_id_, markers);
}

void VizMap::ComLaneLineToMarker(const std::vector<Eigen::Vector3d>& lane_line,
                                 adsfi_proto::viz::Marker* marker) {
  static int id = 0;
  marker->Clear();
  marker->mutable_header()->set_frameid("map");
  marker->set_ns("ns_co_local_map_lane_line");
  marker->set_id(id++);
  marker->set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);  // LINE_STRIP
  marker->set_action(adsfi_proto::viz::MarkerAction::ADD);
  marker->mutable_pose()->mutable_orientation()->set_x(0.);
  marker->mutable_pose()->mutable_orientation()->set_y(0.);
  marker->mutable_pose()->mutable_orientation()->set_z(0.);
  marker->mutable_pose()->mutable_orientation()->set_w(1.);
  marker->mutable_scale()->set_x(
      0.1);  // 这里lane_line.width()的值都为0，手动改为了0.1
  marker->mutable_lifetime()->set_sec(0);
  //  marker.mutable_lifetime()->set_nsec(200000000);
  marker->mutable_lifetime()->set_nsec(200000000);

  adsfi_proto::viz::ColorRGBA color;
  color.set_a(1.0);

  color.set_r(1);
  color.set_g(0);
  color.set_b(0);
  marker->mutable_color()->CopyFrom(color);
  // 插入新填充的点
  if (lane_line.empty()) {
    return;
  }
  for (const auto& line : lane_line) {
    auto* predict_pt = marker->add_points();
    predict_pt->set_x(line.x());
    predict_pt->set_y(line.y());
    predict_pt->set_z(0);
  }
}

void VizMap::VizCenterLane(const std::vector<Vec2d>& cent_points) {
  if (!RVIZ_AGENT.Ok()) {
    return;
  }

  if (cent_points.empty()) {
    return;
  }
  adsfi_proto::viz::MarkerArray markers;
  adsfi_proto::viz::Marker marker;
  static int id = 0;
  marker.Clear();
  marker.mutable_header()->set_frameid("map");
  marker.set_ns("ns_co_local_map_add_lane_line");
  marker.set_id(id++);
  marker.set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
  marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
  marker.mutable_pose()->mutable_orientation()->set_x(0.);
  marker.mutable_pose()->mutable_orientation()->set_y(0.);
  marker.mutable_pose()->mutable_orientation()->set_z(0.);
  marker.mutable_pose()->mutable_orientation()->set_w(1.);
  marker.mutable_scale()->set_x(
      0.3);  // 这里lane_line.width()的值都为0，手动改为了0.1
  marker.mutable_lifetime()->set_sec(0);
  //  marker.mutable_lifetime()->set_nsec(500000000);
  marker.mutable_lifetime()->set_nsec(200000000);
  adsfi_proto::viz::ColorRGBA color;
  color.set_a(0.8);

  color.set_r(1);
  color.set_g(1);
  color.set_b(1);
  marker.mutable_color()->CopyFrom(color);

  for (const auto& cen_point : cent_points) {
    auto* predict_pt = marker.add_points();
    predict_pt->set_x(cen_point.x());
    predict_pt->set_y(cen_point.y());
    predict_pt->set_z(0);
  }

  if (!marker.points().empty()) {
    markers.add_markers()->CopyFrom(marker);
  }
  RVIZ_AGENT.Publish(viz_center_lane_, markers);
}

void VizMap::VizLocalMsg(const std::shared_ptr<hozon::hdmap::Map>& local_msg,
                         const Eigen::Vector3d& pose) {
  if (!RVIZ_AGENT.Ok()) {
    return;
  }
  // 现在开始呈现所有的元素
  // 存储所有的道路边界
  std::vector<adsfi_proto::viz::MarkerArray> result =
      MapProtoMarker::LaneToMarker(local_msg, pose, false);

  // adsfi_proto::viz::MarkerArray lane = result[0];
  // adsfi_proto::viz::MarkerArray left = result[1];
  // adsfi_proto::viz::MarkerArray right = result[2];
  // adsfi_proto::viz::MarkerArray TuoPu =
  //     marker.LaneLeftNeighborForward(local_msg, pose);
  // adsfi_proto::viz::MarkerArray TuoPu2 =
  //     marker.LaneRightNeighborForward(local_msg, pose);
  // adsfi_proto::viz::MarkerArray TuoPu3 =
  //     marker.LanePredecessor(local_msg, pose);
  // adsfi_proto::viz::MarkerArray TuoPu4 =
  //     marker.LaneSuccessor(local_msg, pose);
  // RVIZ_AGENT.Publish("lane", lane);
  // RVIZ_AGENT.Publish("left", left);
  // RVIZ_AGENT.Publish("right", right);
  // RVIZ_AGENT.Publish("TuoPu", TuoPu);
  // RVIZ_AGENT.Publish("TuoPu2", TuoPu2);
  // RVIZ_AGENT.Publish("TuoPu3", TuoPu3);
  // RVIZ_AGENT.Publish("TuoPu4", TuoPu4);
  // 可视化所有的车道线
  // adsfi_proto::viz::MarkerArray markers;
  // for (const auto& lane_line : lane_line) {
  //   adsfi_proto::viz::Marker marker;
  //   LaneLineToMarker(lane_line, &marker);
  //   if (!marker.points().empty()) {
  //     markers.add_markers()->CopyFrom(marker);
  //   }
  // }

  // RVIZ_AGENT.Publish(viz_localmap_line_, markers);
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
