/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: wangjianguo
 *Date: 2023-09-06
 *****************************************************************************/
#pragma once

#include <algorithm>
#include <cfloat>
#include <cstddef>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "Eigen/Dense"
// #include "Eigen/src/Core/Matrix.h"
#include "depend/common/utm_projection/coordinate_convertor.h"
#include "depend/map/hdmap/hdmap.h"
#include "depend/proto/soc/sensor_image.pb.h"
#include "interface/adsfi_proto/viz/sensor_msgs.pb.h"
#include "modules/local_mapping/base/scene/arrow.h"
#include "modules/local_mapping/base/scene/freespace.h"
#include "modules/local_mapping/base/scene/stopline.h"
#include "modules/local_mapping/base/scene/zebracrossing.h"
#include "modules/local_mapping/utils/common.h"
#include "modules/map_fusion/include/map_fusion/map_service/global_hd_map.h"
#include "modules/util/include/util/geo.h"
#include "modules/util/include/util/mapping_log.h"
#include "modules/util/include/util/rviz_agent/rviz_agent.h"
namespace hozon {
namespace mp {
namespace lm {

class RvizUtil {
 public:
  static void PubOdom(const Eigen::Affine3d& T_W_V, const uint64_t& sec,
                      const uint64_t& nsec, const std::string& topic) {
    Eigen::Vector3d p = T_W_V.translation();
    Eigen::Quaterniond q(T_W_V.rotation());
    static bool odom_flag = true;
    if (odom_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::Odometry>(topic);
      odom_flag = false;
    }
    adsfi_proto::viz::Odometry odom_msg;
    odom_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
    odom_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    odom_msg.mutable_header()->set_frameid("localmap");
    odom_msg.set_child_frame_id("vehicle");
    odom_msg.mutable_pose()->mutable_pose()->mutable_position()->set_x(p.x());
    odom_msg.mutable_pose()->mutable_pose()->mutable_position()->set_y(p.y());
    odom_msg.mutable_pose()->mutable_pose()->mutable_position()->set_z(p.z());
    odom_msg.mutable_pose()->mutable_pose()->mutable_orientation()->set_x(
        q.x());
    odom_msg.mutable_pose()->mutable_pose()->mutable_orientation()->set_y(
        q.y());
    odom_msg.mutable_pose()->mutable_pose()->mutable_orientation()->set_z(
        q.z());
    odom_msg.mutable_pose()->mutable_pose()->mutable_orientation()->set_w(
        q.w());
    for (size_t i = 0; i < 36; ++i) {
      odom_msg.mutable_pose()->add_covariance(0.);
    }
    util::RvizAgent::Instance().Publish(topic, odom_msg);
  }

  static void PubTf(const Eigen::Affine3d& T_W_V, const uint64_t& sec,
                    const uint64_t& nsec, const std::string& topic) {
    Eigen::Vector3d p = T_W_V.translation();
    Eigen::Quaterniond q(T_W_V.rotation());
    static bool tf_flag = true;
    if (tf_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::TransformStamped>(
          topic);
      tf_flag = false;
    }
    adsfi_proto::viz::TransformStamped tf_msg;
    tf_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
    tf_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    tf_msg.mutable_header()->set_frameid("localmap");
    tf_msg.set_child_frame_id("vehicle");
    tf_msg.mutable_transform()->mutable_translation()->set_x(p.x());
    tf_msg.mutable_transform()->mutable_translation()->set_y(p.y());
    tf_msg.mutable_transform()->mutable_translation()->set_z(p.z());
    tf_msg.mutable_transform()->mutable_rotation()->set_x(q.x());
    tf_msg.mutable_transform()->mutable_rotation()->set_y(q.y());
    tf_msg.mutable_transform()->mutable_rotation()->set_z(q.z());
    tf_msg.mutable_transform()->mutable_rotation()->set_w(q.w());
    util::RvizAgent::Instance().Publish(topic, tf_msg);
  }

  static void PubPath(const Eigen::Affine3d& T_W_V, const uint64_t& sec,
                      const uint64_t& nsec, const std::string& topic) {
    Eigen::Vector3d p = T_W_V.translation();
    Eigen::Quaterniond q(T_W_V.rotation());
    static bool path_flag = true;
    if (path_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::Path>(topic);
      path_flag = false;
    }
    static adsfi_proto::viz::Path path_msg;
    auto* pose = path_msg.add_poses();
    path_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
    path_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    path_msg.mutable_header()->set_frameid("localmap");
    pose->mutable_header()->mutable_timestamp()->set_sec(sec);
    pose->mutable_header()->mutable_timestamp()->set_nsec(nsec);
    pose->mutable_header()->set_frameid("localmap");
    pose->mutable_pose()->mutable_position()->set_x(p.x());
    pose->mutable_pose()->mutable_position()->set_y(p.y());
    pose->mutable_pose()->mutable_position()->set_z(p.z());
    pose->mutable_pose()->mutable_orientation()->set_x(q.x());
    pose->mutable_pose()->mutable_orientation()->set_y(q.y());
    pose->mutable_pose()->mutable_orientation()->set_z(q.z());
    pose->mutable_pose()->mutable_orientation()->set_w(q.w());
    // if (path_msg.poses().size() > 250) {
    //   path_msg.mutable_poses()->DeleteSubrange(0, 1);
    // }
    util::RvizAgent::Instance().Publish(topic, path_msg);
  }

  static void PubDataTimeStamp(const Eigen::Affine3d& T_W_V,
                               double data_time_stamp, const uint64_t& sec,
                               const uint64_t& nsec, const std::string& topic) {
    static bool register_flag = true;
    Eigen::Vector3d p = T_W_V.translation();
    if (register_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::Marker>(topic);
      register_flag = false;
    }
    adsfi_proto::viz::Marker txt_marker;
    txt_marker.set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
    txt_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
    txt_marker.set_id(0);
    txt_marker.mutable_lifetime()->set_sec(0);
    txt_marker.mutable_lifetime()->set_nsec(200000000);
    txt_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
    txt_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    txt_marker.mutable_header()->set_frameid("localmap");
    txt_marker.mutable_pose()->mutable_position()->set_x(p.x() - 2.0);
    txt_marker.mutable_pose()->mutable_position()->set_y(p.y() + 1.0);
    txt_marker.mutable_pose()->mutable_position()->set_z(10.0);
    txt_marker.mutable_pose()->mutable_orientation()->set_x(0.);
    txt_marker.mutable_pose()->mutable_orientation()->set_y(0.);
    txt_marker.mutable_pose()->mutable_orientation()->set_z(0.);
    txt_marker.mutable_pose()->mutable_orientation()->set_w(1.);
    txt_marker.mutable_color()->set_a(1);
    txt_marker.mutable_color()->set_r(1.0);
    txt_marker.mutable_color()->set_g(0.0);
    txt_marker.mutable_color()->set_b(0.0);
    txt_marker.set_text(std::to_string(data_time_stamp));
    txt_marker.mutable_scale()->set_x(2);
    txt_marker.mutable_scale()->set_y(2);
    txt_marker.mutable_scale()->set_z(2);
    util::RvizAgent::Instance().Publish(topic, txt_marker);
  }

  static void PubPerLaneLine(const Eigen::Affine3d& T_W_V,
                             const std::vector<LaneLine>& per_lane_lines,
                             const uint64_t& sec, const uint64_t& nsec,
                             const std::string& topic) {
    static bool register_flag = true;
    if (register_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::PointCloud>(topic);
      register_flag = false;
    }
    adsfi_proto::viz::PointCloud points_msg;
    points_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
    points_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    points_msg.mutable_header()->set_frameid("localmap");
    auto* channels = points_msg.add_channels();
    channels->set_name("rgb");
    for (const auto& lane_line : per_lane_lines) {
      for (const auto& point : lane_line.vehicle_points) {
        auto point_world = T_W_V * point;
        auto* point_msg = points_msg.add_points();
        point_msg->set_x(static_cast<float>(point_world.x()));
        point_msg->set_y(static_cast<float>(point_world.y()));
        point_msg->set_z(static_cast<float>(point_world.z()));
      }
    }
    util::RvizAgent::Instance().Publish(topic, points_msg);
  }

  static void PubPerRoadEdge(const Eigen::Affine3d& T_W_V,
                             const std::vector<RoadEdge>& per_road_edges,
                             const uint64_t& sec, const uint64_t& nsec,
                             const std::string& topic) {
    static bool register_flag = true;
    if (register_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::PointCloud>(topic);
      register_flag = false;
    }
    adsfi_proto::viz::PointCloud points_msg;
    points_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
    points_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    points_msg.mutable_header()->set_frameid("localmap");
    auto* channels = points_msg.add_channels();
    channels->set_name("rgb");
    for (const auto& road_edge : per_road_edges) {
      for (const auto& point : road_edge.vehicle_points) {
        auto point_world = T_W_V * point;
        auto* point_msg = points_msg.add_points();
        point_msg->set_x(static_cast<float>(point_world.x()));
        point_msg->set_y(static_cast<float>(point_world.y()));
        point_msg->set_z(static_cast<float>(point_world.z()));
      }
    }
    util::RvizAgent::Instance().Publish(topic, points_msg);
  }

  static void PubPerOccEdge(const Eigen::Affine3d& T_W_V,
                            const std::vector<OccEdge>& per_occ_edges,
                            const uint64_t& sec, const uint64_t& nsec,
                            const std::string& topic) {
    static bool register_flag = true;
    if (register_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::PointCloud>(topic);
      register_flag = false;
    }
    adsfi_proto::viz::PointCloud points_msg;
    points_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
    points_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    points_msg.mutable_header()->set_frameid("localmap");
    auto* channels = points_msg.add_channels();
    channels->set_name("rgb");
    for (const auto& occ_edge : per_occ_edges) {
      for (const auto& point : occ_edge.vehicle_points) {
        auto point_world = T_W_V * point;
        auto* point_msg = points_msg.add_points();
        point_msg->set_x(static_cast<float>(point_world.x()));
        point_msg->set_y(static_cast<float>(point_world.y()));
        point_msg->set_z(static_cast<float>(point_world.z()));
      }
    }
    util::RvizAgent::Instance().Publish(topic, points_msg);
  }

  static void PubPerOccEdgeMarker(const Eigen::Affine3d& T_W_V,
                                  const std::vector<OccEdge>& map_occ_edges,
                                  const uint64_t& sec, const uint64_t& nsec,
                                  const std::string& topic) {
    static bool register_flag = true;
    if (register_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::MarkerArray>(
          topic);
      register_flag = false;
    }
    adsfi_proto::viz::MarkerArray markers;
    int id = 0;
    for (const auto& occ_edge : map_occ_edges) {
      if (occ_edge.vehicle_points.empty()) {
        continue;
      }
      Eigen::Vector3d txt_point{occ_edge.vehicle_points.back().x(),
                                occ_edge.vehicle_points.back().y(), 0};
      txt_point = T_W_V * txt_point;
      adsfi_proto::viz::Marker point_marker;
      point_marker.mutable_header()->set_frameid("localmap");
      point_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
      point_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
      point_marker.set_id(id++);
      point_marker.set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
      point_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
      point_marker.mutable_pose()->mutable_orientation()->set_x(0.);
      point_marker.mutable_pose()->mutable_orientation()->set_y(0.);
      point_marker.mutable_pose()->mutable_orientation()->set_z(0.);
      point_marker.mutable_pose()->mutable_orientation()->set_w(1.);
      point_marker.mutable_scale()->set_x(0.2);
      point_marker.mutable_scale()->set_y(0.2);
      point_marker.mutable_scale()->set_z(0.2);
      point_marker.mutable_lifetime()->set_sec(0);
      point_marker.mutable_lifetime()->set_nsec(200000000);
      point_marker.mutable_color()->set_a(1.0);
      point_marker.mutable_color()->set_r(1.0);
      point_marker.mutable_color()->set_g(0.75);
      point_marker.mutable_color()->set_b(0.80);
      for (const auto& point : occ_edge.vehicle_points) {
        auto point_world = T_W_V * point;
        auto* point_msg = point_marker.add_points();
        point_msg->set_x(point_world.x());
        point_msg->set_y(point_world.y());
        point_msg->set_z(point_world.z());
      }
      markers.add_markers()->CopyFrom(point_marker);

      adsfi_proto::viz::Marker txt_marker;
      txt_marker.set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
      txt_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
      txt_marker.set_id(id++);
      txt_marker.mutable_lifetime()->set_sec(0);
      txt_marker.mutable_lifetime()->set_nsec(200000000);
      txt_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
      txt_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
      txt_marker.mutable_header()->set_frameid("localmap");
      txt_marker.mutable_pose()->mutable_position()->set_x(txt_point[0]);
      txt_marker.mutable_pose()->mutable_position()->set_y(txt_point[1]);
      txt_marker.mutable_pose()->mutable_position()->set_z(2);
      txt_marker.mutable_pose()->mutable_orientation()->set_x(0.);
      txt_marker.mutable_pose()->mutable_orientation()->set_y(0.);
      txt_marker.mutable_pose()->mutable_orientation()->set_z(0.);
      txt_marker.mutable_pose()->mutable_orientation()->set_w(1.);
      txt_marker.mutable_color()->set_a(1);
      txt_marker.mutable_color()->set_r(1.0);
      txt_marker.mutable_color()->set_g(0.75);
      txt_marker.mutable_color()->set_b(0.8);
      txt_marker.set_text(std::to_string(static_cast<int>(occ_edge.detect_id)));
      txt_marker.mutable_scale()->set_x(2);
      txt_marker.mutable_scale()->set_y(2);
      txt_marker.mutable_scale()->set_z(2);
      markers.add_markers()->CopyFrom(txt_marker);
    }
    util::RvizAgent::Instance().Publish(topic, markers);
  }

  static void PubPerStopLine(const Eigen::Affine3d& T_W_V,
                             const std::vector<StopLine>& stop_lines,
                             const uint64_t& sec, const uint64_t& nsec,
                             const std::string& topic) {
    static bool register_flag = true;
    if (register_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::MarkerArray>(
          topic);
      register_flag = false;
    }
    adsfi_proto::viz::MarkerArray markers;
    int id = 0;
    for (const auto& stop_line : stop_lines) {
      adsfi_proto::viz::Marker marker;
      marker.mutable_header()->set_frameid("localmap");
      marker.mutable_header()->mutable_timestamp()->set_sec(sec);
      marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
      marker.set_id(id++);
      marker.set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
      marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
      marker.mutable_pose()->mutable_position()->set_x(0);
      marker.mutable_pose()->mutable_position()->set_y(0);
      marker.mutable_pose()->mutable_position()->set_z(0);
      marker.mutable_pose()->mutable_orientation()->set_x(0.);
      marker.mutable_pose()->mutable_orientation()->set_y(0.);
      marker.mutable_pose()->mutable_orientation()->set_z(0.);
      marker.mutable_pose()->mutable_orientation()->set_w(1.);
      marker.mutable_scale()->set_x(0.2);
      marker.mutable_scale()->set_y(0.2);
      marker.mutable_scale()->set_z(0.2);
      marker.mutable_lifetime()->set_sec(0);
      marker.mutable_lifetime()->set_nsec(200000000);
      adsfi_proto::viz::ColorRGBA color;
      color.set_a(1.0);
      color.set_r(1.0);
      color.set_g(0.0);
      color.set_b(0.0);
      marker.mutable_color()->CopyFrom(color);
      auto point = T_W_V * stop_line.left_point;
      auto* left_point = marker.add_points();
      left_point->set_x(static_cast<float>(point.x()));
      left_point->set_y(static_cast<float>(point.y()));
      left_point->set_z(static_cast<float>(point.z()));
      point = T_W_V * stop_line.right_point;
      auto* right_point = marker.add_points();
      right_point->set_x(static_cast<float>(point.x()));
      right_point->set_y(static_cast<float>(point.y()));
      right_point->set_z(static_cast<float>(point.z()));
      markers.add_markers()->CopyFrom(marker);
    }
    util::RvizAgent::Instance().Publish(topic, markers);
  }

  static void PubPerArrow(const Eigen::Affine3d& T_W_V,
                          const std::vector<Arrow>& per_arrows,
                          const uint64_t& sec, const uint64_t& nsec,
                          const std::string& topic) {
    static bool register_flag = true;
    if (register_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::MarkerArray>(
          topic);
      register_flag = false;
    }
    adsfi_proto::viz::MarkerArray markers;
    int id = 0;
    for (const auto& arrow : per_arrows) {
      adsfi_proto::viz::Marker marker;
      marker.mutable_header()->set_frameid("localmap");
      marker.mutable_header()->mutable_timestamp()->set_sec(sec);
      marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
      marker.set_id(id++);
      marker.set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
      marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
      marker.mutable_pose()->mutable_position()->set_x(0);
      marker.mutable_pose()->mutable_position()->set_y(0);
      marker.mutable_pose()->mutable_position()->set_z(0);
      marker.mutable_pose()->mutable_orientation()->set_x(0.);
      marker.mutable_pose()->mutable_orientation()->set_y(0.);
      marker.mutable_pose()->mutable_orientation()->set_z(0.);
      marker.mutable_pose()->mutable_orientation()->set_w(1.);
      marker.mutable_scale()->set_x(0.2);
      marker.mutable_scale()->set_y(0.2);
      marker.mutable_scale()->set_z(0.2);
      marker.mutable_lifetime()->set_sec(0);
      marker.mutable_lifetime()->set_nsec(200000000);
      adsfi_proto::viz::ColorRGBA color;
      color.set_a(1.0);
      color.set_r(1.0);
      color.set_g(0.0);
      color.set_b(0.0);
      marker.mutable_color()->CopyFrom(color);
      Eigen::Vector3d point_0 = arrow.vehicle_points[0];
      point_0 = T_W_V * point_0;
      Eigen::Vector3d point_1 = arrow.vehicle_points[1];
      point_1 = T_W_V * point_1;
      Eigen::Vector3d point_2 = arrow.vehicle_points[2];
      point_2 = T_W_V * point_2;
      Eigen::Vector3d point_3 = arrow.vehicle_points[3];
      point_3 = T_W_V * point_3;
      auto* point_msg = marker.add_points();
      point_msg->set_x(point_0.x());
      point_msg->set_y(point_0.y());
      point_msg = marker.add_points();
      point_msg->set_x(point_1.x());
      point_msg->set_y(point_1.y());
      point_msg = marker.add_points();
      point_msg->set_x(point_2.x());
      point_msg->set_y(point_2.y());
      point_msg = marker.add_points();
      point_msg->set_x(point_3.x());
      point_msg->set_y(point_3.y());
      point_msg = marker.add_points();
      point_msg->set_x(point_0.x());
      point_msg->set_y(point_0.y());
      markers.add_markers()->CopyFrom(marker);
      adsfi_proto::viz::Marker txt_marker;
      txt_marker.set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
      txt_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
      txt_marker.set_id(id++);
      txt_marker.mutable_lifetime()->set_sec(0);
      txt_marker.mutable_lifetime()->set_nsec(200000000);
      txt_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
      txt_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
      txt_marker.mutable_header()->set_frameid("localmap");
      txt_marker.mutable_pose()->mutable_position()->set_x(
          (point_0.x() + point_2.x()) / 2);
      txt_marker.mutable_pose()->mutable_position()->set_y(
          (point_0.y() + point_2.y()) / 2);
      txt_marker.mutable_pose()->mutable_orientation()->set_x(0);
      txt_marker.mutable_pose()->mutable_orientation()->set_y(0);
      txt_marker.mutable_pose()->mutable_orientation()->set_z(0);
      txt_marker.mutable_pose()->mutable_orientation()->set_w(1);
      txt_marker.mutable_color()->set_a(1);
      txt_marker.mutable_color()->set_r(1);
      txt_marker.mutable_color()->set_g(0);
      txt_marker.mutable_color()->set_b(0);
      std::map<ArrowType, std::string> ArrowTypeToString{
          {ArrowType::UNKNOWN, "ARROWTYPE_UNKNOWN"},
          {ArrowType::STRAIGHT_FORWARD, "STRAIGHT_FORWARD"},
          {ArrowType::STRAIGHT_FORWARD_OR_TURN_LEFT,
           "STRAIGHT_FORWARD_OR_TURN_LEFT"},
          {ArrowType::STRAIGHT_FORWARD_OR_TURN_RIGHT,
           "STRAIGHT_FORWARD_OR_TURN_RIGHT"},
          {ArrowType::STRAIGHT_FORWARD_OR_TURN_LEFT_OR_TURN_RIGHT,
           "STRAIGHT_FORWARD_OR_TURN_LEFT_OR_TURN_RIGHT"},
          {ArrowType::STRAIGHT_FORWARD_OR_TURN_AROUND,
           "STRAIGHT_FORWARD_OR_TURN_AROUND"},
          {ArrowType::STRAIGHT_FORWARD_OR_TURN_AROUND_OR_TURN_LEFT,
           "STRAIGHT_FORWARD_OR_TURN_AROUND_OR_TURN_LEFT"},
          {ArrowType::TURN_LEFT, "TURN_LEFT"},
          {ArrowType::TURN_LEFT_OR_MERGE_LEFT, "TURN_LEFT_OR_MERGE_LEFT"},
          {ArrowType::TURN_LEFT_OR_TURN_AROUND, "TURN_LEFT_OR_TURN_AROUND"},
          {ArrowType::TURN_LEFT_OR_TURN_RIGHT, "TURN_LEFT_OR_TURN_RIGHT"},
          {ArrowType::TURN_RIGHT, "TURN_RIGHT"},
          {ArrowType::TURN_RIGHT_OR_MERGE_RIGHT, "TURN_RIGHT_OR_MERGE_RIGHT"},
          {ArrowType::TURN_RIGHT_OR_TURN_AROUND, "TURN_RIGHT_OR_TURN_AROUND"},
          {ArrowType::TURN_AROUND, "TURN_AROUND"},
          {ArrowType::FORBID_TURN_LEFT, "FORBID_TURN_LEFT"},
          {ArrowType::FORBID_TURN_RIGHT, "FORBID_TURN_RIGHT"},
          {ArrowType::FORBID_TURN_AROUND, "FORBID_TURN_AROUND"},
          {ArrowType::FRONT_NEAR_CROSSWALK, "FRONT_NEAR_CROSSWALK"},
      };
      txt_marker.set_text(ArrowTypeToString[arrow.type]);
      txt_marker.mutable_scale()->set_x(1);
      txt_marker.mutable_scale()->set_y(1);
      txt_marker.mutable_scale()->set_z(0.5);
      markers.add_markers()->CopyFrom(txt_marker);
    }
    util::RvizAgent::Instance().Publish(topic, markers);
  }

  static void PubPerZebraCrossing(
      const Eigen::Affine3d& T_W_V,
      const std::vector<ZebraCrossing>& per_zebra_crossings,
      const uint64_t& sec, const uint64_t& nsec, const std::string& topic) {
    static bool register_flag = true;
    if (register_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::MarkerArray>(
          topic);
      register_flag = false;
    }
    adsfi_proto::viz::MarkerArray markers;
    int id = 0;
    for (const auto& zebra_crossing : per_zebra_crossings) {
      adsfi_proto::viz::Marker marker;
      marker.mutable_header()->set_frameid("localmap");
      marker.mutable_header()->mutable_timestamp()->set_sec(sec);
      marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
      marker.set_id(id++);
      marker.set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
      marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
      marker.mutable_pose()->mutable_position()->set_x(0);
      marker.mutable_pose()->mutable_position()->set_y(0);
      marker.mutable_pose()->mutable_position()->set_z(0);
      marker.mutable_pose()->mutable_orientation()->set_x(0.);
      marker.mutable_pose()->mutable_orientation()->set_y(0.);
      marker.mutable_pose()->mutable_orientation()->set_z(0.);
      marker.mutable_pose()->mutable_orientation()->set_w(1.);
      marker.mutable_scale()->set_x(0.2);
      marker.mutable_scale()->set_y(0.2);
      marker.mutable_scale()->set_z(0.2);
      marker.mutable_lifetime()->set_sec(0);
      marker.mutable_lifetime()->set_nsec(200000000);
      adsfi_proto::viz::ColorRGBA color;
      color.set_a(1.0);
      color.set_r(1.0);
      color.set_g(0.0);
      color.set_b(0.0);
      marker.mutable_color()->CopyFrom(color);
      Eigen::Vector3d point_0 = zebra_crossing.vehicle_points[0];
      point_0 = T_W_V * point_0;
      Eigen::Vector3d point_1 = zebra_crossing.vehicle_points[1];
      point_1 = T_W_V * point_1;
      Eigen::Vector3d point_2 = zebra_crossing.vehicle_points[2];
      point_2 = T_W_V * point_2;
      Eigen::Vector3d point_3 = zebra_crossing.vehicle_points[3];
      point_3 = T_W_V * point_3;
      auto* point_msg = marker.add_points();
      point_msg->set_x(point_0.x());
      point_msg->set_y(point_0.y());
      point_msg = marker.add_points();
      point_msg->set_x(point_1.x());
      point_msg->set_y(point_1.y());
      point_msg = marker.add_points();
      point_msg->set_x(point_2.x());
      point_msg->set_y(point_2.y());
      point_msg = marker.add_points();
      point_msg->set_x(point_3.x());
      point_msg->set_y(point_3.y());
      point_msg = marker.add_points();
      point_msg->set_x(point_0.x());
      point_msg->set_y(point_0.y());
      markers.add_markers()->CopyFrom(marker);
      adsfi_proto::viz::Marker txt_marker;
      txt_marker.set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
      txt_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
      txt_marker.set_id(id++);
      txt_marker.mutable_lifetime()->set_sec(0);
      txt_marker.mutable_lifetime()->set_nsec(200000000);
      txt_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
      txt_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
      txt_marker.mutable_header()->set_frameid("localmap");
      txt_marker.mutable_pose()->mutable_position()->set_x(
          (point_0.x() + point_2.x()) / 2);
      txt_marker.mutable_pose()->mutable_position()->set_y(
          (point_0.y() + point_2.y()) / 2);
      txt_marker.mutable_pose()->mutable_orientation()->set_x(0);
      txt_marker.mutable_pose()->mutable_orientation()->set_y(0);
      txt_marker.mutable_pose()->mutable_orientation()->set_z(0);
      txt_marker.mutable_pose()->mutable_orientation()->set_w(1);
      txt_marker.mutable_color()->set_a(1);
      txt_marker.mutable_color()->set_r(1);
      txt_marker.mutable_color()->set_g(0);
      txt_marker.mutable_color()->set_b(0);
      txt_marker.set_text("zebra_crossing");
      txt_marker.mutable_scale()->set_x(1);
      txt_marker.mutable_scale()->set_y(1);
      txt_marker.mutable_scale()->set_z(0.5);
      markers.add_markers()->CopyFrom(txt_marker);
    }

    util::RvizAgent::Instance().Publish(topic, markers);
  }

  static void PubMapLaneLine(const Eigen::Affine3d& T_W_V,
                             const std::vector<LaneLine>& map_lane_lines,
                             const uint64_t& sec, const uint64_t& nsec,
                             const std::string& topic) {
    static bool register_flag = true;
    if (register_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::PointCloud>(topic);
      register_flag = false;
    }
    adsfi_proto::viz::PointCloud points_msg;
    points_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
    points_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    points_msg.mutable_header()->set_frameid("localmap");
    auto* channels = points_msg.add_channels();
    channels->set_name("rgb");
    for (const auto& lane_line : map_lane_lines) {
      if (lane_line.state == TrackState::NOTMATURED) {
        continue;
      }
      for (const auto& point : lane_line.vehicle_points) {
        auto point_world = T_W_V * point;
        auto* point_msg = points_msg.add_points();
        point_msg->set_x(static_cast<float>(point_world.x()));
        point_msg->set_y(static_cast<float>(point_world.y()));
        point_msg->set_z(static_cast<float>(point_world.z()));
      }
    }
    util::RvizAgent::Instance().Publish(topic, points_msg);
  }

  static void PubImmatureMapLaneLine(
      const Eigen::Affine3d& T_W_V, const std::vector<LaneLine>& map_lane_lines,
      const uint64_t& sec, const uint64_t& nsec, const std::string& topic) {
    static bool register_flag = true;
    if (register_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::PointCloud>(topic);
      register_flag = false;
    }
    adsfi_proto::viz::PointCloud points_msg;
    points_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
    points_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    points_msg.mutable_header()->set_frameid("localmap");
    auto* channels = points_msg.add_channels();
    channels->set_name("rgb");
    for (const auto& lane_line : map_lane_lines) {
      if (lane_line.state == TrackState::MATURED) {
        continue;
      }
      for (const auto& point : lane_line.vehicle_points) {
        auto point_world = T_W_V * point;
        auto* point_msg = points_msg.add_points();
        point_msg->set_x(static_cast<float>(point_world.x()));
        point_msg->set_y(static_cast<float>(point_world.y()));
        point_msg->set_z(static_cast<float>(point_world.z()));
      }
    }
    util::RvizAgent::Instance().Publish(topic, points_msg);
  }

  static void PubMapLaneLineMarker(const Eigen::Affine3d& T_W_V,
                                   const std::vector<LaneLine>& map_lane_lines,
                                   const uint64_t& sec, const uint64_t& nsec,
                                   const std::string& topic) {
    static bool register_flag = true;
    if (register_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::MarkerArray>(
          topic);
      register_flag = false;
    }
    adsfi_proto::viz::MarkerArray markers;
    int id = 0;
    for (const auto& lane_line : map_lane_lines) {
      if (lane_line.state == TrackState::NOTMATURED) {
        continue;
      }
      if (lane_line.vehicle_points.empty()) {
        continue;
      }
      double min_abs_x = FLT_MAX;
      double c0 = 0;
      for (const auto& point : lane_line.vehicle_points) {
        if (point.x() < min_abs_x) {
          c0 = point.y();
          min_abs_x = point.x();
        }
      }
      Eigen::Vector3d txt_point{lane_line.vehicle_points.back().x(),
                                lane_line.vehicle_points.back().y(), 0};
      txt_point = T_W_V * txt_point;
      adsfi_proto::viz::Marker point_marker;
      point_marker.mutable_header()->set_frameid("localmap");
      point_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
      point_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
      point_marker.set_id(id++);
      point_marker.set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
      point_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
      point_marker.mutable_pose()->mutable_orientation()->set_x(0.);
      point_marker.mutable_pose()->mutable_orientation()->set_y(0.);
      point_marker.mutable_pose()->mutable_orientation()->set_z(0.);
      point_marker.mutable_pose()->mutable_orientation()->set_w(1.);
      point_marker.mutable_scale()->set_x(0.2);
      point_marker.mutable_scale()->set_y(0.2);
      point_marker.mutable_scale()->set_z(0.2);
      point_marker.mutable_lifetime()->set_sec(0);
      point_marker.mutable_lifetime()->set_nsec(200000000);
      point_marker.mutable_color()->set_a(1.0);
      point_marker.mutable_color()->set_r(0.99);
      point_marker.mutable_color()->set_g(0.69);
      point_marker.mutable_color()->set_b(0.24);
      for (const auto& point : lane_line.vehicle_points) {
        auto point_world = T_W_V * point;
        auto* point_msg = point_marker.add_points();
        point_msg->set_x(point_world.x());
        point_msg->set_y(point_world.y());
        point_msg->set_z(point_world.z());
      }
      markers.add_markers()->CopyFrom(point_marker);

      adsfi_proto::viz::Marker txt_marker;
      txt_marker.set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
      txt_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
      txt_marker.set_id(id++);
      txt_marker.mutable_lifetime()->set_sec(0);
      txt_marker.mutable_lifetime()->set_nsec(200000000);
      txt_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
      txt_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
      txt_marker.mutable_header()->set_frameid("localmap");
      txt_marker.mutable_pose()->mutable_orientation()->set_x(0.);
      txt_marker.mutable_pose()->mutable_orientation()->set_y(0.);
      txt_marker.mutable_pose()->mutable_orientation()->set_z(0.);
      txt_marker.mutable_pose()->mutable_orientation()->set_w(1.);
      txt_marker.mutable_pose()->mutable_position()->set_x(txt_point[0]);
      txt_marker.mutable_pose()->mutable_position()->set_y(txt_point[1]);
      txt_marker.mutable_pose()->mutable_position()->set_z(2);
      txt_marker.mutable_color()->set_r(0);
      txt_marker.mutable_color()->set_g(1);
      txt_marker.mutable_color()->set_b(0);
      txt_marker.mutable_color()->set_a(1);
      txt_marker.set_text(std::to_string(lane_line.id) + ", " +
                          std::to_string(static_cast<int>(lane_line.position)) +
                          ", " +
                          std::to_string(static_cast<int>(lane_line.type)));
      txt_marker.mutable_scale()->set_x(2);
      txt_marker.mutable_scale()->set_y(2);
      txt_marker.mutable_scale()->set_z(2);
      markers.add_markers()->CopyFrom(txt_marker);
    }
    util::RvizAgent::Instance().Publish(topic, markers);
  }

  static void PubImmatureMapLaneLineMarker(
      const Eigen::Affine3d& T_W_V, const std::vector<LaneLine>& map_lane_lines,
      const uint64_t& sec, const uint64_t& nsec, const std::string& topic) {
    static bool register_flag = true;
    if (register_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::MarkerArray>(
          topic);
      register_flag = false;
    }
    adsfi_proto::viz::MarkerArray markers;
    int id = 0;
    for (const auto& lane_line : map_lane_lines) {
      if (lane_line.state == TrackState::MATURED) {
        continue;
      }
      if (lane_line.vehicle_points.empty()) {
        continue;
      }
      double min_abs_x = FLT_MAX;
      double c0 = 0;
      for (const auto& point : lane_line.vehicle_points) {
        if (point.x() < min_abs_x) {
          c0 = point.y();
          min_abs_x = point.x();
        }
      }
      Eigen::Vector3d txt_point{lane_line.vehicle_points.back().x(),
                                lane_line.vehicle_points.back().y(), 0};
      txt_point = T_W_V * txt_point;
      adsfi_proto::viz::Marker point_marker;
      point_marker.mutable_header()->set_frameid("localmap");
      point_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
      point_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
      point_marker.set_id(id++);
      point_marker.set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
      point_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
      point_marker.mutable_pose()->mutable_orientation()->set_x(0.);
      point_marker.mutable_pose()->mutable_orientation()->set_y(0.);
      point_marker.mutable_pose()->mutable_orientation()->set_z(0.);
      point_marker.mutable_pose()->mutable_orientation()->set_w(1.);
      point_marker.mutable_scale()->set_x(0.2);
      point_marker.mutable_scale()->set_y(0.2);
      point_marker.mutable_scale()->set_z(0.2);
      point_marker.mutable_lifetime()->set_sec(0);
      point_marker.mutable_lifetime()->set_nsec(200000000);
      point_marker.mutable_color()->set_a(1.0);
      point_marker.mutable_color()->set_r(0.68);
      point_marker.mutable_color()->set_g(0.50);
      point_marker.mutable_color()->set_b(0.66);
      for (const auto& point : lane_line.vehicle_points) {
        auto point_world = T_W_V * point;
        auto* point_msg = point_marker.add_points();
        point_msg->set_x(point_world.x());
        point_msg->set_y(point_world.y());
        point_msg->set_z(point_world.z());
      }
      markers.add_markers()->CopyFrom(point_marker);

      adsfi_proto::viz::Marker txt_marker;
      txt_marker.set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
      txt_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
      txt_marker.set_id(id++);
      txt_marker.mutable_lifetime()->set_sec(0);
      txt_marker.mutable_lifetime()->set_nsec(200000000);
      txt_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
      txt_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
      txt_marker.mutable_header()->set_frameid("localmap");
      txt_marker.mutable_pose()->mutable_orientation()->set_x(0.);
      txt_marker.mutable_pose()->mutable_orientation()->set_y(0.);
      txt_marker.mutable_pose()->mutable_orientation()->set_z(0.);
      txt_marker.mutable_pose()->mutable_orientation()->set_w(1.);
      txt_marker.mutable_pose()->mutable_position()->set_x(txt_point[0]);
      txt_marker.mutable_pose()->mutable_position()->set_y(txt_point[1]);
      txt_marker.mutable_pose()->mutable_position()->set_z(2);
      txt_marker.mutable_color()->set_r(0);
      txt_marker.mutable_color()->set_g(1);
      txt_marker.mutable_color()->set_b(0);
      txt_marker.mutable_color()->set_a(1);
      txt_marker.set_text(std::to_string(lane_line.tracked_count));
      txt_marker.mutable_scale()->set_x(2);
      txt_marker.mutable_scale()->set_y(2);
      txt_marker.mutable_scale()->set_z(2);
      markers.add_markers()->CopyFrom(txt_marker);
    }
    util::RvizAgent::Instance().Publish(topic, markers);
  }

  static void PubMapImmatureOccEdge(const Eigen::Affine3d& T_W_V,
                                    const std::vector<OccEdge>& map_occ_edges,
                                    const uint64_t& sec, const uint64_t& nsec,
                                    const std::string& topic) {
    static bool register_flag = true;
    if (register_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::PointCloud>(topic);
      register_flag = false;
    }
    adsfi_proto::viz::PointCloud points_msg;
    points_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
    points_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    points_msg.mutable_header()->set_frameid("localmap");
    auto* channels = points_msg.add_channels();
    channels->set_name("rgb");
    for (const auto& occ_edge : map_occ_edges) {
      for (const auto& point : occ_edge.vehicle_points) {
        auto point_world = T_W_V * point;
        auto* point_msg = points_msg.add_points();
        point_msg->set_x(static_cast<float>(point_world.x()));
        point_msg->set_y(static_cast<float>(point_world.y()));
        point_msg->set_z(static_cast<float>(point_world.z()));
      }
    }
    util::RvizAgent::Instance().Publish(topic, points_msg);
  }

  static void PubMapOccEdgeMarker(const Eigen::Affine3d& T_W_V,
                                  const std::vector<OccEdge>& map_occ_edges,
                                  const uint64_t& sec, const uint64_t& nsec,
                                  const std::string& topic) {
    static bool register_flag = true;
    if (register_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::MarkerArray>(
          topic);
      register_flag = false;
    }
    adsfi_proto::viz::MarkerArray markers;
    int id = 0;
    for (const auto& occ_edge : map_occ_edges) {
      if (occ_edge.vehicle_points.empty()) {
        continue;
      }
      if (occ_edge.state == TrackState::NOTMATURED) {
        continue;
      }
      Eigen::Vector3d txt_point{occ_edge.vehicle_points.back().x(),
                                occ_edge.vehicle_points.back().y(), 0};
      txt_point = T_W_V * txt_point;
      adsfi_proto::viz::Marker point_marker;
      point_marker.mutable_header()->set_frameid("localmap");
      point_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
      point_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
      point_marker.set_id(id++);
      point_marker.set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
      point_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
      point_marker.mutable_pose()->mutable_orientation()->set_x(0.);
      point_marker.mutable_pose()->mutable_orientation()->set_y(0.);
      point_marker.mutable_pose()->mutable_orientation()->set_z(0.);
      point_marker.mutable_pose()->mutable_orientation()->set_w(1.);
      point_marker.mutable_scale()->set_x(0.2);
      point_marker.mutable_scale()->set_y(0.2);
      point_marker.mutable_scale()->set_z(0.2);
      point_marker.mutable_lifetime()->set_sec(0);
      point_marker.mutable_lifetime()->set_nsec(200000000);
      point_marker.mutable_color()->set_a(1.0);
      point_marker.mutable_color()->set_r(0.0);
      point_marker.mutable_color()->set_g(1.0);
      point_marker.mutable_color()->set_b(0.0);
      for (const auto& point : occ_edge.fit_points) {
        auto point_world = T_W_V * point;
        auto* point_msg = point_marker.add_points();
        point_msg->set_x(point_world.x());
        point_msg->set_y(point_world.y());
        point_msg->set_z(point_world.z());
      }
      markers.add_markers()->CopyFrom(point_marker);

      adsfi_proto::viz::Marker txt_marker;
      txt_marker.set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
      txt_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
      txt_marker.set_id(id++);
      txt_marker.mutable_lifetime()->set_sec(0);
      txt_marker.mutable_lifetime()->set_nsec(200000000);
      txt_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
      txt_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
      txt_marker.mutable_header()->set_frameid("localmap");
      txt_marker.mutable_pose()->mutable_position()->set_x(txt_point[0]);
      txt_marker.mutable_pose()->mutable_position()->set_y(txt_point[1]);
      txt_marker.mutable_pose()->mutable_position()->set_z(2);
      txt_marker.mutable_pose()->mutable_orientation()->set_x(0.);
      txt_marker.mutable_pose()->mutable_orientation()->set_y(0.);
      txt_marker.mutable_pose()->mutable_orientation()->set_z(0.);
      txt_marker.mutable_pose()->mutable_orientation()->set_w(1.);
      txt_marker.mutable_color()->set_a(1);
      txt_marker.mutable_color()->set_r(0.0);
      txt_marker.mutable_color()->set_g(1.0);
      txt_marker.mutable_color()->set_b(0.0);
      txt_marker.set_text(std::to_string(occ_edge.id) + ", " +
                          std::to_string(static_cast<int>(occ_edge.detect_id)));
      txt_marker.mutable_scale()->set_x(2);
      txt_marker.mutable_scale()->set_y(2);
      txt_marker.mutable_scale()->set_z(2);
      markers.add_markers()->CopyFrom(txt_marker);
    }
    util::RvizAgent::Instance().Publish(topic, markers);
  }

  static void PubMapImmatureOccEdgeMarker(
      const Eigen::Affine3d& T_W_V, const std::vector<OccEdge>& map_occ_edges,
      const uint64_t& sec, const uint64_t& nsec, const std::string& topic) {
    static bool register_flag = true;
    if (register_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::MarkerArray>(
          topic);
      register_flag = false;
    }
    adsfi_proto::viz::MarkerArray markers;
    int id = 0;
    for (const auto& occ_edge : map_occ_edges) {
      if (occ_edge.vehicle_points.empty()) {
        continue;
      }
      if (occ_edge.state == TrackState::MATURED) {
        continue;
      }
      Eigen::Vector3d txt_point{occ_edge.vehicle_points.back().x(),
                                occ_edge.vehicle_points.back().y(), 0};
      txt_point = T_W_V * txt_point;
      adsfi_proto::viz::Marker point_marker;
      point_marker.mutable_header()->set_frameid("localmap");
      point_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
      point_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
      point_marker.set_id(id++);
      point_marker.set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
      point_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
      point_marker.mutable_pose()->mutable_orientation()->set_x(0.);
      point_marker.mutable_pose()->mutable_orientation()->set_y(0.);
      point_marker.mutable_pose()->mutable_orientation()->set_z(0.);
      point_marker.mutable_pose()->mutable_orientation()->set_w(1.);
      point_marker.mutable_scale()->set_x(0.2);
      point_marker.mutable_scale()->set_y(0.2);
      point_marker.mutable_scale()->set_z(0.2);
      point_marker.mutable_lifetime()->set_sec(0);
      point_marker.mutable_lifetime()->set_nsec(200000000);
      point_marker.mutable_color()->set_a(1.0);
      point_marker.mutable_color()->set_r(0.68);
      point_marker.mutable_color()->set_g(0.85);
      point_marker.mutable_color()->set_b(0.90);
      // for (const auto& point : occ_edge.vehicle_points) {
      //   auto point_world = T_W_V * point;
      //   auto* point_msg = point_marker.add_points();
      //   point_msg->set_x(point_world.x());
      //   point_msg->set_y(point_world.y());
      //   point_msg->set_z(point_world.z());
      // }
      const auto& curve = occ_edge.vehicle_curve;
      for (auto pt_x = curve.min; pt_x <= curve.max; pt_x += 1.0) {
        double pt_y = 0.0;
        double val = 1.0;
        for (const auto& param : curve.coeffs) {
          pt_y += param * val;
          val *= pt_x;
        }
        Eigen::Vector3d point(pt_x, pt_y, 0.0);
        auto point_world = T_W_V * point;
        auto* point_msg = point_marker.add_points();
        point_msg->set_x(point_world.x());
        point_msg->set_y(point_world.y());
        point_msg->set_z(point_world.z());
      }
      markers.add_markers()->CopyFrom(point_marker);

      adsfi_proto::viz::Marker txt_marker;
      txt_marker.set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
      txt_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
      txt_marker.set_id(id++);
      txt_marker.mutable_lifetime()->set_sec(0);
      txt_marker.mutable_lifetime()->set_nsec(200000000);
      txt_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
      txt_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
      txt_marker.mutable_header()->set_frameid("localmap");
      txt_marker.mutable_pose()->mutable_position()->set_x(txt_point[0]);
      txt_marker.mutable_pose()->mutable_position()->set_y(txt_point[1]);
      txt_marker.mutable_pose()->mutable_position()->set_z(2);
      txt_marker.mutable_pose()->mutable_orientation()->set_x(0.);
      txt_marker.mutable_pose()->mutable_orientation()->set_y(0.);
      txt_marker.mutable_pose()->mutable_orientation()->set_z(0.);
      txt_marker.mutable_pose()->mutable_orientation()->set_w(1.);
      txt_marker.mutable_color()->set_a(1);
      txt_marker.mutable_color()->set_r(0.68);
      txt_marker.mutable_color()->set_g(0.85);
      txt_marker.mutable_color()->set_b(0.90);
      txt_marker.set_text(std::to_string(occ_edge.id) + ", " +
                          std::to_string(static_cast<int>(occ_edge.detect_id)));
      txt_marker.mutable_scale()->set_x(2);
      txt_marker.mutable_scale()->set_y(2);
      txt_marker.mutable_scale()->set_z(2);
      markers.add_markers()->CopyFrom(txt_marker);
    }
    util::RvizAgent::Instance().Publish(topic, markers);
  }

  static void PubMapRoadEdge(const Eigen::Affine3d& T_W_V,
                             const std::vector<RoadEdge>& map_road_edges,
                             const uint64_t& sec, const uint64_t& nsec,
                             const std::string& topic) {
    static bool register_flag = true;
    if (register_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::PointCloud>(topic);
      register_flag = false;
    }
    adsfi_proto::viz::PointCloud points_msg;
    points_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
    points_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    points_msg.mutable_header()->set_frameid("localmap");
    auto* channels = points_msg.add_channels();
    channels->set_name("rgb");
    for (const auto& road_edge : map_road_edges) {
      if (road_edge.state == TrackState::NOTMATURED) {
        continue;
      }
      for (const auto& point : road_edge.vehicle_points) {
        auto point_world = T_W_V * point;
        auto* point_msg = points_msg.add_points();
        point_msg->set_x(static_cast<float>(point_world.x()));
        point_msg->set_y(static_cast<float>(point_world.y()));
        point_msg->set_z(static_cast<float>(point_world.z()));
      }
    }
    util::RvizAgent::Instance().Publish(topic, points_msg);
  }

  static void PubMapRoadEdgeMarker(const Eigen::Affine3d& T_W_V,
                                   const std::vector<RoadEdge>& map_road_edges,
                                   const uint64_t& sec, const uint64_t& nsec,
                                   const std::string& topic) {
    static bool register_flag = true;
    if (register_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::MarkerArray>(
          topic);
      register_flag = false;
    }
    adsfi_proto::viz::MarkerArray markers;
    int id = 0;
    for (const auto& road_edge : map_road_edges) {
      if (road_edge.vehicle_points.empty()) {
        continue;
      }
      if (road_edge.state == TrackState::NOTMATURED) {
        continue;
      }
      double min_abs_x = FLT_MAX;
      double c0 = 0;
      for (const auto& point : road_edge.vehicle_points) {
        if (point.x() < min_abs_x) {
          c0 = point.y();
          min_abs_x = point.x();
        }
      }
      Eigen::Vector3d txt_point{road_edge.vehicle_points.back().x(),
                                road_edge.vehicle_points.back().y(), 0};
      txt_point = T_W_V * txt_point;
      adsfi_proto::viz::Marker point_marker;
      point_marker.mutable_header()->set_frameid("localmap");
      point_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
      point_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
      point_marker.set_id(id++);
      point_marker.set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
      point_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
      point_marker.mutable_pose()->mutable_orientation()->set_x(0.);
      point_marker.mutable_pose()->mutable_orientation()->set_y(0.);
      point_marker.mutable_pose()->mutable_orientation()->set_z(0.);
      point_marker.mutable_pose()->mutable_orientation()->set_w(1.);
      point_marker.mutable_scale()->set_x(0.2);
      point_marker.mutable_scale()->set_y(0.2);
      point_marker.mutable_scale()->set_z(0.2);
      point_marker.mutable_lifetime()->set_sec(0);
      point_marker.mutable_lifetime()->set_nsec(200000000);
      point_marker.mutable_color()->set_a(1.0);
      point_marker.mutable_color()->set_r(0.53);
      point_marker.mutable_color()->set_g(0.54);
      point_marker.mutable_color()->set_b(0.52);
      for (const auto& point : road_edge.vehicle_points) {
        auto point_world = T_W_V * point;
        auto* point_msg = point_marker.add_points();
        point_msg->set_x(point_world.x());
        point_msg->set_y(point_world.y());
        point_msg->set_z(point_world.z());
      }
      markers.add_markers()->CopyFrom(point_marker);

      adsfi_proto::viz::Marker txt_marker;
      txt_marker.set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
      txt_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
      txt_marker.set_id(id++);
      txt_marker.mutable_lifetime()->set_sec(0);
      txt_marker.mutable_lifetime()->set_nsec(200000000);
      txt_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
      txt_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
      txt_marker.mutable_header()->set_frameid("localmap");
      txt_marker.mutable_pose()->mutable_position()->set_x(txt_point[0]);
      txt_marker.mutable_pose()->mutable_position()->set_y(txt_point[1]);
      txt_marker.mutable_pose()->mutable_position()->set_z(2);
      txt_marker.mutable_pose()->mutable_orientation()->set_x(0.);
      txt_marker.mutable_pose()->mutable_orientation()->set_y(0.);
      txt_marker.mutable_pose()->mutable_orientation()->set_z(0.);
      txt_marker.mutable_pose()->mutable_orientation()->set_w(1.);
      txt_marker.mutable_color()->set_a(1);
      txt_marker.mutable_color()->set_r(0.53);
      txt_marker.mutable_color()->set_g(0.54);
      txt_marker.mutable_color()->set_b(0.52);
      txt_marker.set_text(std::to_string(road_edge.id) + ", " +
                          std::to_string(static_cast<int>(road_edge.position)) +
                          ", " +
                          std::to_string(static_cast<int>(road_edge.type)));
      txt_marker.mutable_scale()->set_x(2);
      txt_marker.mutable_scale()->set_y(2);
      txt_marker.mutable_scale()->set_z(2);
      markers.add_markers()->CopyFrom(txt_marker);
    }
    util::RvizAgent::Instance().Publish(topic, markers);
  }

  static void PubImmatureMapRoadEdge(
      const Eigen::Affine3d& T_W_V, const std::vector<RoadEdge>& map_road_edges,
      const uint64_t& sec, const uint64_t& nsec, const std::string& topic) {
    static bool register_flag = true;
    if (register_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::PointCloud>(topic);
      register_flag = false;
    }
    adsfi_proto::viz::PointCloud points_msg;
    points_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
    points_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    points_msg.mutable_header()->set_frameid("localmap");
    auto* channels = points_msg.add_channels();
    channels->set_name("rgb");
    for (const auto& road_edge : map_road_edges) {
      if (road_edge.state == TrackState::MATURED) {
        continue;
      }
      for (const auto& point : road_edge.vehicle_points) {
        auto point_world = T_W_V * point;
        auto* point_msg = points_msg.add_points();
        point_msg->set_x(static_cast<float>(point_world.x()));
        point_msg->set_y(static_cast<float>(point_world.y()));
        point_msg->set_z(static_cast<float>(point_world.z()));
      }
    }
    util::RvizAgent::Instance().Publish(topic, points_msg);
  }

  static void PubImmatureMapRoadEdgeMarker(
      const Eigen::Affine3d& T_W_V, const std::vector<RoadEdge>& map_road_edges,
      const uint64_t& sec, const uint64_t& nsec, const std::string& topic) {
    static bool register_flag = true;
    if (register_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::MarkerArray>(
          topic);
      register_flag = false;
    }
    adsfi_proto::viz::MarkerArray markers;
    int id = 0;
    for (const auto& road_edge : map_road_edges) {
      if (road_edge.vehicle_points.empty()) {
        continue;
      }
      if (road_edge.state == TrackState::MATURED) {
        continue;
      }
      double min_abs_x = FLT_MAX;
      double c0 = 0.0;
      for (const auto& point : road_edge.vehicle_points) {
        if (point.x() < min_abs_x) {
          c0 = point.y();
          min_abs_x = point.x();
        }
      }
      Eigen::Vector3d txt_point{road_edge.vehicle_points.back().x(),
                                road_edge.vehicle_points.back().y(), 0};
      txt_point = T_W_V * txt_point;
      adsfi_proto::viz::Marker point_marker;
      point_marker.mutable_header()->set_frameid("localmap");
      point_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
      point_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
      point_marker.set_id(id++);
      point_marker.set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
      point_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
      point_marker.mutable_pose()->mutable_orientation()->set_x(0.);
      point_marker.mutable_pose()->mutable_orientation()->set_y(0.);
      point_marker.mutable_pose()->mutable_orientation()->set_z(0.);
      point_marker.mutable_pose()->mutable_orientation()->set_w(1.);
      point_marker.mutable_scale()->set_x(0.2);
      point_marker.mutable_scale()->set_y(0.2);
      point_marker.mutable_scale()->set_z(0.2);
      point_marker.mutable_lifetime()->set_sec(0);
      point_marker.mutable_lifetime()->set_nsec(200000000);
      point_marker.mutable_color()->set_a(1.0);
      point_marker.mutable_color()->set_r(0.68);
      point_marker.mutable_color()->set_g(0.50);
      point_marker.mutable_color()->set_b(0.66);
      for (const auto& point : road_edge.vehicle_points) {
        auto point_world = T_W_V * point;
        auto* point_msg = point_marker.add_points();
        point_msg->set_x(point_world.x());
        point_msg->set_y(point_world.y());
        point_msg->set_z(point_world.z());
      }
      markers.add_markers()->CopyFrom(point_marker);

      adsfi_proto::viz::Marker txt_marker;
      txt_marker.set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
      txt_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
      txt_marker.set_id(id++);
      txt_marker.mutable_lifetime()->set_sec(0);
      txt_marker.mutable_lifetime()->set_nsec(200000000);
      txt_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
      txt_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
      txt_marker.mutable_header()->set_frameid("localmap");
      txt_marker.mutable_pose()->mutable_position()->set_x(txt_point[0]);
      txt_marker.mutable_pose()->mutable_position()->set_y(txt_point[1]);
      txt_marker.mutable_pose()->mutable_position()->set_z(2);
      txt_marker.mutable_pose()->mutable_orientation()->set_x(0.);
      txt_marker.mutable_pose()->mutable_orientation()->set_y(0.);
      txt_marker.mutable_pose()->mutable_orientation()->set_z(0.);
      txt_marker.mutable_pose()->mutable_orientation()->set_w(1.);
      txt_marker.mutable_color()->set_a(1);
      txt_marker.mutable_color()->set_r(0.53);
      txt_marker.mutable_color()->set_g(0.54);
      txt_marker.mutable_color()->set_b(0.52);
      txt_marker.set_text(std::to_string(road_edge.tracked_count));
      txt_marker.mutable_scale()->set_x(2);
      txt_marker.mutable_scale()->set_y(2);
      txt_marker.mutable_scale()->set_z(2);
      markers.add_markers()->CopyFrom(txt_marker);
    }
    util::RvizAgent::Instance().Publish(topic, markers);
  }

  static void PubMapStopLine(const Eigen::Affine3d& T_W_V,
                             const std::vector<StopLine>& map_stop_lines,
                             const uint64_t& sec, const uint64_t& nsec,
                             const std::string& topic) {
    static bool register_flag = true;
    if (register_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::MarkerArray>(
          topic);
      register_flag = false;
    }
    adsfi_proto::viz::MarkerArray markers;
    int id = 0;
    for (const auto& stop_line : map_stop_lines) {
      if (stop_line.state == TrackState::NOTMATURED) {
        continue;
      }
      adsfi_proto::viz::Marker marker;
      marker.mutable_header()->set_frameid("localmap");
      marker.mutable_header()->mutable_timestamp()->set_sec(sec);
      marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
      marker.set_id(id++);
      marker.set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
      marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
      marker.mutable_pose()->mutable_position()->set_x(0);
      marker.mutable_pose()->mutable_position()->set_y(0);
      marker.mutable_pose()->mutable_position()->set_z(0);
      marker.mutable_pose()->mutable_orientation()->set_x(0.);
      marker.mutable_pose()->mutable_orientation()->set_y(0.);
      marker.mutable_pose()->mutable_orientation()->set_z(0.);
      marker.mutable_pose()->mutable_orientation()->set_w(1.);
      marker.mutable_scale()->set_x(0.2);
      marker.mutable_scale()->set_y(0.2);
      marker.mutable_scale()->set_z(0.2);
      marker.mutable_lifetime()->set_sec(0);
      marker.mutable_lifetime()->set_nsec(200000000);
      adsfi_proto::viz::ColorRGBA color;
      color.set_a(1.0);
      color.set_r(1.0);
      color.set_g(1.0);
      color.set_b(1.0);
      marker.mutable_color()->CopyFrom(color);
      auto point = T_W_V * stop_line.left_point;
      auto* left_point = marker.add_points();
      left_point->set_x(static_cast<float>(point.x()));
      left_point->set_y(static_cast<float>(point.y()));
      left_point->set_z(static_cast<float>(point.z()));
      point = T_W_V * stop_line.right_point;
      auto* right_point = marker.add_points();
      right_point->set_x(static_cast<float>(point.x()));
      right_point->set_y(static_cast<float>(point.y()));
      right_point->set_z(static_cast<float>(point.z()));
      markers.add_markers()->CopyFrom(marker);
    }
    util::RvizAgent::Instance().Publish(topic, markers);
  }

  static void PubImmatureMapStopLine(
      const Eigen::Affine3d& T_W_V, const std::vector<StopLine>& map_stop_lines,
      const uint64_t& sec, const uint64_t& nsec, const std::string& topic) {
    static bool register_flag = true;
    if (register_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::MarkerArray>(
          topic);
      register_flag = false;
    }
    adsfi_proto::viz::MarkerArray markers;
    int id = 0;
    for (const auto& stop_line : map_stop_lines) {
      if (stop_line.state == TrackState::MATURED) {
        continue;
      }
      adsfi_proto::viz::Marker marker;
      marker.mutable_header()->set_frameid("localmap");
      marker.mutable_header()->mutable_timestamp()->set_sec(sec);
      marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
      marker.set_id(id++);
      marker.set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
      marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
      marker.mutable_pose()->mutable_position()->set_x(0);
      marker.mutable_pose()->mutable_position()->set_y(0);
      marker.mutable_pose()->mutable_position()->set_z(0);
      marker.mutable_pose()->mutable_orientation()->set_x(0.);
      marker.mutable_pose()->mutable_orientation()->set_y(0.);
      marker.mutable_pose()->mutable_orientation()->set_z(0.);
      marker.mutable_pose()->mutable_orientation()->set_w(1.);
      marker.mutable_scale()->set_x(0.2);
      marker.mutable_scale()->set_y(0.2);
      marker.mutable_scale()->set_z(0.2);
      marker.mutable_lifetime()->set_sec(0);
      marker.mutable_lifetime()->set_nsec(200000000);
      adsfi_proto::viz::ColorRGBA color;
      color.set_r(0.68);
      color.set_g(0.50);
      color.set_b(0.66);
      color.set_a(1);
      marker.mutable_color()->CopyFrom(color);
      auto point = T_W_V * stop_line.left_point;
      auto* left_point = marker.add_points();
      left_point->set_x(static_cast<float>(point.x()));
      left_point->set_y(static_cast<float>(point.y()));
      left_point->set_z(static_cast<float>(point.z()));
      point = T_W_V * stop_line.right_point;
      auto* right_point = marker.add_points();
      right_point->set_x(static_cast<float>(point.x()));
      right_point->set_y(static_cast<float>(point.y()));
      right_point->set_z(static_cast<float>(point.z()));
      markers.add_markers()->CopyFrom(marker);
    }
    util::RvizAgent::Instance().Publish(topic, markers);
  }

  static void PubImmatureMapStopLineMarker(
      const Eigen::Affine3d& T_W_V, const std::vector<StopLine>& map_stop_lines,
      const uint64_t& sec, const uint64_t& nsec, const std::string& topic) {
    static bool register_flag = true;
    if (register_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::MarkerArray>(
          topic);
      register_flag = false;
    }
    adsfi_proto::viz::MarkerArray markers;
    int id = 0;
    for (const auto& stop_line : map_stop_lines) {
      if (stop_line.state == TrackState::MATURED) {
        continue;
      }
      adsfi_proto::viz::Marker txt_marker;
      txt_marker.set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
      txt_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
      txt_marker.set_id(id++);
      txt_marker.mutable_lifetime()->set_sec(0);
      txt_marker.mutable_lifetime()->set_nsec(200000000);
      txt_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
      txt_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
      txt_marker.mutable_header()->set_frameid("localmap");
      Eigen::Vector3d point = {stop_line.center_point.x(),
                               stop_line.center_point.y(), 0};
      point = T_W_V * point;
      txt_marker.mutable_pose()->mutable_position()->set_x(point.x());
      txt_marker.mutable_pose()->mutable_position()->set_y(point.y());
      txt_marker.mutable_pose()->mutable_position()->set_z(2);
      txt_marker.mutable_pose()->mutable_orientation()->set_x(0);
      txt_marker.mutable_pose()->mutable_orientation()->set_y(0);
      txt_marker.mutable_pose()->mutable_orientation()->set_z(0);
      txt_marker.mutable_pose()->mutable_orientation()->set_w(1);
      txt_marker.mutable_color()->set_r(0.68);
      txt_marker.mutable_color()->set_g(0.50);
      txt_marker.mutable_color()->set_b(0.66);
      txt_marker.mutable_color()->set_a(1);
      txt_marker.set_text(std::to_string(stop_line.tracked_count) + ", " +
                          std::to_string(stop_line.id));
      txt_marker.mutable_scale()->set_x(2);
      txt_marker.mutable_scale()->set_y(2);
      txt_marker.mutable_scale()->set_z(2);
      markers.add_markers()->CopyFrom(txt_marker);
    }
    util::RvizAgent::Instance().Publish(topic, markers);
  }

  static void PubMapArrow(const Eigen::Affine3d& T_W_V,
                          const std::vector<Arrow>& map_arrows,
                          const uint64_t& sec, const uint64_t& nsec,
                          const std::string& topic) {
    static bool register_flag = true;
    if (register_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::MarkerArray>(
          topic);
      register_flag = false;
    }
    adsfi_proto::viz::MarkerArray markers;
    int id = 0;
    for (const auto& arrow : map_arrows) {
      if (arrow.state == TrackState::NOTMATURED) {
        continue;
      }
      adsfi_proto::viz::Marker point_marker;
      point_marker.mutable_header()->set_frameid("localmap");
      point_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
      point_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
      point_marker.set_id(id++);
      point_marker.set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
      point_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
      point_marker.mutable_pose()->mutable_position()->set_x(0);
      point_marker.mutable_pose()->mutable_position()->set_y(0);
      point_marker.mutable_pose()->mutable_position()->set_z(0);
      point_marker.mutable_pose()->mutable_orientation()->set_x(0.);
      point_marker.mutable_pose()->mutable_orientation()->set_y(0.);
      point_marker.mutable_pose()->mutable_orientation()->set_z(0.);
      point_marker.mutable_pose()->mutable_orientation()->set_w(1.);
      point_marker.mutable_scale()->set_x(0.2);
      point_marker.mutable_scale()->set_y(0.2);
      point_marker.mutable_scale()->set_z(0.2);
      point_marker.mutable_lifetime()->set_sec(0);
      point_marker.mutable_lifetime()->set_nsec(200000000);
      adsfi_proto::viz::ColorRGBA color;
      color.set_a(1.0);
      color.set_r(1.0);
      color.set_g(1.0);
      color.set_b(1.0);
      point_marker.mutable_color()->CopyFrom(color);

      Eigen::Vector3d point_0 = arrow.vehicle_points[0];
      point_0 = T_W_V * point_0;
      Eigen::Vector3d point_1 = arrow.vehicle_points[1];
      point_1 = T_W_V * point_1;
      Eigen::Vector3d point_2 = arrow.vehicle_points[2];
      point_2 = T_W_V * point_2;
      Eigen::Vector3d point_3 = arrow.vehicle_points[3];
      point_3 = T_W_V * point_3;
      auto* point_msg = point_marker.add_points();
      point_msg->set_x(point_0.x());
      point_msg->set_y(point_0.y());
      point_msg = point_marker.add_points();
      point_msg->set_x(point_1.x());
      point_msg->set_y(point_1.y());
      point_msg = point_marker.add_points();
      point_msg->set_x(point_2.x());
      point_msg->set_y(point_2.y());
      point_msg = point_marker.add_points();
      point_msg->set_x(point_3.x());
      point_msg->set_y(point_3.y());
      point_msg = point_marker.add_points();
      point_msg->set_x(point_0.x());
      point_msg->set_y(point_0.y());
      markers.add_markers()->CopyFrom(point_marker);
      adsfi_proto::viz::Marker txt_marker;
      txt_marker.set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
      txt_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
      txt_marker.set_id(id++);
      txt_marker.mutable_lifetime()->set_sec(0);
      txt_marker.mutable_lifetime()->set_nsec(200000000);
      txt_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
      txt_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
      txt_marker.mutable_header()->set_frameid("localmap");
      txt_marker.mutable_pose()->mutable_position()->set_x(
          (point_0.x() + point_2.x()) / 2);
      txt_marker.mutable_pose()->mutable_position()->set_y(
          (point_0.y() + point_2.y()) / 2);
      txt_marker.mutable_pose()->mutable_orientation()->set_x(0);
      txt_marker.mutable_pose()->mutable_orientation()->set_y(0);
      txt_marker.mutable_pose()->mutable_orientation()->set_z(0);
      txt_marker.mutable_pose()->mutable_orientation()->set_w(1);
      txt_marker.mutable_color()->set_a(1);
      txt_marker.mutable_color()->set_r(1);
      txt_marker.mutable_color()->set_g(1);
      txt_marker.mutable_color()->set_b(1);
      std::map<ArrowType, std::string> ArrowTypeToString{
          {ArrowType::UNKNOWN, "ARROWTYPE_UNKNOWN"},
          {ArrowType::STRAIGHT_FORWARD, "STRAIGHT_FORWARD"},
          {ArrowType::STRAIGHT_FORWARD_OR_TURN_LEFT,
           "STRAIGHT_FORWARD_OR_TURN_LEFT"},
          {ArrowType::STRAIGHT_FORWARD_OR_TURN_RIGHT,
           "STRAIGHT_FORWARD_OR_TURN_RIGHT"},
          {ArrowType::STRAIGHT_FORWARD_OR_TURN_LEFT_OR_TURN_RIGHT,
           "STRAIGHT_FORWARD_OR_TURN_LEFT_OR_TURN_RIGHT"},
          {ArrowType::STRAIGHT_FORWARD_OR_TURN_AROUND,
           "STRAIGHT_FORWARD_OR_TURN_AROUND"},
          {ArrowType::STRAIGHT_FORWARD_OR_TURN_AROUND_OR_TURN_LEFT,
           "STRAIGHT_FORWARD_OR_TURN_AROUND_OR_TURN_LEFT"},
          {ArrowType::TURN_LEFT, "TURN_LEFT"},
          {ArrowType::TURN_LEFT_OR_MERGE_LEFT, "TURN_LEFT_OR_MERGE_LEFT"},
          {ArrowType::TURN_LEFT_OR_TURN_AROUND, "TURN_LEFT_OR_TURN_AROUND"},
          {ArrowType::TURN_LEFT_OR_TURN_RIGHT, "TURN_LEFT_OR_TURN_RIGHT"},
          {ArrowType::TURN_RIGHT, "TURN_RIGHT"},
          {ArrowType::TURN_RIGHT_OR_MERGE_RIGHT, "TURN_RIGHT_OR_MERGE_RIGHT"},
          {ArrowType::TURN_RIGHT_OR_TURN_AROUND, "TURN_RIGHT_OR_TURN_AROUND"},
          {ArrowType::TURN_AROUND, "TURN_AROUND"},
          {ArrowType::FORBID_TURN_LEFT, "FORBID_TURN_LEFT"},
          {ArrowType::FORBID_TURN_RIGHT, "FORBID_TURN_RIGHT"},
          {ArrowType::FORBID_TURN_AROUND, "FORBID_TURN_AROUND"},
          {ArrowType::FRONT_NEAR_CROSSWALK, "FRONT_NEAR_CROSSWALK"},
      };
      txt_marker.set_text(ArrowTypeToString[arrow.type]);
      txt_marker.mutable_scale()->set_x(1);
      txt_marker.mutable_scale()->set_y(1);
      txt_marker.mutable_scale()->set_z(0.5);
      markers.add_markers()->CopyFrom(txt_marker);
    }
    util::RvizAgent::Instance().Publish(topic, markers);
  }

  static void PubImmatureMapArrow(const Eigen::Affine3d& T_W_V,
                                  const std::vector<Arrow>& map_arrows,
                                  const uint64_t& sec, const uint64_t& nsec,
                                  const std::string& topic) {
    static bool register_flag = true;
    if (register_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::MarkerArray>(
          topic);
      register_flag = false;
    }
    adsfi_proto::viz::MarkerArray markers;
    int id = 0;
    for (const auto& arrow : map_arrows) {
      if (arrow.state == TrackState::MATURED) {
        continue;
      }
      adsfi_proto::viz::Marker point_marker;
      point_marker.mutable_header()->set_frameid("localmap");
      point_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
      point_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
      point_marker.set_id(id++);
      point_marker.set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
      point_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
      point_marker.mutable_pose()->mutable_position()->set_x(0);
      point_marker.mutable_pose()->mutable_position()->set_y(0);
      point_marker.mutable_pose()->mutable_position()->set_z(0);
      point_marker.mutable_pose()->mutable_orientation()->set_x(0.);
      point_marker.mutable_pose()->mutable_orientation()->set_y(0.);
      point_marker.mutable_pose()->mutable_orientation()->set_z(0.);
      point_marker.mutable_pose()->mutable_orientation()->set_w(1.);
      point_marker.mutable_scale()->set_x(0.2);
      point_marker.mutable_scale()->set_y(0.2);
      point_marker.mutable_scale()->set_z(0.2);
      point_marker.mutable_lifetime()->set_sec(0);
      point_marker.mutable_lifetime()->set_nsec(200000000);
      adsfi_proto::viz::ColorRGBA color;
      color.set_r(0.68);
      color.set_g(0.50);
      color.set_b(0.66);
      color.set_a(1);
      point_marker.mutable_color()->CopyFrom(color);
      Eigen::Vector3d point_0 = arrow.vehicle_points[0];
      point_0 = T_W_V * point_0;
      Eigen::Vector3d point_1 = arrow.vehicle_points[1];
      point_1 = T_W_V * point_1;
      Eigen::Vector3d point_2 = arrow.vehicle_points[2];
      point_2 = T_W_V * point_2;
      Eigen::Vector3d point_3 = arrow.vehicle_points[3];
      point_3 = T_W_V * point_3;
      auto* point_msg = point_marker.add_points();
      point_msg->set_x(point_0.x());
      point_msg->set_y(point_0.y());
      point_msg = point_marker.add_points();
      point_msg->set_x(point_1.x());
      point_msg->set_y(point_1.y());
      point_msg = point_marker.add_points();
      point_msg->set_x(point_2.x());
      point_msg->set_y(point_2.y());
      point_msg = point_marker.add_points();
      point_msg->set_x(point_3.x());
      point_msg->set_y(point_3.y());
      point_msg = point_marker.add_points();
      point_msg->set_x(point_0.x());
      point_msg->set_y(point_0.y());
      markers.add_markers()->CopyFrom(point_marker);
      adsfi_proto::viz::Marker txt_marker;
      txt_marker.set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
      txt_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
      txt_marker.set_id(id++);
      txt_marker.mutable_lifetime()->set_sec(0);
      txt_marker.mutable_lifetime()->set_nsec(200000000);
      txt_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
      txt_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
      txt_marker.mutable_header()->set_frameid("localmap");
      txt_marker.mutable_pose()->mutable_position()->set_x(
          (point_0.x() + point_2.x()) / 2);
      txt_marker.mutable_pose()->mutable_position()->set_y(
          (point_0.y() + point_2.y()) / 2);
      txt_marker.mutable_pose()->mutable_orientation()->set_x(0);
      txt_marker.mutable_pose()->mutable_orientation()->set_y(0);
      txt_marker.mutable_pose()->mutable_orientation()->set_z(0);
      txt_marker.mutable_pose()->mutable_orientation()->set_w(1);
      txt_marker.mutable_color()->set_a(1);
      txt_marker.mutable_color()->set_r(1);
      txt_marker.mutable_color()->set_g(1);
      txt_marker.mutable_color()->set_b(1);
      std::map<ArrowType, std::string> ArrowTypeToString{
          {ArrowType::UNKNOWN, "ARROWTYPE_UNKNOWN"},
          {ArrowType::STRAIGHT_FORWARD, "STRAIGHT_FORWARD"},
          {ArrowType::STRAIGHT_FORWARD_OR_TURN_LEFT,
           "STRAIGHT_FORWARD_OR_TURN_LEFT"},
          {ArrowType::STRAIGHT_FORWARD_OR_TURN_RIGHT,
           "STRAIGHT_FORWARD_OR_TURN_RIGHT"},
          {ArrowType::STRAIGHT_FORWARD_OR_TURN_LEFT_OR_TURN_RIGHT,
           "STRAIGHT_FORWARD_OR_TURN_LEFT_OR_TURN_RIGHT"},
          {ArrowType::STRAIGHT_FORWARD_OR_TURN_AROUND,
           "STRAIGHT_FORWARD_OR_TURN_AROUND"},
          {ArrowType::STRAIGHT_FORWARD_OR_TURN_AROUND_OR_TURN_LEFT,
           "STRAIGHT_FORWARD_OR_TURN_AROUND_OR_TURN_LEFT"},
          {ArrowType::TURN_LEFT, "TURN_LEFT"},
          {ArrowType::TURN_LEFT_OR_MERGE_LEFT, "TURN_LEFT_OR_MERGE_LEFT"},
          {ArrowType::TURN_LEFT_OR_TURN_AROUND, "TURN_LEFT_OR_TURN_AROUND"},
          {ArrowType::TURN_LEFT_OR_TURN_RIGHT, "TURN_LEFT_OR_TURN_RIGHT"},
          {ArrowType::TURN_RIGHT, "TURN_RIGHT"},
          {ArrowType::TURN_RIGHT_OR_MERGE_RIGHT, "TURN_RIGHT_OR_MERGE_RIGHT"},
          {ArrowType::TURN_RIGHT_OR_TURN_AROUND, "TURN_RIGHT_OR_TURN_AROUND"},
          {ArrowType::TURN_AROUND, "TURN_AROUND"},
          {ArrowType::FORBID_TURN_LEFT, "FORBID_TURN_LEFT"},
          {ArrowType::FORBID_TURN_RIGHT, "FORBID_TURN_RIGHT"},
          {ArrowType::FORBID_TURN_AROUND, "FORBID_TURN_AROUND"},
          {ArrowType::FRONT_NEAR_CROSSWALK, "FRONT_NEAR_CROSSWALK"},
      };
      txt_marker.set_text(ArrowTypeToString[arrow.type]);
      txt_marker.mutable_scale()->set_x(1);
      txt_marker.mutable_scale()->set_y(1);
      txt_marker.mutable_scale()->set_z(0.5);
      markers.add_markers()->CopyFrom(txt_marker);
    }
    util::RvizAgent::Instance().Publish(topic, markers);
  }

  static void PubImmatureMapArrowMarker(const Eigen::Affine3d& T_W_V,
                                        const std::vector<Arrow>& map_arrows,
                                        const uint64_t& sec,
                                        const uint64_t& nsec,
                                        const std::string& topic) {
    static bool register_flag = true;
    if (register_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::MarkerArray>(
          topic);
      register_flag = false;
    }
    adsfi_proto::viz::MarkerArray markers;
    int id = 0;
    for (const auto& arrow : map_arrows) {
      if (arrow.state == TrackState::MATURED) {
        continue;
      }
      adsfi_proto::viz::Marker txt_marker;
      txt_marker.set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
      txt_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
      txt_marker.set_id(id++);
      txt_marker.mutable_lifetime()->set_sec(0);
      txt_marker.mutable_lifetime()->set_nsec(200000000);
      txt_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
      txt_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
      txt_marker.mutable_header()->set_frameid("localmap");
      Eigen::Vector3d point = {arrow.center_point.x(), arrow.center_point.y(),
                               0};
      point = T_W_V * point;
      txt_marker.mutable_pose()->mutable_position()->set_x(point.x());
      txt_marker.mutable_pose()->mutable_position()->set_y(point.y());
      txt_marker.mutable_pose()->mutable_position()->set_z(2);
      txt_marker.mutable_pose()->mutable_orientation()->set_x(0);
      txt_marker.mutable_pose()->mutable_orientation()->set_y(0);
      txt_marker.mutable_pose()->mutable_orientation()->set_z(0);
      txt_marker.mutable_pose()->mutable_orientation()->set_w(1);
      txt_marker.mutable_color()->set_r(0.68);
      txt_marker.mutable_color()->set_g(0.50);
      txt_marker.mutable_color()->set_b(0.66);
      txt_marker.mutable_color()->set_a(1);
      txt_marker.set_text(std::to_string(arrow.tracked_count));
      txt_marker.mutable_scale()->set_x(2);
      txt_marker.mutable_scale()->set_y(2);
      txt_marker.mutable_scale()->set_z(2);
      markers.add_markers()->CopyFrom(txt_marker);
    }
    util::RvizAgent::Instance().Publish(topic, markers);
  }

  static void PubMapZebraCrossing(
      const Eigen::Affine3d& T_W_V,
      const std::vector<ZebraCrossing>& map_zebra_crossings,
      const uint64_t& sec, const uint64_t& nsec, const std::string& topic) {
    static bool register_flag = true;
    if (register_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::MarkerArray>(
          topic);
      register_flag = false;
    }
    adsfi_proto::viz::MarkerArray markers;
    int id = 0;
    for (const auto& zebra_crossing : map_zebra_crossings) {
      if (zebra_crossing.state == TrackState::NOTMATURED) {
        continue;
      }
      adsfi_proto::viz::Marker marker;
      marker.mutable_header()->set_frameid("localmap");
      marker.mutable_header()->mutable_timestamp()->set_sec(sec);
      marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
      marker.set_id(id++);
      marker.set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
      marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
      marker.mutable_pose()->mutable_position()->set_x(0);
      marker.mutable_pose()->mutable_position()->set_y(0);
      marker.mutable_pose()->mutable_position()->set_z(0);
      marker.mutable_pose()->mutable_orientation()->set_x(0.);
      marker.mutable_pose()->mutable_orientation()->set_y(0.);
      marker.mutable_pose()->mutable_orientation()->set_z(0.);
      marker.mutable_pose()->mutable_orientation()->set_w(1.);
      marker.mutable_scale()->set_x(0.2);
      marker.mutable_scale()->set_y(0.2);
      marker.mutable_scale()->set_z(0.2);
      marker.mutable_lifetime()->set_sec(0);
      marker.mutable_lifetime()->set_nsec(200000000);
      adsfi_proto::viz::ColorRGBA color;
      color.set_a(1.0);
      color.set_r(1.0);
      color.set_g(1.0);
      color.set_b(1.0);
      marker.mutable_color()->CopyFrom(color);
      Eigen::Vector3d point_0 = zebra_crossing.vehicle_points[0];
      point_0 = T_W_V * point_0;
      Eigen::Vector3d point_1 = zebra_crossing.vehicle_points[1];
      point_1 = T_W_V * point_1;
      Eigen::Vector3d point_2 = zebra_crossing.vehicle_points[2];
      point_2 = T_W_V * point_2;
      Eigen::Vector3d point_3 = zebra_crossing.vehicle_points[3];
      point_3 = T_W_V * point_3;
      auto* point_msg = marker.add_points();
      point_msg->set_x(point_0.x());
      point_msg->set_y(point_0.y());
      point_msg = marker.add_points();
      point_msg->set_x(point_1.x());
      point_msg->set_y(point_1.y());
      point_msg = marker.add_points();
      point_msg->set_x(point_2.x());
      point_msg->set_y(point_2.y());
      point_msg = marker.add_points();
      point_msg->set_x(point_3.x());
      point_msg->set_y(point_3.y());
      point_msg = marker.add_points();
      point_msg->set_x(point_0.x());
      point_msg->set_y(point_0.y());
      markers.add_markers()->CopyFrom(marker);
      adsfi_proto::viz::Marker txt_marker;
      txt_marker.set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
      txt_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
      txt_marker.set_id(id++);
      txt_marker.mutable_lifetime()->set_sec(0);
      txt_marker.mutable_lifetime()->set_nsec(200000000);
      txt_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
      txt_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
      txt_marker.mutable_header()->set_frameid("localmap");
      txt_marker.mutable_pose()->mutable_position()->set_x(
          (point_0.x() + point_2.x()) / 2);
      txt_marker.mutable_pose()->mutable_position()->set_y(
          (point_0.y() + point_2.y()) / 2);
      txt_marker.mutable_pose()->mutable_orientation()->set_x(0);
      txt_marker.mutable_pose()->mutable_orientation()->set_y(0);
      txt_marker.mutable_pose()->mutable_orientation()->set_z(0);
      txt_marker.mutable_pose()->mutable_orientation()->set_w(1);
      txt_marker.mutable_color()->set_a(1);
      txt_marker.mutable_color()->set_r(1);
      txt_marker.mutable_color()->set_g(1);
      txt_marker.mutable_color()->set_b(1);
      txt_marker.set_text("zebra_crossing, " +
                          std::to_string(zebra_crossing.id));
      txt_marker.mutable_scale()->set_x(1);
      txt_marker.mutable_scale()->set_y(1);
      txt_marker.mutable_scale()->set_z(0.5);
      markers.add_markers()->CopyFrom(txt_marker);
    }
    util::RvizAgent::Instance().Publish(topic, markers);
  }

  static void PubImmatureMapZebraCrossing(
      const Eigen::Affine3d& T_W_V,
      const std::vector<ZebraCrossing>& map_zebra_crossings,
      const uint64_t& sec, const uint64_t& nsec, const std::string& topic) {
    static bool register_flag = true;
    if (register_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::MarkerArray>(
          topic);
      register_flag = false;
    }
    adsfi_proto::viz::MarkerArray markers;
    int id = 0;
    for (const auto& zebra_crossing : map_zebra_crossings) {
      if (zebra_crossing.state == TrackState::MATURED) {
        continue;
      }
      adsfi_proto::viz::Marker marker;
      marker.mutable_header()->set_frameid("localmap");
      marker.mutable_header()->mutable_timestamp()->set_sec(sec);
      marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
      marker.set_id(id++);
      marker.set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
      marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
      marker.mutable_pose()->mutable_position()->set_x(0);
      marker.mutable_pose()->mutable_position()->set_y(0);
      marker.mutable_pose()->mutable_position()->set_z(0);
      marker.mutable_pose()->mutable_orientation()->set_x(0.);
      marker.mutable_pose()->mutable_orientation()->set_y(0.);
      marker.mutable_pose()->mutable_orientation()->set_z(0.);
      marker.mutable_pose()->mutable_orientation()->set_w(1.);
      marker.mutable_scale()->set_x(0.2);
      marker.mutable_scale()->set_y(0.2);
      marker.mutable_scale()->set_z(0.2);
      marker.mutable_lifetime()->set_sec(0);
      marker.mutable_lifetime()->set_nsec(200000000);
      adsfi_proto::viz::ColorRGBA color;
      color.set_r(0.68);
      color.set_g(0.50);
      color.set_b(0.66);
      color.set_a(1);
      marker.mutable_color()->CopyFrom(color);
      Eigen::Vector3d point_0 = zebra_crossing.vehicle_points[0];
      point_0 = T_W_V * point_0;
      Eigen::Vector3d point_1 = zebra_crossing.vehicle_points[1];
      point_1 = T_W_V * point_1;
      Eigen::Vector3d point_2 = zebra_crossing.vehicle_points[2];
      point_2 = T_W_V * point_2;
      Eigen::Vector3d point_3 = zebra_crossing.vehicle_points[3];
      point_3 = T_W_V * point_3;
      auto* point_msg = marker.add_points();
      point_msg->set_x(point_0.x());
      point_msg->set_y(point_0.y());
      point_msg = marker.add_points();
      point_msg->set_x(point_1.x());
      point_msg->set_y(point_1.y());
      point_msg = marker.add_points();
      point_msg->set_x(point_2.x());
      point_msg->set_y(point_2.y());
      point_msg = marker.add_points();
      point_msg->set_x(point_3.x());
      point_msg->set_y(point_3.y());
      point_msg = marker.add_points();
      point_msg->set_x(point_0.x());
      point_msg->set_y(point_0.y());
      markers.add_markers()->CopyFrom(marker);
      adsfi_proto::viz::Marker txt_marker;
      txt_marker.set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
      txt_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
      txt_marker.set_id(id++);
      txt_marker.mutable_lifetime()->set_sec(0);
      txt_marker.mutable_lifetime()->set_nsec(200000000);
      txt_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
      txt_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
      txt_marker.mutable_header()->set_frameid("localmap");
      txt_marker.mutable_pose()->mutable_position()->set_x(
          (point_0.x() + point_2.x()) / 2);
      txt_marker.mutable_pose()->mutable_position()->set_y(
          (point_0.y() + point_2.y()) / 2);
      txt_marker.mutable_pose()->mutable_orientation()->set_x(0);
      txt_marker.mutable_pose()->mutable_orientation()->set_y(0);
      txt_marker.mutable_pose()->mutable_orientation()->set_z(0);
      txt_marker.mutable_pose()->mutable_orientation()->set_w(1);
      txt_marker.mutable_color()->set_a(1);
      txt_marker.mutable_color()->set_r(1);
      txt_marker.mutable_color()->set_g(1);
      txt_marker.mutable_color()->set_b(1);
      txt_marker.set_text("zebra_crossing");
      txt_marker.mutable_scale()->set_x(1);
      txt_marker.mutable_scale()->set_y(1);
      txt_marker.mutable_scale()->set_z(0.5);
      markers.add_markers()->CopyFrom(txt_marker);
    }
    util::RvizAgent::Instance().Publish(topic, markers);
  }

  static void PubImmatureMapZebraCrossingMarker(
      const Eigen::Affine3d& T_W_V,
      const std::vector<ZebraCrossing>& map_zebra_crossings,
      const uint64_t& sec, const uint64_t& nsec, const std::string& topic) {
    static bool register_flag = true;
    if (register_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::MarkerArray>(
          topic);
      register_flag = false;
    }
    adsfi_proto::viz::MarkerArray markers;
    int id = 0;
    for (const auto& zebra_crossing : map_zebra_crossings) {
      if (zebra_crossing.state == TrackState::MATURED) {
        continue;
      }
      adsfi_proto::viz::Marker txt_marker;
      txt_marker.set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
      txt_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
      txt_marker.set_id(id++);
      txt_marker.mutable_lifetime()->set_sec(0);
      txt_marker.mutable_lifetime()->set_nsec(200000000);
      txt_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
      txt_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
      txt_marker.mutable_header()->set_frameid("localmap");
      Eigen::Vector3d point = {zebra_crossing.center_point.x(),
                               zebra_crossing.center_point.y(), 0};
      point = T_W_V * point;
      txt_marker.mutable_pose()->mutable_position()->set_x(point.x());
      txt_marker.mutable_pose()->mutable_position()->set_y(point.y());
      txt_marker.mutable_pose()->mutable_position()->set_z(2);
      txt_marker.mutable_pose()->mutable_orientation()->set_x(0);
      txt_marker.mutable_pose()->mutable_orientation()->set_y(0);
      txt_marker.mutable_pose()->mutable_orientation()->set_z(0);
      txt_marker.mutable_pose()->mutable_orientation()->set_w(1);
      txt_marker.mutable_color()->set_r(0.68);
      txt_marker.mutable_color()->set_g(0.50);
      txt_marker.mutable_color()->set_b(0.66);
      txt_marker.mutable_color()->set_a(1);
      txt_marker.set_text(std::to_string(zebra_crossing.tracked_count));
      txt_marker.mutable_scale()->set_x(2);
      txt_marker.mutable_scale()->set_y(2);
      txt_marker.mutable_scale()->set_z(2);
      markers.add_markers()->CopyFrom(txt_marker);
    }
    util::RvizAgent::Instance().Publish(topic, markers);
  }

  static void PubMapFreespace(
      const Eigen::Affine3d& T_W_V,
      const std::vector<FreeSpaceOutput>& map_freespace_outputs,
      const uint64_t& sec, const uint64_t& nsec, const std::string& topic) {
    static bool register_flag = true;
    if (register_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::PointCloud>(topic);
      register_flag = false;
    }
    adsfi_proto::viz::PointCloud points_msg;
    points_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
    points_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    points_msg.mutable_header()->set_frameid("localmap");
    auto* channels = points_msg.add_channels();
    channels->set_name("rgb");
    for (const auto& freespace_output : map_freespace_outputs) {
      if (!freespace_output.free) {
        continue;
      }
      for (const auto& points : freespace_output.vec_edges_points) {
        for (const auto& point : points) {
          auto point_world = T_W_V * point;
          auto* point_msg = points_msg.add_points();
          point_msg->set_x(static_cast<float>(point_world.x()));
          point_msg->set_y(static_cast<float>(point_world.y()));
          point_msg->set_z(static_cast<float>(point_world.z()));
        }
      }
    }
    util::RvizAgent::Instance().Publish(topic, points_msg);
  }

  static void PubMapNonFreespace(
      const Eigen::Affine3d& T_W_V,
      const std::vector<FreeSpaceOutput>& map_freespace_outputs,
      const uint64_t& sec, const uint64_t& nsec, const std::string& topic) {
    static bool register_flag = true;
    if (register_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::PointCloud>(topic);
      register_flag = false;
    }
    adsfi_proto::viz::PointCloud points_msg;
    points_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
    points_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    points_msg.mutable_header()->set_frameid("localmap");
    auto* channels = points_msg.add_channels();
    channels->set_name("rgb");
    for (const auto& freespace_output : map_freespace_outputs) {
      if (freespace_output.free) {
        continue;
      }
      for (const auto& points : freespace_output.vec_edges_points) {
        for (const auto& point : points) {
          auto point_world = T_W_V * point;
          auto* point_msg = points_msg.add_points();
          point_msg->set_x(static_cast<float>(point_world.x()));
          point_msg->set_y(static_cast<float>(point_world.y()));
          point_msg->set_z(static_cast<float>(point_world.z()));
        }
      }
    }
    util::RvizAgent::Instance().Publish(topic, points_msg);
  }
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
