/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: wangjianguo
 *Date: 2023-09-06
 *****************************************************************************/
#pragma once

#include <algorithm>
#include <cstddef>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "Eigen/Dense"
#include "Eigen/src/Core/Matrix.h"
#include "depend/common/utm_projection/coordinate_convertor.h"
#include "depend/map/hdmap/hdmap.h"
#include "depend/proto/soc/sensor_image.pb.h"
#include "interface/adsfi_proto/viz/sensor_msgs.pb.h"
#include "modules/local_mapping/types/types.h"
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
  static void PubOdom(const Sophus::SE3d& T_W_V, const uint64_t& sec,
                      const uint64_t& nsec, const std::string& topic) {
    Eigen::Vector3d p = T_W_V.translation();
    Eigen::Quaterniond q = T_W_V.so3().unit_quaternion();
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

  static void PubTf(const Sophus::SE3d& T_W_V, const uint64_t& sec,
                    const uint64_t& nsec, const std::string& topic) {
    Eigen::Vector3d p = T_W_V.translation();
    Eigen::Quaterniond q = T_W_V.so3().unit_quaternion();
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

  static void PubPath(const Sophus::SE3d& T_W_V, const uint64_t& sec,
                      const uint64_t& nsec, const std::string& topic) {
    Eigen::Vector3d p = T_W_V.translation();
    Eigen::Quaterniond q = T_W_V.so3().unit_quaternion();
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

  static void PubPerLaneLine(const Sophus::SE3d& T_W_V,
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
      for (const auto& point : lane_line.points_) {
        auto point_world = T_W_V * point;
        auto* point_msg = points_msg.add_points();
        point_msg->set_x(static_cast<float>(point_world.x()));
        point_msg->set_y(static_cast<float>(point_world.y()));
        point_msg->set_z(static_cast<float>(point_world.z()));
      }
    }
    util::RvizAgent::Instance().Publish(topic, points_msg);
  }

  static void PubPerRoadEdge(const Sophus::SE3d& T_W_V,
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
      for (const auto& point : road_edge.points_) {
        auto point_world = T_W_V * point;
        auto* point_msg = points_msg.add_points();
        point_msg->set_x(static_cast<float>(point_world.x()));
        point_msg->set_y(static_cast<float>(point_world.y()));
        point_msg->set_z(static_cast<float>(point_world.z()));
      }
    }
    util::RvizAgent::Instance().Publish(topic, points_msg);
  }

  static void PubPerStopLine(const Sophus::SE3d& T_W_V,
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
      auto point = T_W_V * stop_line.left_point_;
      auto* left_point = marker.add_points();
      left_point->set_x(static_cast<float>(point.x()));
      left_point->set_y(static_cast<float>(point.y()));
      left_point->set_z(static_cast<float>(point.z()));
      point = T_W_V * stop_line.right_point_;
      auto* right_point = marker.add_points();
      right_point->set_x(static_cast<float>(point.x()));
      right_point->set_y(static_cast<float>(point.y()));
      right_point->set_z(static_cast<float>(point.z()));
      markers.add_markers()->CopyFrom(marker);
    }
    util::RvizAgent::Instance().Publish(topic, markers);
  }

  static void PubPerArrow(const Sophus::SE3d& T_W_V,
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
      Eigen::Vector3d point_0 = arrow.points_.points_[0];
      point_0 = T_W_V * point_0;
      Eigen::Vector3d point_1 = arrow.points_.points_[1];
      point_1 = T_W_V * point_1;
      Eigen::Vector3d point_2 = arrow.points_.points_[2];
      point_2 = T_W_V * point_2;
      Eigen::Vector3d point_3 = arrow.points_.points_[3];
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
          {ArrowType::ARROWTYPE_UNKNOWN, "ARROWTYPE_UNKNOWN"},
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
      txt_marker.set_text(ArrowTypeToString[arrow.type_]);
      txt_marker.mutable_scale()->set_x(1);
      txt_marker.mutable_scale()->set_y(1);
      txt_marker.mutable_scale()->set_z(0.5);
      markers.add_markers()->CopyFrom(txt_marker);
    }

    util::RvizAgent::Instance().Publish(topic, markers);
  }

  static void PubPerZebraCrossing(
      const Sophus::SE3d& T_W_V,
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
      Eigen::Vector3d point_0 = zebra_crossing.points_.points_[0];
      point_0 = T_W_V * point_0;
      Eigen::Vector3d point_1 = zebra_crossing.points_.points_[1];
      point_1 = T_W_V * point_1;
      Eigen::Vector3d point_2 = zebra_crossing.points_.points_[2];
      point_2 = T_W_V * point_2;
      Eigen::Vector3d point_3 = zebra_crossing.points_.points_[3];
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

  static void PubMapLaneLine(const Sophus::SE3d& T_W_V,
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
      if (!lane_line.ismature_) {
        continue;
      }
      for (const auto& point : lane_line.fit_points_) {
        auto point_world = T_W_V * point;
        auto* point_msg = points_msg.add_points();
        point_msg->set_x(static_cast<float>(point_world.x()));
        point_msg->set_y(static_cast<float>(point_world.y()));
        point_msg->set_z(static_cast<float>(point_world.z()));
      }
    }
    util::RvizAgent::Instance().Publish(topic, points_msg);
  }

  static void PubMapLaneLineOri(const Sophus::SE3d& T_W_V,
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
      if (!lane_line.ismature_) {
        continue;
      }
      for (const auto& point : lane_line.points_) {
        auto point_world = T_W_V * point;
        auto* point_msg = points_msg.add_points();
        point_msg->set_x(static_cast<float>(point_world.x()));
        point_msg->set_y(static_cast<float>(point_world.y()));
        point_msg->set_z(static_cast<float>(point_world.z()));
      }
    }
    util::RvizAgent::Instance().Publish(topic, points_msg);
  }

  static void PubMapRoadEdge(const Sophus::SE3d& T_W_V,
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
      if (!road_edge.ismature_) {
        continue;
      }
      for (const auto& point : road_edge.fit_points_) {
        auto point_world = T_W_V * point;
        auto* point_msg = points_msg.add_points();
        point_msg->set_x(static_cast<float>(point_world.x()));
        point_msg->set_y(static_cast<float>(point_world.y()));
        point_msg->set_z(static_cast<float>(point_world.z()));
      }
    }
    util::RvizAgent::Instance().Publish(topic, points_msg);
  }

  static void PubMapStopLine(const Sophus::SE3d& T_W_V,
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
      if (!stop_line.ismature_) {
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
      Eigen::Vector3d left_point_ = {
          stop_line.mid_point_.x() +
              stop_line.length_ / 2.0 * cos(stop_line.heading_),
          stop_line.mid_point_.y() +
              stop_line.length_ / 2.0 * sin(stop_line.heading_),
          0};
      auto point = T_W_V * left_point_;
      auto* left_point = marker.add_points();
      left_point->set_x(static_cast<float>(point.x()));
      left_point->set_y(static_cast<float>(point.y()));
      left_point->set_z(static_cast<float>(point.z()));
      Eigen::Vector3d right_point_ = {
          stop_line.mid_point_.x() -
              stop_line.length_ / 2.0 * cos(stop_line.heading_),
          stop_line.mid_point_.y() -
              stop_line.length_ / 2.0 * sin(stop_line.heading_),
          0};
      point = T_W_V * right_point_;
      auto* right_point = marker.add_points();
      right_point->set_x(static_cast<float>(point.x()));
      right_point->set_y(static_cast<float>(point.y()));
      right_point->set_z(static_cast<float>(point.z()));
      markers.add_markers()->CopyFrom(marker);
    }
    util::RvizAgent::Instance().Publish(topic, markers);
  }

  static void PubMapArrow(const Sophus::SE3d& T_W_V,
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
      if (!arrow.ismature_) {
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
      Eigen::Vector3d l = {arrow.length_ / 2 * cos(arrow.heading_),
                           arrow.length_ / 2 * sin(arrow.heading_), 0};
      Eigen::Vector3d w = {-arrow.width_ / 2 * sin(arrow.heading_),
                           arrow.width_ / 2 * cos(arrow.heading_), 0};
      Eigen::Vector3d point_0 = arrow.mid_point_ + l + w;
      point_0 = T_W_V * point_0;
      Eigen::Vector3d point_1 = arrow.mid_point_ - l + w;
      point_1 = T_W_V * point_1;
      Eigen::Vector3d point_2 = arrow.mid_point_ - l - w;
      point_2 = T_W_V * point_2;
      Eigen::Vector3d point_3 = arrow.mid_point_ + l - w;
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
          {ArrowType::ARROWTYPE_UNKNOWN, "ARROWTYPE_UNKNOWN"},
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
      txt_marker.set_text(ArrowTypeToString[arrow.type_]);
      txt_marker.mutable_scale()->set_x(1);
      txt_marker.mutable_scale()->set_y(1);
      txt_marker.mutable_scale()->set_z(0.5);
      markers.add_markers()->CopyFrom(txt_marker);
    }

    util::RvizAgent::Instance().Publish(topic, markers);
  }

  static void PubMapZebraCrossing(
      const Sophus::SE3d& T_W_V,
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
      if (!zebra_crossing.ismature_) {
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
      Eigen::Vector3d l = {
          -zebra_crossing.length_ / 2 * sin(zebra_crossing.heading_),
          zebra_crossing.length_ / 2 * cos(zebra_crossing.heading_), 0};
      Eigen::Vector3d w = {
          zebra_crossing.width_ / 2 * cos(zebra_crossing.heading_),
          zebra_crossing.width_ / 2 * sin(zebra_crossing.heading_), 0};
      Eigen::Vector3d point_0 = zebra_crossing.mid_point_ + l + w;
      point_0 = T_W_V * point_0;
      Eigen::Vector3d point_1 = zebra_crossing.mid_point_ + l - w;
      point_1 = T_W_V * point_1;
      Eigen::Vector3d point_2 = zebra_crossing.mid_point_ - l - w;
      point_2 = T_W_V * point_2;
      Eigen::Vector3d point_3 = zebra_crossing.mid_point_ - l + w;
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

  static void PubImmatureMapZebraCrossing(
      const Sophus::SE3d& T_W_V,
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
      if (zebra_crossing.ismature_) {
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
      Eigen::Vector3d l = {
          -zebra_crossing.length_ / 2 * sin(zebra_crossing.heading_),
          zebra_crossing.length_ / 2 * cos(zebra_crossing.heading_), 0};
      Eigen::Vector3d w = {
          zebra_crossing.width_ / 2 * cos(zebra_crossing.heading_),
          zebra_crossing.width_ / 2 * sin(zebra_crossing.heading_), 0};
      Eigen::Vector3d point_0 = zebra_crossing.mid_point_ + l + w;
      point_0 = T_W_V * point_0;
      Eigen::Vector3d point_1 = zebra_crossing.mid_point_ - l + w;
      point_1 = T_W_V * point_1;
      Eigen::Vector3d point_2 = zebra_crossing.mid_point_ - l - w;
      point_2 = T_W_V * point_2;
      Eigen::Vector3d point_3 = zebra_crossing.mid_point_ + l - w;
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
      const Sophus::SE3d& T_W_V,
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
      if (zebra_crossing.ismature_) {
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
      Eigen::Vector3d point = {zebra_crossing.mid_point_.x(),
                               zebra_crossing.mid_point_.y(), 0};
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
      txt_marker.set_text(std::to_string(zebra_crossing.tracked_count_));
      txt_marker.mutable_scale()->set_x(1);
      txt_marker.mutable_scale()->set_y(1);
      txt_marker.mutable_scale()->set_z(1);
      markers.add_markers()->CopyFrom(txt_marker);
    }
    util::RvizAgent::Instance().Publish(topic, markers);
  }

  static void PubImmatureMapStopLine(
      const Sophus::SE3d& T_W_V, const std::vector<StopLine>& map_stop_lines,
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
      if (stop_line.ismature_) {
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
      Eigen::Vector3d left_point_ = {
          stop_line.mid_point_.x() +
              stop_line.length_ / 2.0 * cos(stop_line.heading_),
          stop_line.mid_point_.y() +
              stop_line.length_ / 2.0 * sin(stop_line.heading_),
          0};
      auto point = T_W_V * left_point_;
      auto* left_point = marker.add_points();
      left_point->set_x(static_cast<float>(point.x()));
      left_point->set_y(static_cast<float>(point.y()));
      left_point->set_z(static_cast<float>(point.z()));
      Eigen::Vector3d right_point_ = {
          stop_line.mid_point_.x() -
              stop_line.length_ / 2.0 * cos(stop_line.heading_),
          stop_line.mid_point_.y() -
              stop_line.length_ / 2.0 * sin(stop_line.heading_),
          0};
      point = T_W_V * right_point_;
      auto* right_point = marker.add_points();
      right_point->set_x(static_cast<float>(point.x()));
      right_point->set_y(static_cast<float>(point.y()));
      right_point->set_z(static_cast<float>(point.z()));
      markers.add_markers()->CopyFrom(marker);
    }
    util::RvizAgent::Instance().Publish(topic, markers);
  }

  static void PubImmatureMapStopLineMarker(
      const Sophus::SE3d& T_W_V, const std::vector<StopLine>& map_stop_lines,
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
      if (stop_line.ismature_) {
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
      Eigen::Vector3d point = {stop_line.mid_point_.x(),
                               stop_line.mid_point_.y(), 0};
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
      txt_marker.set_text(std::to_string(stop_line.tracked_count_));
      txt_marker.mutable_scale()->set_x(1);
      txt_marker.mutable_scale()->set_y(1);
      txt_marker.mutable_scale()->set_z(1);
      markers.add_markers()->CopyFrom(txt_marker);
    }
    util::RvizAgent::Instance().Publish(topic, markers);
  }

  static void PubImmatureMapArrow(const Sophus::SE3d& T_W_V,
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
      if (arrow.ismature_) {
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
      Eigen::Vector3d l = {arrow.length_ / 2 * cos(arrow.heading_),
                           arrow.length_ / 2 * sin(arrow.heading_), 0};
      Eigen::Vector3d w = {-arrow.width_ / 2 * sin(arrow.heading_),
                           arrow.width_ / 2 * cos(arrow.heading_), 0};
      Eigen::Vector3d point_0 = arrow.mid_point_ + l + w;
      point_0 = T_W_V * point_0;
      Eigen::Vector3d point_1 = arrow.mid_point_ - l + w;
      point_1 = T_W_V * point_1;
      Eigen::Vector3d point_2 = arrow.mid_point_ - l - w;
      point_2 = T_W_V * point_2;
      Eigen::Vector3d point_3 = arrow.mid_point_ + l - w;
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
          {ArrowType::ARROWTYPE_UNKNOWN, "ARROWTYPE_UNKNOWN"},
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
      txt_marker.set_text(ArrowTypeToString[arrow.type_]);
      txt_marker.mutable_scale()->set_x(1);
      txt_marker.mutable_scale()->set_y(1);
      txt_marker.mutable_scale()->set_z(0.5);
      markers.add_markers()->CopyFrom(txt_marker);
    }

    util::RvizAgent::Instance().Publish(topic, markers);
  }

  static void PubImmatureMapArrowMarker(const Sophus::SE3d& T_W_V,
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
      if (arrow.ismature_) {
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
      Eigen::Vector3d point = {arrow.mid_point_.x(), arrow.mid_point_.y(), 0};
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
      txt_marker.set_text(std::to_string(arrow.tracked_count_));
      txt_marker.mutable_scale()->set_x(1);
      txt_marker.mutable_scale()->set_y(1);
      txt_marker.mutable_scale()->set_z(1);
      markers.add_markers()->CopyFrom(txt_marker);
    }
    util::RvizAgent::Instance().Publish(topic, markers);
  }

  static void PubMapLaneLineControl(const Sophus::SE3d& T_W_V,
                                    const LocalMap& local_map,
                                    const uint64_t& sec, const uint64_t& nsec,
                                    const std::string& topic) {
    static bool map_control_flag = true;
    if (map_control_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::PointCloud>(topic);
      map_control_flag = false;
    }
    adsfi_proto::viz::PointCloud points_msg;
    points_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
    points_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    points_msg.mutable_header()->set_frameid("localmap");
    auto* channels = points_msg.add_channels();
    channels->set_name("rgb");
    for (const auto& lane_line : local_map.lane_lines_) {
      if (!lane_line.ismature_) {
        continue;
      }
      for (const auto& point : lane_line.control_points_) {
        auto point_world = T_W_V * point;
        auto* point_msg = points_msg.add_points();
        point_msg->set_x(static_cast<float>(point_world.x()));
        point_msg->set_y(static_cast<float>(point_world.y()));
        point_msg->set_z(static_cast<float>(point_world.z()));
      }
    }
    util::RvizAgent::Instance().Publish(topic, points_msg);
  }

  static void PubMapLaneLineMarker(const Sophus::SE3d& T_W_V,
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
      if (!lane_line.ismature_ || lane_line.fit_points_.empty()) {
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
      Eigen::Vector3d point = {lane_line.fit_points_.back().x(),
                               lane_line.fit_points_.back().y(), 0};
      point = T_W_V * point;
      txt_marker.mutable_pose()->mutable_position()->set_x(point.x());
      txt_marker.mutable_pose()->mutable_position()->set_y(point.y());
      txt_marker.mutable_pose()->mutable_position()->set_z(2);
      txt_marker.mutable_pose()->mutable_orientation()->set_x(0);
      txt_marker.mutable_pose()->mutable_orientation()->set_y(0);
      txt_marker.mutable_pose()->mutable_orientation()->set_z(0);
      txt_marker.mutable_pose()->mutable_orientation()->set_w(1);
      txt_marker.mutable_color()->set_r(0);
      txt_marker.mutable_color()->set_g(1);
      txt_marker.mutable_color()->set_b(0);
      txt_marker.mutable_color()->set_a(1);
      txt_marker.set_text(std::to_string(lane_line.track_id_) + ", " +
                          std::to_string(lane_line.lanepos_) + ", " +
                          std::to_string(lane_line.lanetype_));
      txt_marker.mutable_scale()->set_x(1);
      txt_marker.mutable_scale()->set_y(1);
      txt_marker.mutable_scale()->set_z(1);
      markers.add_markers()->CopyFrom(txt_marker);
    }
    util::RvizAgent::Instance().Publish(topic, markers);
  }
  static void PubMapRoadEdgeMarker(const Sophus::SE3d& T_W_V,
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
      if (!road_edge.ismature_ || road_edge.fit_points_.empty()) {
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
      Eigen::Vector3d point = {road_edge.fit_points_.back().x() - 10,
                               road_edge.fit_points_.back().y(), 0};
      point = T_W_V * point;
      txt_marker.mutable_pose()->mutable_position()->set_x(point.x());
      txt_marker.mutable_pose()->mutable_position()->set_y(point.y());
      txt_marker.mutable_pose()->mutable_position()->set_z(2);
      txt_marker.mutable_pose()->mutable_orientation()->set_x(0);
      txt_marker.mutable_pose()->mutable_orientation()->set_y(0);
      txt_marker.mutable_pose()->mutable_orientation()->set_z(0);
      txt_marker.mutable_pose()->mutable_orientation()->set_w(1);
      txt_marker.mutable_color()->set_r(1);
      txt_marker.mutable_color()->set_g(1);
      txt_marker.mutable_color()->set_b(1);
      txt_marker.mutable_color()->set_a(1);
      txt_marker.set_text(std::to_string(road_edge.track_id_) + ", " +
                          std::to_string(road_edge.lanepos_) + ", " +
                          std::to_string(road_edge.edgetype_));
      txt_marker.mutable_scale()->set_x(1);
      txt_marker.mutable_scale()->set_y(1);
      txt_marker.mutable_scale()->set_z(1);
      markers.add_markers()->CopyFrom(txt_marker);
    }
    util::RvizAgent::Instance().Publish(topic, markers);
  }

  static void PubImmatureMapLaneLine(
      const Sophus::SE3d& T_W_V, const std::vector<LaneLine>& map_lane_lines,
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
      if (lane_line.ismature_) {
        continue;
      }
      for (const auto& point : lane_line.points_) {
        auto point_world = T_W_V * point;
        auto* point_msg = points_msg.add_points();
        point_msg->set_x(static_cast<float>(point_world.x()));
        point_msg->set_y(static_cast<float>(point_world.y()));
        point_msg->set_z(static_cast<float>(point_world.z()));
      }
    }
    util::RvizAgent::Instance().Publish(topic, points_msg);
  }

  static void PubImmatureMapLaneLineMarker(
      const Sophus::SE3d& T_W_V, const std::vector<LaneLine>& map_lane_lines,
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
      if (lane_line.ismature_ || lane_line.fit_points_.empty()) {
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
      Eigen::Vector3d point = {lane_line.fit_points_.back().x(),
                               lane_line.fit_points_.back().y(), 0};
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
      txt_marker.set_text(std::to_string(lane_line.tracked_count_));
      txt_marker.mutable_scale()->set_x(1);
      txt_marker.mutable_scale()->set_y(1);
      txt_marker.mutable_scale()->set_z(1);
      markers.add_markers()->CopyFrom(txt_marker);
    }
    util::RvizAgent::Instance().Publish(topic, markers);
  }

  static void PubHdMapPoints(const Sophus::SE3d& T_G_V,
                             const Sophus::SE3d& T_W_V,
                             const std::shared_ptr<hozon::hdmap::HDMap>& hdmap,
                             const uint64_t& sec, const uint64_t& nsec,
                             const std::string& topic) {
    static bool hd_map_flag = true;
    static Eigen::Vector3d ref_point;
    if (hd_map_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::MarkerArray>(
          topic);
      ref_point << T_G_V.translation().x(), T_G_V.translation().y(), 0;
      hd_map_flag = false;
    }
    Sophus::SE3d T_U_V;
    CommonUtil::Gcj02ToUtm(T_G_V, &T_U_V);
    hozon::common::PointENU pos_u_v;
    pos_u_v.set_x(T_U_V.matrix()(0, 3));
    pos_u_v.set_y(T_U_V.matrix()(1, 3));
    pos_u_v.set_z(0);
    std::vector<hozon::hdmap::LaneInfoConstPtr> lanes;
    hdmap->GetLanes(pos_u_v, 150, &lanes);
    adsfi_proto::viz::MarkerArray markers;
    int id = 0;
    for (const auto& lane : lanes) {
      for (const auto& i : lane->lane().left_boundary().curve().segment()) {
        std::vector<Eigen::Vector3d> hd_points;
        for (const auto& point : i.line_segment().point()) {
          Eigen::Vector3d point_utm(point.x(), point.y(), 0);
          Eigen::Vector3d point_vehicle =
              CommonUtil::UtmPointToVehicle(point_utm, ref_point, T_G_V);
          Eigen::Vector3d point_world = T_W_V * point_vehicle;
          hd_points.emplace_back(point_world);
        }
        adsfi_proto::viz::Marker marker_msg;
        marker_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
        marker_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
        marker_msg.mutable_header()->set_frameid("localmap");
        marker_msg.set_ns("localmap");
        marker_msg.set_id(id++);
        marker_msg.set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
        marker_msg.set_action(adsfi_proto::viz::MarkerAction::ADD);
        marker_msg.mutable_pose()->mutable_orientation()->set_x(0.);
        marker_msg.mutable_pose()->mutable_orientation()->set_y(0.);
        marker_msg.mutable_pose()->mutable_orientation()->set_z(0.);
        marker_msg.mutable_pose()->mutable_orientation()->set_w(1.);
        marker_msg.mutable_scale()->set_x(0.1);
        marker_msg.mutable_lifetime()->set_sec(1);
        marker_msg.mutable_lifetime()->set_nsec(0);
        adsfi_proto::viz::ColorRGBA color;
        color.set_a(1.0);
        color.set_r(1.0);
        color.set_g(1.0);
        color.set_b(1.0);
        marker_msg.mutable_color()->CopyFrom(color);
        AppendPoints(hd_points, &marker_msg);
        markers.add_markers()->CopyFrom(marker_msg);
      }
      for (const auto& i : lane->lane().right_boundary().curve().segment()) {
        std::vector<Eigen::Vector3d> hd_points;
        for (const auto& point : i.line_segment().point()) {
          Eigen::Vector3d point_utm(point.x(), point.y(), 0);
          Eigen::Vector3d point_vehicle =
              CommonUtil::UtmPointToVehicle(point_utm, ref_point, T_G_V);
          Eigen::Vector3d point_world = T_W_V * point_vehicle;
          hd_points.emplace_back(point_world);
        }
        adsfi_proto::viz::Marker marker_msg;
        marker_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
        marker_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
        marker_msg.mutable_header()->set_frameid("localmap");
        marker_msg.set_ns("localmap");
        marker_msg.set_id(id++);
        marker_msg.set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
        marker_msg.set_action(adsfi_proto::viz::MarkerAction::ADD);
        marker_msg.mutable_pose()->mutable_orientation()->set_x(0.);
        marker_msg.mutable_pose()->mutable_orientation()->set_y(0.);
        marker_msg.mutable_pose()->mutable_orientation()->set_z(0.);
        marker_msg.mutable_pose()->mutable_orientation()->set_w(1.);
        marker_msg.mutable_scale()->set_x(0.1);
        marker_msg.mutable_lifetime()->set_sec(1);
        marker_msg.mutable_lifetime()->set_nsec(0);
        adsfi_proto::viz::ColorRGBA color;
        color.set_a(1.0);
        color.set_r(1.0);
        color.set_g(1.0);
        color.set_b(1.0);
        marker_msg.mutable_color()->CopyFrom(color);
        AppendPoints(hd_points, &marker_msg);
        markers.add_markers()->CopyFrom(marker_msg);
      }
    }
    util::RvizAgent::Instance().Publish(topic, markers);
  }

  static void AppendPoints(const std::vector<Eigen::Vector3d>& points,
                           adsfi_proto::viz::Marker* marker_msg) {
    for (const auto& point : points) {
      auto* marker_point = marker_msg->add_points();
      marker_point->set_x(point[0]);
      marker_point->set_y(point[1]);
      marker_point->set_z(0);
    }
  }

  static void PubHqMapPoints(const Sophus::SE3d& T_G_V,
                             const Sophus::SE3d& T_W_V, const uint64_t& sec,
                             const uint64_t& nsec, const std::string& topic) {
    static bool hq_map_flag = true;
    static Eigen::Vector3d ref_point;
    if (hq_map_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::MarkerArray>(
          topic);
      ref_point << T_G_V.translation().x(), T_G_V.translation().y(), 0;
      hq_map_flag = false;
    }
    Sophus::SE3d T_U_V;
    CommonUtil::Gcj02ToUtm(T_G_V, &T_U_V);
    hozon::common::PointENU pos_u_v;
    pos_u_v.set_x(T_U_V.matrix()(0, 3));
    pos_u_v.set_y(T_U_V.matrix()(1, 3));
    pos_u_v.set_z(0);
    std::vector<hozon::hdmap::LaneInfoConstPtr> lanes;
    GLOBAL_HD_MAP->GetLanes(pos_u_v, 150, &lanes);
    adsfi_proto::viz::MarkerArray markers;
    int id = 0;
    for (const auto& lane : lanes) {
      for (const auto& i : lane->lane().left_boundary().curve().segment()) {
        std::vector<Eigen::Vector3d> hd_points;
        for (const auto& point : i.line_segment().point()) {
          Eigen::Vector3d point_utm(point.x(), point.y(), 0);
          Eigen::Vector3d point_vehicle =
              CommonUtil::UtmPointToVehicle(point_utm, ref_point, T_G_V);
          Eigen::Vector3d point_world = T_W_V * point_vehicle;
          hd_points.emplace_back(point_world);
        }
        adsfi_proto::viz::Marker marker_msg;
        marker_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
        marker_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
        marker_msg.mutable_header()->set_frameid("localmap");
        marker_msg.set_ns("localmap");
        marker_msg.set_id(id++);
        marker_msg.set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
        marker_msg.set_action(adsfi_proto::viz::MarkerAction::ADD);
        marker_msg.mutable_pose()->mutable_orientation()->set_x(0.);
        marker_msg.mutable_pose()->mutable_orientation()->set_y(0.);
        marker_msg.mutable_pose()->mutable_orientation()->set_z(0.);
        marker_msg.mutable_pose()->mutable_orientation()->set_w(1.);
        marker_msg.mutable_scale()->set_x(0.1);
        marker_msg.mutable_lifetime()->set_sec(1);
        marker_msg.mutable_lifetime()->set_nsec(0);
        adsfi_proto::viz::ColorRGBA color;
        color.set_a(1.0);
        color.set_r(1.0);
        color.set_g(1.0);
        color.set_b(1.0);
        marker_msg.mutable_color()->CopyFrom(color);
        AppendPoints(hd_points, &marker_msg);
        markers.add_markers()->CopyFrom(marker_msg);
      }
      for (const auto& i : lane->lane().right_boundary().curve().segment()) {
        std::vector<Eigen::Vector3d> hd_points;
        for (const auto& point : i.line_segment().point()) {
          Eigen::Vector3d point_utm(point.x(), point.y(), 0);
          Eigen::Vector3d point_vehicle =
              CommonUtil::UtmPointToVehicle(point_utm, ref_point, T_G_V);
          Eigen::Vector3d point_world = T_W_V * point_vehicle;
          hd_points.emplace_back(point_world);
        }
        adsfi_proto::viz::Marker marker_msg;
        marker_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
        marker_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
        marker_msg.mutable_header()->set_frameid("localmap");
        marker_msg.set_ns("localmap");
        marker_msg.set_id(id++);
        marker_msg.set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
        marker_msg.set_action(adsfi_proto::viz::MarkerAction::ADD);
        marker_msg.mutable_pose()->mutable_orientation()->set_x(0.);
        marker_msg.mutable_pose()->mutable_orientation()->set_y(0.);
        marker_msg.mutable_pose()->mutable_orientation()->set_z(0.);
        marker_msg.mutable_pose()->mutable_orientation()->set_w(1.);
        marker_msg.mutable_scale()->set_x(0.1);
        marker_msg.mutable_lifetime()->set_sec(1);
        marker_msg.mutable_lifetime()->set_nsec(0);
        adsfi_proto::viz::ColorRGBA color;
        color.set_a(1.0);
        color.set_r(1.0);
        color.set_g(1.0);
        color.set_b(1.0);
        marker_msg.mutable_color()->CopyFrom(color);
        AppendPoints(hd_points, &marker_msg);
        markers.add_markers()->CopyFrom(marker_msg);
      }
    }
    util::RvizAgent::Instance().Publish(topic, markers);
  }
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
