/******************************************************************************
 *   Copyright (C) 2024 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： reloc_rviz.hpp
 *   author     ： zhaohaowu
 *   date       ： 2024.04
 ******************************************************************************/

#pragma once

#include <Eigen/Dense>
#include <string>

#include "modules/util/include/util/rviz_agent/rviz_agent.h"
#include "reloc/reloc.hpp"

namespace hozon {
namespace mp {
namespace loc {
namespace pe {
class RelocRviz {
 public:
  static void PubFcOdom(const Eigen::Affine3d& T_W_V, uint64_t sec,
                        uint64_t nsec, const std::string& topic) {
    static bool register_flag = true;
    if (register_flag) {
      RVIZ_AGENT.Register<adsfi_proto::viz::Odometry>(topic);
      register_flag = false;
    }
    Eigen::Vector3d p = T_W_V.translation();
    Eigen::Quaterniond q(T_W_V.rotation());
    adsfi_proto::viz::Odometry odom_msg;
    odom_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
    odom_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    odom_msg.mutable_header()->set_frameid("map");
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
    RVIZ_AGENT.Publish(topic, odom_msg);
  }

  static void PubInsOdom(const Eigen::Affine3d& T_W_V, uint64_t sec,
                         uint64_t nsec, const std::string& topic) {
    static bool register_flag = true;
    if (register_flag) {
      RVIZ_AGENT.Register<adsfi_proto::viz::Odometry>(topic);
      register_flag = false;
    }
    Eigen::Vector3d p = T_W_V.translation();
    Eigen::Quaterniond q(T_W_V.rotation());
    adsfi_proto::viz::Odometry odom_msg;
    odom_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
    odom_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    odom_msg.mutable_header()->set_frameid("map");
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
    RVIZ_AGENT.Publish(topic, odom_msg);
  }

  static void PubInputOdom(const Eigen::Affine3d& T_W_V, uint64_t sec,
                           uint64_t nsec, const std::string& topic) {
    static bool register_flag = true;
    if (register_flag) {
      RVIZ_AGENT.Register<adsfi_proto::viz::Odometry>(topic);
      register_flag = false;
    }
    Eigen::Vector3d p = T_W_V.translation();
    Eigen::Quaterniond q(T_W_V.rotation());
    adsfi_proto::viz::Odometry odom_msg;
    odom_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
    odom_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    odom_msg.mutable_header()->set_frameid("map");
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
    RVIZ_AGENT.Publish(topic, odom_msg);
  }

  static void PubRelocOdom(const Eigen::Affine3d& T_W_V, uint64_t sec,
                           uint64_t nsec, const std::string& topic) {
    static bool register_flag = true;
    if (register_flag) {
      RVIZ_AGENT.Register<adsfi_proto::viz::Odometry>(topic);
      register_flag = false;
    }
    Eigen::Vector3d p = T_W_V.translation();
    Eigen::Quaterniond q(T_W_V.rotation());
    adsfi_proto::viz::Odometry odom_msg;
    odom_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
    odom_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    odom_msg.mutable_header()->set_frameid("map");
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
    RVIZ_AGENT.Publish(topic, odom_msg);
  }

  static void PubFcTf(const Eigen::Affine3d& T_W_V, uint64_t sec, uint64_t nsec,
                      const std::string& topic) {
    Eigen::Vector3d p = T_W_V.translation();
    Eigen::Quaterniond q(T_W_V.rotation());
    static bool register_flag = true;
    if (register_flag) {
      RVIZ_AGENT.Register<adsfi_proto::viz::TransformStamped>(topic);
      register_flag = false;
    }
    adsfi_proto::viz::TransformStamped tf_msg;
    tf_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
    tf_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    tf_msg.mutable_header()->set_frameid("map");
    tf_msg.set_child_frame_id("vehicle");
    tf_msg.mutable_transform()->mutable_translation()->set_x(p.x());
    tf_msg.mutable_transform()->mutable_translation()->set_y(p.y());
    tf_msg.mutable_transform()->mutable_translation()->set_z(p.z());
    tf_msg.mutable_transform()->mutable_rotation()->set_x(q.x());
    tf_msg.mutable_transform()->mutable_rotation()->set_y(q.y());
    tf_msg.mutable_transform()->mutable_rotation()->set_z(q.z());
    tf_msg.mutable_transform()->mutable_rotation()->set_w(q.w());
    RVIZ_AGENT.Publish(topic, tf_msg);
  }

  static void PubFcPath(const Eigen::Affine3d& T_W_V, uint64_t sec,
                        uint64_t nsec, const std::string& topic) {
    Eigen::Vector3d p = T_W_V.translation();
    Eigen::Quaterniond q(T_W_V.rotation());
    static bool register_flag = true;
    if (register_flag) {
      RVIZ_AGENT.Register<adsfi_proto::viz::Path>(topic);
      register_flag = false;
    }
    static adsfi_proto::viz::Path path_msg;
    auto* pose = path_msg.add_poses();
    path_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
    path_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    path_msg.mutable_header()->set_frameid("map");
    pose->mutable_header()->mutable_timestamp()->set_sec(sec);
    pose->mutable_header()->mutable_timestamp()->set_nsec(nsec);
    pose->mutable_header()->set_frameid("map");
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
    RVIZ_AGENT.Publish(topic, path_msg);
  }

  static void PubPerception(const Eigen::Affine3d& T_W_V,
                            const TrackingManager& perception, uint64_t sec,
                            uint64_t nsec, const std::string& topic) {
    static bool register_flag = true;
    if (register_flag) {
      RVIZ_AGENT.Register<adsfi_proto::viz::PointCloud>(topic);
      register_flag = false;
    }
    adsfi_proto::viz::PointCloud points_msg;
    points_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
    points_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    points_msg.mutable_header()->set_frameid("map");
    auto* channels = points_msg.add_channels();
    channels->set_name("rgb");
    for (const auto& lane_line : perception.lane_lines) {
      for (const auto& element : lane_line.second.points) {
        Eigen::Vector3d point_vehicle{element.x, element.y, element.z};
        auto point_world = T_W_V * point_vehicle;
        auto* point_msg = points_msg.add_points();
        point_msg->set_x(static_cast<float>(point_world.x()));
        point_msg->set_y(static_cast<float>(point_world.y()));
        point_msg->set_z(static_cast<float>(point_world.z()));
      }
    }
    RVIZ_AGENT.Publish(topic, points_msg);
  }

  static void PubPerceptionMarker(const Eigen::Affine3d& T_W_V,
                                  const TrackingManager& perception,
                                  uint64_t sec, uint64_t nsec,
                                  const std::string& topic) {
    static bool register_flag = true;
    if (register_flag) {
      RVIZ_AGENT.Register<adsfi_proto::viz::MarkerArray>(topic);
      register_flag = false;
    }
    adsfi_proto::viz::MarkerArray markers;
    int id = 0;
    for (const auto& lane_line : perception.lane_lines) {
      if (lane_line.second.points.empty()) {
        continue;
      }
      adsfi_proto::viz::Marker point_marker;
      point_marker.mutable_header()->set_frameid("map");
      point_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
      point_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
      point_marker.set_id(id++);
      point_marker.set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
      point_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
      point_marker.mutable_pose()->mutable_orientation()->set_x(0.);
      point_marker.mutable_pose()->mutable_orientation()->set_y(0.);
      point_marker.mutable_pose()->mutable_orientation()->set_z(0.);
      point_marker.mutable_pose()->mutable_orientation()->set_w(1.);
      point_marker.mutable_scale()->set_x(0.4);
      point_marker.mutable_scale()->set_y(0.4);
      point_marker.mutable_scale()->set_z(0.4);
      point_marker.mutable_lifetime()->set_sec(0);
      point_marker.mutable_lifetime()->set_nsec(200000000);
      point_marker.mutable_color()->set_a(0.5);
      point_marker.mutable_color()->set_r(1);
      point_marker.mutable_color()->set_g(0);
      point_marker.mutable_color()->set_b(0);
      for (const auto& element : lane_line.second.points) {
        Eigen::Vector3d point_vehicle{element.x, element.y, element.z};
        auto point_world = T_W_V * point_vehicle;
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
      txt_marker.mutable_header()->set_frameid("map");
      txt_marker.mutable_pose()->mutable_orientation()->set_x(0.);
      txt_marker.mutable_pose()->mutable_orientation()->set_y(0.);
      txt_marker.mutable_pose()->mutable_orientation()->set_z(0.);
      txt_marker.mutable_pose()->mutable_orientation()->set_w(1.);
      Eigen::Vector3d point_vehicle{lane_line.second.points[0].x,
                                    lane_line.second.points[0].y,
                                    lane_line.second.points[0].z};
      auto point_world = T_W_V * point_vehicle;
      txt_marker.mutable_pose()->mutable_position()->set_x(point_world.x());
      txt_marker.mutable_pose()->mutable_position()->set_y(point_world.y());
      txt_marker.mutable_pose()->mutable_position()->set_z(2);
      txt_marker.mutable_color()->set_r(1);
      txt_marker.mutable_color()->set_g(0);
      txt_marker.mutable_color()->set_b(0);
      txt_marker.mutable_color()->set_a(1);
      std::string txt;
      switch (lane_line.second.lane_type) {
        case UNKNOWN:
          txt = "unknown ";
          break;
        case SOLID_LINE:
          txt = "solid_line ";
          break;
        case DASHED_LINE:
          txt = "dashed_line ";
          break;
        case Road_Edge:
          txt = "road_edge ";
          break;
      }
      txt_marker.set_text(txt + std::to_string(lane_line.first));
      txt_marker.mutable_scale()->set_x(0.5);
      txt_marker.mutable_scale()->set_y(0.5);
      txt_marker.mutable_scale()->set_z(0.5);
      markers.add_markers()->CopyFrom(txt_marker);
    }
    RVIZ_AGENT.Publish(topic, markers);
  }

  static void PubPerceptionMarkerReloc(const Eigen::Affine3d& T_W_V,
                                       const TrackingManager& perception,
                                       uint64_t sec, uint64_t nsec,
                                       const std::string& topic) {
    static bool register_flag = true;
    if (register_flag) {
      RVIZ_AGENT.Register<adsfi_proto::viz::MarkerArray>(topic);
      register_flag = false;
    }
    adsfi_proto::viz::MarkerArray markers;
    int id = 0;
    for (const auto& lane_line : perception.lane_lines) {
      if (lane_line.second.points.empty()) {
        continue;
      }
      adsfi_proto::viz::Marker point_marker;
      point_marker.mutable_header()->set_frameid("map");
      point_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
      point_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
      point_marker.set_id(id++);
      point_marker.set_type(adsfi_proto::viz::MarkerType::LINE_STRIP);
      point_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
      point_marker.mutable_pose()->mutable_orientation()->set_x(0.);
      point_marker.mutable_pose()->mutable_orientation()->set_y(0.);
      point_marker.mutable_pose()->mutable_orientation()->set_z(0.);
      point_marker.mutable_pose()->mutable_orientation()->set_w(1.);
      point_marker.mutable_scale()->set_x(0.4);
      point_marker.mutable_scale()->set_y(0.4);
      point_marker.mutable_scale()->set_z(0.4);
      point_marker.mutable_lifetime()->set_sec(0);
      point_marker.mutable_lifetime()->set_nsec(200000000);
      point_marker.mutable_color()->set_a(0.5);
      point_marker.mutable_color()->set_r(1);
      point_marker.mutable_color()->set_g(1);
      point_marker.mutable_color()->set_b(0);
      for (const auto& element : lane_line.second.points) {
        Eigen::Vector3d point_vehicle{element.x, element.y, element.z};
        auto point_world = T_W_V * point_vehicle;
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
      txt_marker.mutable_header()->set_frameid("map");
      txt_marker.mutable_pose()->mutable_orientation()->set_x(0.);
      txt_marker.mutable_pose()->mutable_orientation()->set_y(0.);
      txt_marker.mutable_pose()->mutable_orientation()->set_z(0.);
      txt_marker.mutable_pose()->mutable_orientation()->set_w(1.);
      Eigen::Vector3d point_vehicle{lane_line.second.points[0].x,
                                    lane_line.second.points[0].y,
                                    lane_line.second.points[0].z};
      auto point_world = T_W_V * point_vehicle;
      txt_marker.mutable_pose()->mutable_position()->set_x(point_world.x());
      txt_marker.mutable_pose()->mutable_position()->set_y(point_world.y());
      txt_marker.mutable_pose()->mutable_position()->set_z(2);
      txt_marker.mutable_color()->set_r(1);
      txt_marker.mutable_color()->set_g(1);
      txt_marker.mutable_color()->set_b(0);
      txt_marker.mutable_color()->set_a(1);
      std::string txt;
      switch (lane_line.second.lane_type) {
        case UNKNOWN:
          txt = "unknown ";
          break;
        case SOLID_LINE:
          txt = "solid_line ";
          break;
        case DASHED_LINE:
          txt = "dashed_line ";
          break;
        case Road_Edge:
          txt = "road_edge ";
          break;
      }
      txt_marker.set_text(txt + std::to_string(lane_line.first));
      txt_marker.mutable_scale()->set_x(0.5);
      txt_marker.mutable_scale()->set_y(0.5);
      txt_marker.mutable_scale()->set_z(0.5);
      markers.add_markers()->CopyFrom(txt_marker);
    }
    RVIZ_AGENT.Publish(topic, markers);
  }

  static void PubHdmap(const Eigen::Affine3d& T_W_V,
                       const MappingManager& hdmap, uint64_t sec, uint64_t nsec,
                       const std::string& topic) {
    static bool register_flag = true;
    if (register_flag) {
      RVIZ_AGENT.Register<adsfi_proto::viz::PointCloud>(topic);
      register_flag = false;
    }
    adsfi_proto::viz::PointCloud points_msg;
    points_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
    points_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    points_msg.mutable_header()->set_frameid("map");
    auto* channels = points_msg.add_channels();
    channels->set_name("rgb");
    for (const auto& lane_line : hdmap.lane_lines) {
      for (const auto& element : lane_line.second.points) {
        Eigen::Vector3d point_vehicle{element.x, element.y, element.z};
        auto point_world = T_W_V * point_vehicle;
        auto* point_msg = points_msg.add_points();
        point_msg->set_x(static_cast<float>(point_world.x()));
        point_msg->set_y(static_cast<float>(point_world.y()));
        point_msg->set_z(static_cast<float>(point_world.z()));
      }
    }
    RVIZ_AGENT.Publish(topic, points_msg);
  }

  static void PubHdmapMarker(const Eigen::Affine3d& T_W_V,
                             const MappingManager& hdmap, uint64_t sec,
                             uint64_t nsec, const std::string& topic) {
    static bool register_flag = true;
    if (register_flag) {
      RVIZ_AGENT.Register<adsfi_proto::viz::MarkerArray>(topic);
      register_flag = false;
    }
    adsfi_proto::viz::MarkerArray markers;
    int id = 0;
    for (const auto& lane_line : hdmap.lane_lines) {
      if (lane_line.second.points.empty()) {
        continue;
      }
      adsfi_proto::viz::Marker point_marker;
      point_marker.mutable_header()->set_frameid("map");
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
      point_marker.mutable_color()->set_r(1);
      point_marker.mutable_color()->set_g(1);
      point_marker.mutable_color()->set_b(1);
      for (const auto& element : lane_line.second.points) {
        Eigen::Vector3d point_vehicle{element.x, element.y, element.z};
        auto point_world = T_W_V * point_vehicle;
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
      txt_marker.mutable_header()->set_frameid("map");
      txt_marker.mutable_pose()->mutable_orientation()->set_x(0.);
      txt_marker.mutable_pose()->mutable_orientation()->set_y(0.);
      txt_marker.mutable_pose()->mutable_orientation()->set_z(0.);
      txt_marker.mutable_pose()->mutable_orientation()->set_w(1.);
      Eigen::Vector3d point_vehicle{lane_line.second.points[0].x,
                                    lane_line.second.points[0].y,
                                    lane_line.second.points[0].z};
      auto point_world = T_W_V * point_vehicle;
      txt_marker.mutable_pose()->mutable_position()->set_x(point_world.x());
      txt_marker.mutable_pose()->mutable_position()->set_y(point_world.y());
      txt_marker.mutable_pose()->mutable_position()->set_z(2);
      txt_marker.mutable_color()->set_r(0);
      txt_marker.mutable_color()->set_g(1);
      txt_marker.mutable_color()->set_b(0);
      txt_marker.mutable_color()->set_a(1);
      std::string txt;
      switch (lane_line.second.lane_type) {
        case UNKNOWN:
          txt = "unknown ";
          break;
        case SOLID_LINE:
          txt = "solid_line ";
          break;
        case DASHED_LINE:
          txt = "dashed_line ";
          break;
        case Road_Edge:
          txt = "road_edge ";
          break;
      }
      txt_marker.set_text(txt + std::to_string(lane_line.first));
      txt_marker.mutable_scale()->set_x(0.5);
      txt_marker.mutable_scale()->set_y(0.5);
      txt_marker.mutable_scale()->set_z(0.5);
      markers.add_markers()->CopyFrom(txt_marker);
    }
    RVIZ_AGENT.Publish(topic, markers);
  }

  static void PubInsLocationState(const Eigen::Affine3d& T_W_V, int ins_state,
                                  double sd_position, int location_state,
                                  double velocity, uint64_t sec, uint64_t nsec,
                                  const std::string& topic) {
    static bool register_flag = true;
    if (register_flag) {
      RVIZ_AGENT.Register<adsfi_proto::viz::Marker>(topic);
      register_flag = false;
    }
    adsfi_proto::viz::Marker text_marker;
    text_marker.set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
    text_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
    text_marker.set_id(0);
    text_marker.mutable_lifetime()->set_sec(0);
    text_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
    text_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    text_marker.mutable_header()->set_frameid("map");
    text_marker.mutable_pose()->mutable_position()->set_x(
        T_W_V.translation().x() - 5);
    text_marker.mutable_pose()->mutable_position()->set_y(
        T_W_V.translation().y());
    text_marker.mutable_pose()->mutable_position()->set_z(
        T_W_V.translation().z());
    text_marker.mutable_pose()->mutable_orientation()->set_x(0);
    text_marker.mutable_pose()->mutable_orientation()->set_y(0);
    text_marker.mutable_pose()->mutable_orientation()->set_z(0);
    text_marker.mutable_pose()->mutable_orientation()->set_w(1);
    text_marker.mutable_color()->set_r(1);
    text_marker.mutable_color()->set_g(1);
    text_marker.mutable_color()->set_b(1);
    text_marker.mutable_color()->set_a(1);
    text_marker.set_text(
        "location_state: " + std::to_string(location_state) +
        "\nins_state: " + std::to_string(ins_state) +
        "\nsd_position: " + std::to_string(sd_position) + "\ntimestamp: " +
        std::to_string(static_cast<double>(sec) +
                       static_cast<double>(nsec) * 1e-9) +
        "\nvelocity: " + std::to_string(velocity * 3.6) + "km/h");
    text_marker.mutable_scale()->set_x(0.1);
    text_marker.mutable_scale()->set_y(0);
    text_marker.mutable_scale()->set_z(0.8);
    hozon::mp::util::RvizAgent::Instance().Publish(topic, text_marker);
  }
};
}  // namespace pe
}  // namespace loc
}  // namespace mp
}  // namespace hozon
