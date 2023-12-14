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
#include "modules/local_mapping/utils/map_manager.h"
#include "modules/map_fusion/include/map_fusion/map_service/global_hd_map.h"
#include "modules/util/include/util/geo.h"
#include "modules/util/include/util/mapping_log.h"
#include "modules/util/include/util/rviz_agent/rviz_agent.h"
namespace hozon {
namespace mp {
namespace lm {

class CommonUtil {
 public:
  static void FitLaneLine(const std::vector<Eigen::Vector3d>& pts,
                          LaneLine* lane_line) {
    int n = static_cast<int>(pts.size());
    Eigen::Matrix<double, Eigen::Dynamic, 4> A(n, 4);
    Eigen::VectorXd x(n);
    Eigen::VectorXd b(n);
    for (int i = 0; i < n; i++) {
      double xi = pts[i][0];
      double yi = pts[i][1];
      A(i, 0) = 1.0;
      A(i, 1) = xi;
      A(i, 2) = xi * xi;
      A(i, 3) = xi * xi * xi;
      b[i] = yi;
    }
    x = (A.transpose() * A).inverse() * A.transpose() * b;
    lane_line->c0_ = x[0];
    lane_line->c1_ = x[1];
    lane_line->c2_ = x[2];
    lane_line->c3_ = x[3];
  }

  static void EraseErrorPts(std::vector<Eigen::Vector3d>* pts) {
    for (int i = 0; i < static_cast<int>(pts->size() - 2); i++) {
      if ((pts->at(i + 1).y() - pts->at(i).y() > 1 &&
           pts->at(i + 2).y() - pts->at(i + 1).y() < -1) ||
          (pts->at(i + 1).y() - pts->at(i).y() < -1 &&
           pts->at(i + 2).y() - pts->at(i + 1).y() > 1)) {
        pts->erase(pts->begin() + i + 1);
        i--;
      }
    }
  }
  static void CatmullRom(const std::vector<Eigen::Vector3d>& pts,
                         LaneLine* lane_line) {
    auto func = [](double p0, double p1, double p2, double p3, double t) {
      double s = 0.5;
      double a = -s * p0 + (2 - s) * p1 + (s - 2) * p2 + s * p3;
      double b = 2 * s * p0 + (s - 3) * p1 + (3 - 2 * s) * p2 - s * p3;
      double c = -s * p0 + s * p2;
      double d = p1;
      double t2 = t * t;
      double t3 = t2 * t;
      return (a * t3 + b * t2 + c * t + d);
    };
    for (size_t i = 1; i < pts.size() - 2; ++i) {
      for (int t = 0; t < 10; t++) {
        double px = func(pts[i - 1].x(), pts[i].x(), pts[i + 1].x(),
                         pts[i + 2].x(), t * 0.1);
        double py = func(pts[i - 1].y(), pts[i].y(), pts[i + 1].y(),
                         pts[i + 2].y(), t * 0.1);
        Eigen::Vector3d point = {px, py, 0.0};
        lane_line->fit_points_.emplace_back(point);
      }
    }
  }

  static void FitLocalMap(LocalMap* local_map) {
    for (auto& lane_line : local_map->lane_lines_) {
      if (!lane_line.need_fit_ || !lane_line.ismature_) {
        continue;
      }
      lane_line.fit_points_.clear();
      lane_line.control_points_.clear();
      int n = static_cast<int>(lane_line.points_.size());
      if (n < 10) {
        continue;
      }
      if (n < 21) {
        lane_line.fit_points_ = lane_line.points_;
        continue;
      }
      std::vector<Eigen::Vector3d> pts;
      std::vector<Eigen::Vector3d> back_pts;
      for (int i = 0; i < n; i++) {
        if (i == 1 || i % 10 == 0 || i == n - 1) {
          pts.push_back(lane_line.points_[i]);
        }
        if (((n - 1) % 10 == 0 && i >= ((n - 1) / 10 - 1) * 10) ||
            ((n - 1) % 10 != 0 && i >= (n - 1) / 10 * 10)) {
          back_pts.push_back(lane_line.points_[i]);
        }
      }
      if (lane_line.c2_ < 0.001) {
        CommonUtil::EraseErrorPts(&pts);
      }
      if (pts.size() < 4) {
        continue;
      }
      lane_line.control_points_ = pts;
      CatmullRom(pts, &lane_line);
      for (const auto& point : back_pts) {
        lane_line.fit_points_.push_back(point);
      }
    }
  }

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

  static void PubPercepPoints(const Sophus::SE3d& T_W_V,
                              const Perception& cur_lane_lines,
                              const uint64_t& sec, const uint64_t& nsec,
                              const std::string& topic) {
    static bool percep_flag = true;
    if (percep_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::PointCloud>(topic);
      percep_flag = false;
    }
    adsfi_proto::viz::PointCloud percep_points_msg;
    percep_points_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
    percep_points_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    percep_points_msg.mutable_header()->set_frameid("localmap");
    auto* channels = percep_points_msg.add_channels();
    channels->set_name("rgb");
    for (const auto& lane_line : cur_lane_lines.lane_lines_) {
      for (auto point : lane_line.points_) {
        point = T_W_V * point;
        auto* percep_point = percep_points_msg.add_points();
        percep_point->set_x(static_cast<float>(point.x()));
        percep_point->set_y(static_cast<float>(point.y()));
        percep_point->set_z(static_cast<float>(point.z()));
      }
    }
    util::RvizAgent::Instance().Publish(topic, percep_points_msg);
  }

  static void PubPercepPointsMarker(const Sophus::SE3d& T_W_V,
                                    const Perception& cur_lane_lines,
                                    const uint64_t& sec, const uint64_t& nsec,
                                    const std::string& topic) {
    static bool percep_marker_flag = true;
    if (percep_marker_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::MarkerArray>(
          topic);
      percep_marker_flag = false;
    }
    adsfi_proto::viz::MarkerArray markers;
    int id = 0;
    for (size_t i = 0; i < cur_lane_lines.lane_lines_.size(); i++) {
      if (cur_lane_lines.lane_lines_[i].points_.empty()) {
        continue;
      }
      auto start_point = cur_lane_lines.lane_lines_[i].points_.front();
      start_point = T_W_V * start_point;
      adsfi_proto::viz::Marker lane_index;
      lane_index.set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
      lane_index.set_action(adsfi_proto::viz::MarkerAction::ADD);
      lane_index.set_id(id++);
      lane_index.mutable_lifetime()->set_sec(0);
      lane_index.mutable_lifetime()->set_nsec(200000000);
      lane_index.mutable_header()->mutable_timestamp()->set_sec(sec);
      lane_index.mutable_header()->mutable_timestamp()->set_nsec(nsec);
      lane_index.mutable_header()->set_frameid("localmap");
      lane_index.mutable_pose()->mutable_position()->set_x(start_point.x());
      lane_index.mutable_pose()->mutable_position()->set_y(start_point.y());
      lane_index.mutable_pose()->mutable_position()->set_z(2);
      lane_index.mutable_pose()->mutable_orientation()->set_x(0);
      lane_index.mutable_pose()->mutable_orientation()->set_y(0);
      lane_index.mutable_pose()->mutable_orientation()->set_z(0);
      lane_index.mutable_pose()->mutable_orientation()->set_w(1);
      lane_index.mutable_color()->set_r(1);
      lane_index.mutable_color()->set_g(0);
      lane_index.mutable_color()->set_b(0);
      lane_index.mutable_color()->set_a(1);
      lane_index.set_text(std::to_string(i));
      lane_index.mutable_scale()->set_x(1);
      lane_index.mutable_scale()->set_y(1);
      lane_index.mutable_scale()->set_z(1);
      markers.add_markers()->CopyFrom(lane_index);
    }
    util::RvizAgent::Instance().Publish(topic, markers);
  }

  static void PubMapPoints(const LocalMap& local_map, const uint64_t& sec,
                           const uint64_t& nsec, const std::string& topic,
                           const Sophus::SE3d& T_W_V) {
    static bool map_flag = true;
    if (map_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::PointCloud>(topic);
      map_flag = false;
    }
    adsfi_proto::viz::PointCloud map_points_msg;
    map_points_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
    map_points_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    map_points_msg.mutable_header()->set_frameid("localmap");
    auto* channels = map_points_msg.add_channels();
    channels->set_name("rgb");
    for (const auto& lane_line : local_map.lane_lines_) {
      for (auto point : lane_line.fit_points_) {
        point = T_W_V * point;
        auto* map_point = map_points_msg.add_points();
        map_point->set_x(static_cast<float>(point.x()));
        map_point->set_y(static_cast<float>(point.y()));
        map_point->set_z(static_cast<float>(point.z()));
      }
    }
    util::RvizAgent::Instance().Publish(topic, map_points_msg);
  }

  static void PubMapControlPoints(const LocalMap& local_map,
                                  const uint64_t& sec, const uint64_t& nsec,
                                  const std::string& topic,
                                  const Sophus::SE3d& T_W_V) {
    static bool map_control_flag = true;
    if (map_control_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::PointCloud>(topic);
      map_control_flag = false;
    }
    adsfi_proto::viz::PointCloud map_points_msg;
    map_points_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
    map_points_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    map_points_msg.mutable_header()->set_frameid("localmap");
    auto* channels = map_points_msg.add_channels();
    channels->set_name("rgb");
    for (const auto& lane_line : local_map.lane_lines_) {
      for (auto point : lane_line.control_points_) {
        point = T_W_V * point;
        auto* map_point = map_points_msg.add_points();
        map_point->set_x(static_cast<float>(point.x()));
        map_point->set_y(static_cast<float>(point.y()));
        map_point->set_z(static_cast<float>(point.z()));
      }
    }
    util::RvizAgent::Instance().Publish(topic, map_points_msg);
  }

  static void PubMapPointsMarker(const LocalMap& local_map, const uint64_t& sec,
                                 const uint64_t& nsec, const std::string& topic,
                                 const Sophus::SE3d& T_W_V) {
    static bool map_marker_flag = true;
    if (map_marker_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::MarkerArray>(
          topic);
      map_marker_flag = false;
    }
    adsfi_proto::viz::MarkerArray markers;
    int id = 0;
    for (size_t i = 0; i < local_map.lane_lines_.size(); i++) {
      if (local_map.lane_lines_[i].fit_points_.empty()) {
        continue;
      }
      auto end_points = T_W_V * local_map.lane_lines_[i].fit_points_.back();
      adsfi_proto::viz::Marker point_marker;
      point_marker.mutable_header()->set_frameid("localmap");
      point_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
      point_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
      // point_marker.set_ns("ns_local_map_lane");
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
      color.set_r(0.0);
      color.set_g(1.0);
      color.set_b(0.0);
      point_marker.mutable_color()->CopyFrom(color);
      for (auto point : local_map.lane_lines_[i].fit_points_) {
        point = T_W_V * point;
        auto* pt = point_marker.add_points();
        pt->set_x(point.x());
        pt->set_y(point.y());
        pt->set_z(point.z());
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
      txt_marker.mutable_pose()->mutable_position()->set_x(end_points.x());
      txt_marker.mutable_pose()->mutable_position()->set_y(end_points.y());
      txt_marker.mutable_pose()->mutable_position()->set_z(2);
      txt_marker.mutable_pose()->mutable_orientation()->set_x(0);
      txt_marker.mutable_pose()->mutable_orientation()->set_y(0);
      txt_marker.mutable_pose()->mutable_orientation()->set_z(0);
      txt_marker.mutable_pose()->mutable_orientation()->set_w(1);
      txt_marker.mutable_color()->set_r(0);
      txt_marker.mutable_color()->set_g(1);
      txt_marker.mutable_color()->set_b(0);
      txt_marker.mutable_color()->set_a(1);
      txt_marker.set_text(
          std::to_string(local_map.lane_lines_[i].track_id_) + ", " +
          std::to_string(local_map.lane_lines_[i].lanepos_) + ", " +
          std::to_string(local_map.lane_lines_[i].lanetype_) + ", " +
          std::to_string(local_map.lane_lines_[i].edge_laneline_count_));
      txt_marker.mutable_scale()->set_x(1);
      txt_marker.mutable_scale()->set_y(1);
      txt_marker.mutable_scale()->set_z(1);
      markers.add_markers()->CopyFrom(txt_marker);
    }
    util::RvizAgent::Instance().Publish(topic, markers);
  }

  static void PubOriMapPoints(const LocalMap& local_map, const uint64_t& sec,
                              const uint64_t& nsec, const std::string& topic,
                              const Sophus::SE3d& T_W_V) {
    static bool ori_map_flag = true;
    if (ori_map_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::PointCloud>(topic);
      ori_map_flag = false;
    }
    adsfi_proto::viz::PointCloud map_points_msg;
    map_points_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
    map_points_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    map_points_msg.mutable_header()->set_frameid("localmap");
    auto* channels = map_points_msg.add_channels();
    channels->set_name("rgb");
    for (const auto& lane_line : local_map.lane_lines_) {
      for (auto point : lane_line.points_) {
        point = T_W_V * point;
        auto* map_point = map_points_msg.add_points();
        map_point->set_x(static_cast<float>(point.x()));
        map_point->set_y(static_cast<float>(point.y()));
        map_point->set_z(static_cast<float>(point.z()));
      }
    }
    util::RvizAgent::Instance().Publish(topic, map_points_msg);
  }

  static void PubLane(const LocalMap& local_map, const uint64_t& sec,
                      const uint64_t& nsec, const std::string& topic,
                      const Sophus::SE3d& T_W_V) {
    static bool map_lane_flag = true;
    if (map_lane_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::MarkerArray>(
          topic);
      map_lane_flag = false;
    }
    adsfi_proto::viz::MarkerArray markers;
    int id = 0;
    for (auto lane : local_map.lanes_) {
      adsfi_proto::viz::Marker txt_marker;
      txt_marker.set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
      txt_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
      txt_marker.set_id(id++);
      txt_marker.mutable_lifetime()->set_sec(0);
      txt_marker.mutable_lifetime()->set_nsec(200000000);
      txt_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
      txt_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
      txt_marker.mutable_header()->set_frameid("localmap");
      if (lane.left_line_.points_.empty()) {
        continue;
      }
      auto left_point = lane.left_line_.points_[0];
      left_point = T_W_V * left_point;
      if (lane.right_line_.points_.empty()) {
        continue;
      }
      auto right_point = lane.right_line_.points_[0];
      right_point = T_W_V * right_point;
      txt_marker.mutable_pose()->mutable_position()->set_x(left_point.x());
      txt_marker.mutable_pose()->mutable_position()->set_y(
          (left_point.y() + right_point.y()) / 2);
      txt_marker.mutable_pose()->mutable_position()->set_z(2);
      txt_marker.mutable_pose()->mutable_orientation()->set_x(0);
      txt_marker.mutable_pose()->mutable_orientation()->set_y(0);
      txt_marker.mutable_pose()->mutable_orientation()->set_z(0);
      txt_marker.mutable_pose()->mutable_orientation()->set_w(1);
      txt_marker.mutable_color()->set_r(0);
      txt_marker.mutable_color()->set_g(1);
      txt_marker.mutable_color()->set_b(0);
      txt_marker.mutable_color()->set_a(1);
      txt_marker.set_text("lane" + std::to_string(lane.lane_id_));
      txt_marker.mutable_scale()->set_x(1);
      txt_marker.mutable_scale()->set_y(1);
      txt_marker.mutable_scale()->set_z(1);
      markers.add_markers()->CopyFrom(txt_marker);
    }

    util::RvizAgent::Instance().Publish(topic, markers);
  }

  static void PubMapLane(const LocalMap& local_map, const uint64_t& sec,
                         const uint64_t& nsec, const std::string& topic,
                         const Sophus::SE3d& T_W_V) {
    static bool map_lane_flag = true;
    if (map_lane_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::MarkerArray>(
          topic);
      map_lane_flag = false;
    }
    adsfi_proto::viz::MarkerArray markers;
    int id = 0;
    for (auto lane : local_map.map_lanes_) {
      adsfi_proto::viz::Marker txt_marker;
      txt_marker.set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
      txt_marker.set_action(adsfi_proto::viz::MarkerAction::ADD);
      txt_marker.set_id(id++);
      txt_marker.mutable_lifetime()->set_sec(0);
      txt_marker.mutable_lifetime()->set_nsec(200000000);
      txt_marker.mutable_header()->mutable_timestamp()->set_sec(sec);
      txt_marker.mutable_header()->mutable_timestamp()->set_nsec(nsec);
      txt_marker.mutable_header()->set_frameid("localmap");
      if (lane.left_line_.points_.empty()) {
        continue;
      }
      auto left_point = lane.left_line_.points_[0];
      left_point = T_W_V * left_point;
      if (lane.right_line_.points_.empty()) {
        continue;
      }
      auto right_point = lane.right_line_.points_[0];
      right_point = T_W_V * right_point;
      txt_marker.mutable_pose()->mutable_position()->set_x(left_point.x());
      txt_marker.mutable_pose()->mutable_position()->set_y(
          (left_point.y() + right_point.y()) / 2);
      txt_marker.mutable_pose()->mutable_position()->set_z(2);
      txt_marker.mutable_pose()->mutable_orientation()->set_x(0);
      txt_marker.mutable_pose()->mutable_orientation()->set_y(0);
      txt_marker.mutable_pose()->mutable_orientation()->set_z(0);
      txt_marker.mutable_pose()->mutable_orientation()->set_w(1);
      txt_marker.mutable_color()->set_r(0);
      txt_marker.mutable_color()->set_g(1);
      txt_marker.mutable_color()->set_b(0);
      txt_marker.mutable_color()->set_a(1);
      txt_marker.set_text("lane" + std::to_string(lane.lane_id_));
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

  static void Gcj02ToUtm(const Sophus::SE3d& T_G_V, Sophus::SE3d* T_U_V) {
    Eigen::Vector3d p_U_V = Eigen::Vector3d::Identity();
    util::Geo::LatLonToUtmXy(51, T_G_V.translation().y(),
                             T_G_V.translation().x(), &p_U_V);
    *T_U_V = Sophus::SE3d(T_G_V.so3().unit_quaternion(), p_U_V);
  }

  static Eigen::Vector3d UtmPointToVehicle(const Eigen::Vector3d& point_utm,
                                           const Eigen::Vector3d& ref_point,
                                           const Sophus::SE3d& T_G_V) {
    double x = point_utm.x();
    double y = point_utm.y();
    hozon::common::coordinate_convertor::UTM2GCS(51, &x, &y);
    Eigen::Vector3d point_gcj(y, x, 0);
    Eigen::Vector3d point_enu = util::Geo::Gcj02ToEnu(point_gcj, ref_point);

    Eigen::Vector3d p_enu_vehicle =
        util::Geo::Gcj02ToEnu(T_G_V.translation(), ref_point);
    Eigen::Quaterniond q_enu_vehicle = T_G_V.so3().unit_quaternion();
    Sophus::SE3d T_enu_vehicle = Sophus::SE3d(q_enu_vehicle, p_enu_vehicle);

    return T_enu_vehicle.inverse() * point_enu;
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

  static void PubImage(
      const std::string& topic,
      const std::shared_ptr<const hozon::soc::CompressedImage>& sensor_img) {
    static bool img_marker_flag = true;
    if (img_marker_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::CompressedImage>(
          topic);
      img_marker_flag = false;
    }
    if (sensor_img->format() != "jpeg" && sensor_img->format() != "jpg") {
      HLOG_ERROR << "not support " << sensor_img->format() << ", only support "
                 << "jpeg/jpg";
      return;
    }
    adsfi_proto::viz::CompressedImage img;
    img.mutable_header()->set_seq(sensor_img->header().seq());
    img.mutable_header()->set_frameid(sensor_img->header().frame_id());
    auto sec = static_cast<uint32_t>(sensor_img->header().publish_stamp());
    auto nsec = static_cast<uint32_t>(
        (sensor_img->header().publish_stamp() - sec) * 1e9);
    img.mutable_header()->mutable_timestamp()->set_sec(sec);
    img.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    img.set_format(sensor_img->format());
    for (const auto& b : sensor_img->data()) {
      img.mutable_data()->push_back(b);
    }
    if (img.data().empty()) {
      return;
    }
    util::RvizAgent::Instance().Publish(topic, img);
  }

  static DrData Interpolate(const double& scale, const DrData& start,
                            const DrData& end, const double& timestamp) {
    DrData res;
    res.timestamp = timestamp;
    res.pose = start.pose + (end.pose - start.pose) * scale;
    res.quaternion = start.quaternion.slerp(scale, end.quaternion);

    if (scale >= 0.5) {
      res.local_vel = end.local_vel;
      res.local_omg = end.local_omg;
    } else {
      res.local_vel = start.local_vel;
      res.local_omg = start.local_omg;
    }

    return res;
  }
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
