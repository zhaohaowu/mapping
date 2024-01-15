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
                          std::vector<double>* c) {
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
    c->at(0) = x[0];
    c->at(1) = x[1];
    c->at(2) = x[2];
    c->at(3) = x[3];
  }

  static double CalCubicCurveY(const LaneLine& laneline, const double& x) {
    double y = laneline.c0_ + laneline.c1_ * x + laneline.c2_ * x * x +
               laneline.c3_ * x * x * x;
    return y;
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

  static bool IsConvex(const std::vector<Eigen::Vector3d>& points) {
    if (points.size() != 4) {
      return false;
    }
    auto IsOutsideTriangle =
        [](const Eigen::Vector3d& P, const Eigen::Vector3d& A,
           const Eigen::Vector3d& B, const Eigen::Vector3d& C) {
          Eigen::Vector2d AP = {P.x() - A.x(), P.y() - A.y()};
          Eigen::Vector2d BP = {P.x() - B.x(), P.y() - B.y()};
          Eigen::Vector2d CP = {P.x() - C.x(), P.y() - C.y()};
          Eigen::Vector2d AB = {B.x() - A.x(), B.y() - A.y()};
          Eigen::Vector2d BC = {C.x() - B.x(), C.y() - B.y()};
          Eigen::Vector2d CA = {A.x() - C.x(), A.y() - C.y()};
          double S1 = AP.x() * AB.y() - AP.y() * AB.x();
          double S2 = BP.x() * BC.y() - BP.y() * BC.x();
          double S3 = CP.x() * CA.y() - CP.y() * CA.x();
          return (S1 < 0 || S2 < 0 || S3 < 0) && (S1 > 0 || S2 > 0 || S3 > 0);
        };
    bool P0 = IsOutsideTriangle(points[0], points[1], points[2], points[3]);
    bool P1 = IsOutsideTriangle(points[1], points[0], points[2], points[3]);
    bool P2 = IsOutsideTriangle(points[2], points[0], points[1], points[3]);
    bool P3 = IsOutsideTriangle(points[3], points[0], points[1], points[2]);
    return P0 && P1 && P2 && P3;
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

  static void FitLaneLines(std::vector<LaneLine>* map_lane_lines) {
    for (auto& lane_line : *map_lane_lines) {
      if (!lane_line.ismature_) {
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
      if (fabs(lane_line.c2_) < 0.001) {
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

  static void FitEdgeLines(std::vector<LaneLine>* map_edge_lines) {
    for (auto& lane_line : *map_edge_lines) {
      if (!lane_line.ismature_) {
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
      if (fabs(lane_line.c2_) < 0.001) {
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

  static void PubPerEdgeLine(const Sophus::SE3d& T_W_V,
                             const std::vector<LaneLine>& per_edge_lines,
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
    for (const auto& edge_line : per_edge_lines) {
      for (const auto& point : edge_line.points_) {
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
      Eigen::Vector3d point_0(arrow.min_x, arrow.min_y, 0);
      point_0 = T_W_V * point_0;
      Eigen::Vector3d point_1(arrow.max_x, arrow.min_y, 0);
      point_1 = T_W_V * point_1;
      Eigen::Vector3d point_2(arrow.max_x, arrow.max_y, 0);
      point_2 = T_W_V * point_2;
      Eigen::Vector3d point_3(arrow.min_x, arrow.max_y, 0);
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
      Eigen::Vector3d point_0(zebra_crossing.min_x, zebra_crossing.min_y, 0);
      point_0 = T_W_V * point_0;
      Eigen::Vector3d point_1(zebra_crossing.max_x, zebra_crossing.min_y, 0);
      point_1 = T_W_V * point_1;
      Eigen::Vector3d point_2(zebra_crossing.max_x, zebra_crossing.max_y, 0);
      point_2 = T_W_V * point_2;
      Eigen::Vector3d point_3(zebra_crossing.min_x, zebra_crossing.max_y, 0);
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

  static void PubMapEdgeLine(const Sophus::SE3d& T_W_V,
                             const std::vector<LaneLine>& map_edge_lines,
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
    for (const auto& edge_line : map_edge_lines) {
      if (!edge_line.ismature_) {
        continue;
      }
      for (const auto& point : edge_line.fit_points_) {
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
      if (!arrow.ismature_ || !CommonUtil::IsConvex(arrow.points_.points_)) {
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
      Eigen::Vector3d point_0(arrow.min_x, arrow.min_y, 0);
      point_0 = T_W_V * arrow.points_.points_[0];
      Eigen::Vector3d point_1(arrow.max_x, arrow.min_y, 0);
      point_1 = T_W_V * arrow.points_.points_[1];
      Eigen::Vector3d point_2(arrow.max_x, arrow.max_y, 0);
      point_2 = T_W_V * arrow.points_.points_[2];
      Eigen::Vector3d point_3(arrow.min_x, arrow.max_y, 0);
      point_3 = T_W_V * arrow.points_.points_[3];
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
      if (!zebra_crossing.ismature_ ||
          !CommonUtil::IsConvex(zebra_crossing.points_.points_)) {
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
      Eigen::Vector3d point_0(zebra_crossing.min_x, zebra_crossing.min_y, 0);
      point_0 = T_W_V * zebra_crossing.points_.points_[0];
      Eigen::Vector3d point_1(zebra_crossing.max_x, zebra_crossing.min_y, 0);
      point_1 = T_W_V * zebra_crossing.points_.points_[1];
      Eigen::Vector3d point_2(zebra_crossing.max_x, zebra_crossing.max_y, 0);
      point_2 = T_W_V * zebra_crossing.points_.points_[2];
      Eigen::Vector3d point_3(zebra_crossing.min_x, zebra_crossing.max_y, 0);
      point_3 = T_W_V * zebra_crossing.points_.points_[3];
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
      if (!lane_line.ismature_) {
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
      Eigen::Vector3d left_point = {8, lane_line.c0_, 0};
      left_point = T_W_V * left_point;
      Eigen::Vector3d right_point = {4, lane_line.c0_, 0};
      right_point = T_W_V * right_point;
      txt_marker.mutable_pose()->mutable_position()->set_x(
          (left_point.x() + right_point.x()) / 2);
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
      if (lane_line.ismature_) {
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
      Eigen::Vector3d left_point = {8, lane_line.c0_, 0};
      left_point = T_W_V * left_point;
      Eigen::Vector3d right_point = {4, lane_line.c0_, 0};
      right_point = T_W_V * right_point;
      txt_marker.mutable_pose()->mutable_position()->set_x(
          (left_point.x() + right_point.x()) / 2);
      txt_marker.mutable_pose()->mutable_position()->set_y(
          (left_point.y() + right_point.y()) / 2);
      txt_marker.mutable_pose()->mutable_position()->set_z(2);
      txt_marker.mutable_pose()->mutable_orientation()->set_x(0);
      txt_marker.mutable_pose()->mutable_orientation()->set_y(0);
      txt_marker.mutable_pose()->mutable_orientation()->set_z(0);
      txt_marker.mutable_pose()->mutable_orientation()->set_w(1);
      txt_marker.mutable_color()->set_r(0.32);
      txt_marker.mutable_color()->set_g(0.29);
      txt_marker.mutable_color()->set_b(0.39);
      txt_marker.mutable_color()->set_a(1);
      txt_marker.set_text(std::to_string(lane_line.count_));
      txt_marker.mutable_scale()->set_x(1);
      txt_marker.mutable_scale()->set_y(1);
      txt_marker.mutable_scale()->set_z(1);
      markers.add_markers()->CopyFrom(txt_marker);
    }
    util::RvizAgent::Instance().Publish(topic, markers);
  }

  static void PubMapLane(const Sophus::SE3d& T_W_V, const LocalMap& local_map,
                         const uint64_t& sec, const uint64_t& nsec,
                         const std::string& topic) {
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
      Eigen::Vector3d left_point = {2, lane.left_line_.c0_, 0};
      left_point = T_W_V * left_point;
      Eigen::Vector3d right_point = {2, lane.right_line_.c0_, 0};
      right_point = T_W_V * right_point;
      txt_marker.mutable_pose()->mutable_position()->set_x(
          (left_point.x() + right_point.x()) / 2);
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
