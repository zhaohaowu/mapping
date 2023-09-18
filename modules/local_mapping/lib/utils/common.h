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
#include "modules/local_mapping/lib/types/common.h"
#include "modules/local_mapping/lib/utils/map_manager.h"
#include "util/rviz_agent/rviz_agent.h"
#include "util/temp_log.h"

namespace hozon {
namespace mp {
namespace lm {

class CommonUtil {
 public:
  /**
   * @brief y = c0 + c1 * x + c2 * x^2 + c3 * x^3
   *
   * @param x
   * @param c0
   * @param c1
   * @param c2
   * @param c3
   * @return double result y
   */
  static double f(const double& x, const double& c0, const double& c1,
                  const double& c2, const double& c3) {
    return c0 + c1 * x + c2 * x * x + c3 * x * x * x;
  }

  /**
   * @brief sample points from lane
   *
   * @param lane target lane
   * @param T_V_W translation from vehicle to world
   * @param pts sample points
   * @return
   */
  static void SampleLanePoints(
      std::shared_ptr<const Lane> lane, const Eigen::Matrix4d T_V_W,
      std::shared_ptr<std::vector<Eigen::Vector3d>> pts) {
    double interval = 0.1;
    double x, y, z;
    for (x = lane->x_start_vrf_; x <= lane->x_end_vrf_; x += interval) {
      y = f(x, lane->lane_fit_d_, lane->lane_fit_c_, lane->lane_fit_b_,
            lane->lane_fit_a_);
      z = 0;
      Eigen::Vector4d pt_v(x, y, z, 1);
      Eigen::Vector4d pt_w = T_V_W * pt_v;
      Eigen::Vector3d pt(pt_w(0), pt_w(1), pt_w(2));
      pts->emplace_back(pt);
    }
  }

  /**
   * @brief sample points from lane
   *
   * @param lane target lane
   * @param pts sample points
   * @return
   */
  static void SampleLanePointsInLocal(const Lane& lane,
                                      const LanePointsPtr pts) {
    double interval = 0.1;
    double x, y, z;
    for (x = lane.x_start_vrf_; x <= lane.x_end_vrf_; x += interval) {
      y = f(x, lane.lane_fit_d_, lane.lane_fit_c_, lane.lane_fit_b_,
            lane.lane_fit_a_);
      Eigen::Vector3d pt(x, y, 0);
      pts->emplace_back(pt);
    }
  }

  static Eigen::Matrix3d Se2Vector2Matrix(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m = Eigen::Matrix3d::Identity();

    m(0, 2) = v.x();
    m(1, 2) = v.y();

    m(0, 0) = cos(v.z());
    m(0, 1) = -sin(v.z());
    m(1, 0) = sin(v.z());
    m(1, 1) = cos(v.z());
    return m;
  }

  static Eigen::Vector3d Get2DPose(ConstDrDataPtr dr_ptr) {
    Eigen::Quaterniond quat(dr_ptr->quaternion.w(), dr_ptr->quaternion.x(),
                            dr_ptr->quaternion.y(), dr_ptr->quaternion.z());
    Eigen::Matrix3d rotation = quat.toRotationMatrix();
    double theta = atan2(rotation(1, 0), rotation(0, 0));
    return Eigen::Vector3d(dr_ptr->pose.x(), dr_ptr->pose.y(), theta);
  }

  /**
   * @brief lane from sample points
   *
   * @param pts target points
   * @param T_V_W translation from vehicle to world
   * @param lane output lane
   * @return
   */
  static void FitLanePoints(
      std::shared_ptr<const std::vector<Eigen::Vector3d>> pts,
      const Eigen::Matrix4d& T_V_W, std::shared_ptr<Lane> lane) {
    size_t n = (*pts).size();
    Eigen::Matrix<double, Eigen::Dynamic, 4> A(n, 4);
    Eigen::VectorXd x(n);
    Eigen::VectorXd b(n);
    for (size_t i = 0; i < n; i++) {
      double xi = (*pts)[i][0];
      double yi = (*pts)[i][1];
      A(i, 0) = xi * xi * xi;
      A(i, 1) = xi * xi;
      A(i, 2) = xi;
      A(i, 3) = 1.0;
      b[i] = yi;
    }
    x = (A.transpose() * A).inverse() * A.transpose() * b;
    lane->lane_fit_a_ = x[0];
    lane->lane_fit_b_ = x[1];
    lane->lane_fit_c_ = x[2];
    lane->lane_fit_d_ = x[3];
    lane->x_start_vrf_ = (*pts)[0][0];
    lane->x_end_vrf_ = (*pts)[n - 1][0];
  }

  /**
   * @brief map_lane from sample points
   *
   * @param pts target points
   * @param lane_param output lane_param
   * @return
   */
  static void FitMapLanePoints(const std::vector<Eigen::Vector3d>& points,
                               std::vector<LaneCubicSpline>* lane_param) {
    lane_param->clear();
    std::vector<Eigen::Vector3d> pts;
    for (size_t i = 0; i < points.size(); i++) {
      if (i % 10 == 0) {
        pts.push_back(points[i]);
      }
    }
    size_t n = pts.size();
    if (n <= 4) return;
    std::vector<double> h(n);
    std::vector<double> alpha(n);
    std::vector<double> l(n);
    std::vector<double> u(n);
    std::vector<double> z(n);
    std::vector<double> c(n + 1);
    std::vector<double> b(n);
    std::vector<double> d(n);
    for (size_t i = 0; i < n - 1; i++) {
      h[i] = pts[i + 1].x() - pts[i].x();
    }
    for (size_t i = 1; i < n - 1; i++) {
      alpha[i] = 3.0 * ((pts[i + 1].y() - pts[i].y()) / h[i]) -
                 3.0 * ((pts[i].y() - pts[i - 1].y()) / h[i - 1]);
    }
    l[0] = 1.0;
    u[0] = 0.0;
    z[0] = 0.0;
    for (size_t i = 1; i < n - 1; i++) {
      l[i] = 2.0 * (pts[i + 1].x() - pts[i - 1].x()) - h[i - 1] * u[i - 1];
      u[i] = h[i] / l[i];
      z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
    }
    l[n - 1] = 1.0;
    z[n - 1] = 0.0;
    c[n - 1] = 0.0;
    for (int j = n - 2; j >= 0; j--) {
      c[j] = z[j] - u[j] * c[j + 1];
    }
    for (size_t i = 0; i < n - 1; i++) {
      d[i] = (c[i + 1] - c[i]) / (3.0 * h[i]);
      b[i] = ((pts[i + 1].y() - pts[i].y()) / h[i]) -
             ((h[i] * (c[i + 1] + 2.0 * c[i])) / 3.0);
    }
    for (size_t i = 0; i < n - 1; i++) {
      LaneCubicSpline tmp;
      tmp.c0_ = pts[i].y();
      tmp.c1_ = b[i];
      tmp.c2_ = c[i];
      tmp.c3_ = d[i];
      tmp.start_point_x_ = pts[i].x();
      tmp.end_point_x_ = pts[i + 1].x();
      lane_param->emplace_back(tmp);
    }
  }

  /**
   * @brief fit lane points to lane_param in local map
   *
   * @param local_map local_map
   * @return
   */
  static void CubicCurve(LocalMap* local_map) {
    for (size_t k = 0; k < local_map->local_map_lane_.size(); k++) {
      local_map->local_map_lane_[k].lane_param_.clear();
      std::vector<Eigen::Vector3d> pts;
      for (size_t i = 0; i < local_map->local_map_lane_[k].points_.size();
           i++) {
        if (i % 10 == 0) {
          pts.push_back(local_map->local_map_lane_[k].points_[i]);
        }
      }
      size_t n = pts.size();
      if (n <= 4) continue;
      std::vector<double> h(n);
      std::vector<double> alpha(n);
      std::vector<double> l(n);
      std::vector<double> u(n);
      std::vector<double> z(n);
      std::vector<double> c(n + 1);
      std::vector<double> b(n);
      std::vector<double> d(n);
      for (size_t i = 0; i < n - 1; i++) {
        h[i] = pts[i + 1].x() - pts[i].x();
      }
      for (size_t i = 1; i < n - 1; i++) {
        alpha[i] = 3.0 * ((pts[i + 1].y() - pts[i].y()) / h[i]) -
                   3.0 * ((pts[i].y() - pts[i - 1].y()) / h[i - 1]);
      }
      l[0] = 1.0;
      u[0] = 0.0;
      z[0] = 0.0;
      for (size_t i = 1; i < n - 1; i++) {
        l[i] = 2.0 * (pts[i + 1].x() - pts[i - 1].x()) - h[i - 1] * u[i - 1];
        u[i] = h[i] / l[i];
        z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
      }
      l[n - 1] = 1.0;
      z[n - 1] = 0.0;
      c[n - 1] = 0.0;
      for (int j = n - 2; j >= 0; j--) {
        c[j] = z[j] - u[j] * c[j + 1];
      }
      for (size_t i = 0; i < n - 1; i++) {
        d[i] = (c[i + 1] - c[i]) / (3.0 * h[i]);
        b[i] = ((pts[i + 1].y() - pts[i].y()) / h[i]) -
               ((h[i] * (c[i + 1] + 2.0 * c[i])) / 3.0);
      }
      for (size_t i = 0; i < n - 1; i++) {
        LaneCubicSpline tmp;
        tmp.c0_ = pts[i].y();
        tmp.c1_ = b[i];
        tmp.c2_ = c[i];
        tmp.c3_ = d[i];
        tmp.start_point_x_ = pts[i].x();
        tmp.end_point_x_ = pts[i + 1].x();
        local_map->local_map_lane_[k].lane_param_.emplace_back(tmp);
      }
    }
  }

  /**
   * @brief find min x of lane
   *
   * @param lane_params lane params
   * @return min x
   */
  static double FindMinX(const std::vector<LaneCubicSpline>& lane_params) {
    double min_x = std::numeric_limits<double>::max();
    for (size_t i = 0; i < lane_params.size(); i++) {
      min_x = std::min(lane_params[i].start_point_x_, min_x);
    }
    return min_x;
  }
  /**
   * @brief convert points from vehicle to world
   *
   * @param pts vehicle points
   * @param T_V_W translation from vehicle to world
   * @param lane world points
   * @return
   */
  static void PointsToWorld(
      std::shared_ptr<const std::vector<Eigen::Vector3d>> pts,
      const Eigen::Matrix4d& T_V_W,
      std::shared_ptr<std::vector<Eigen::Vector3d>> world_pts) {
    for (auto& pt : *pts) {
      Eigen::Vector4d v_pt(pt(0), pt(1), pt(2), 1);
      Eigen::Vector4d w_tmp_pt = T_V_W * v_pt;
      Eigen::Vector3d w_pt(w_tmp_pt(0), w_tmp_pt(1), w_tmp_pt(2));
      world_pts->push_back(w_pt);
    }
  }

  /**
   * @brief pub odom in rviz
   *
   * @param T_V_W translation from vehicle to world
   * @param sec second in timestamp
   * @param nsec nsecond in timestamp
   * @param topic topic
   * @return
   */
  static void PubOdom(const Eigen::Matrix4d& T_V_W, const uint64_t& sec,
                      const uint64_t& nsec, const std::string& topic) {
    Eigen::Vector3d p = T_V_W.block<3, 1>(0, 3);
    Eigen::Quaterniond q(T_V_W.block<3, 3>(0, 0));
    static bool odom_flag = 1;
    if (odom_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::Odometry>(topic);
      odom_flag = 0;
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

  /**
   * @brief pub tf in rviz
   *
   * @param T_V_W translation from vehicle to world
   * @param sec second in timestamp
   * @param nsec nsecond in timestamp
   * @param topic topic
   * @return
   */
  static void PubTf(const Eigen::Matrix4d& T_V_W, const uint64_t& sec,
                    const uint64_t& nsec, const std::string& topic) {
    Eigen::Vector3d p = T_V_W.block<3, 1>(0, 3);
    Eigen::Quaterniond q(T_V_W.block<3, 3>(0, 0));
    static bool tf_flag = 1;
    if (tf_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::TransformStamped>(
          topic);
      tf_flag = 0;
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

  /**
   * @brief pub path in rviz
   *
   * @param T_V_W translation from vehicle to world
   * @param sec second in timestamp
   * @param nsec nsecond in timestamp
   * @param topic topic
   * @return
   */
  static void PubPath(const Eigen::Matrix4d& T_V_W, const uint64_t& sec,
                      const uint64_t& nsec, const std::string& topic) {
    Eigen::Vector3d p = T_V_W.block<3, 1>(0, 3);
    Eigen::Quaterniond q(T_V_W.block<3, 3>(0, 0));
    static bool path_flag = 1;
    if (path_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::Path>(topic);
      path_flag = 0;
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
    if (path_msg.poses().size() > 250) {
      path_msg.mutable_poses()->DeleteSubrange(0, 1);
    }
    util::RvizAgent::Instance().Publish(topic, path_msg);
  }

  /**
   * @brief pub perception points in rviz
   *
   * @param T_V_W translation from vehicle to world
   * @param latest_lanes input lanes
   * @param sec second in timestamp
   * @param nsec nsecond in timestamp
   * @param topic topic
   * @return
   */
  static void PubPercepPoints(const Eigen::Matrix4d& T_V_W,
                              const std::shared_ptr<Lanes>& latest_lanes,
                              const uint64_t& sec, const uint64_t& nsec,
                              const std::string& topic) {
    static bool percep_flag = 1;
    if (percep_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::PointCloud>(topic);
      percep_flag = 0;
    }
    std::shared_ptr<std::vector<Eigen::Vector3d>> percep_points =
        std::make_shared<std::vector<Eigen::Vector3d>>();
    for (size_t i = 0; i < latest_lanes->front_lanes_.size(); i++) {
      CommonUtil::SampleLanePoints(
          std::make_shared<Lane>(latest_lanes->front_lanes_[i]), T_V_W,
          percep_points);
    }
    adsfi_proto::viz::PointCloud lane_points_msg;
    lane_points_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
    lane_points_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    lane_points_msg.mutable_header()->set_frameid("localmap");
    auto* channels = lane_points_msg.add_channels();
    channels->set_name("rgb");
    for (auto p : *percep_points) {
      auto* points_ = lane_points_msg.add_points();
      points_->set_x(p.x());
      points_->set_y(p.y());
      points_->set_z(p.z());
    }
    util::RvizAgent::Instance().Publish(topic, lane_points_msg);
  }

  /**
   * @brief pub map points in rviz
   *
   * @param local_map input local_map
   * @param sec second in timestamp
   * @param nsec nsecond in timestamp
   * @param topic topic
   * @return
   */
  static void PubMapPoints(const LocalMap& local_map, const uint64_t& sec,
                           const uint64_t& nsec, const std::string& topic) {
    static bool map_flag = 1;
    if (map_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::PointCloud>(topic);
      map_flag = 0;
    }
    std::vector<Eigen::Vector3d> map_points;
    for (size_t i = 0; i < local_map.local_map_lane_.size(); i++) {
      for (size_t j = 0; j < local_map.local_map_lane_[i].lane_param_.size();
           ++j) {
        float start_x =
            local_map.local_map_lane_[i].lane_param_[j].start_point_x_;
        float end_x = local_map.local_map_lane_[i].lane_param_[j].end_point_x_;
        float x = start_x;
        while (x <= end_x) {
          double y =
              local_map.local_map_lane_[i].lane_param_[j].c0_ +
              local_map.local_map_lane_[i].lane_param_[j].c1_ * (x - start_x) +
              local_map.local_map_lane_[i].lane_param_[j].c2_ *
                  pow(x - start_x, 2) +
              local_map.local_map_lane_[i].lane_param_[j].c3_ *
                  pow(x - start_x, 3);
          map_points.push_back({x, y, 0});
          x += 0.1;
        }
      }
    }
    for (size_t i = 0; i < local_map.local_map_lane_.size(); ++i) {
      for (size_t j = 0; j < local_map.local_map_lane_[i].points_.size(); ++j) {
        // map_points.push_back(local_map.local_map_lane_[i].points_[j]);
      }
    }
    adsfi_proto::viz::PointCloud map_points_msg;
    map_points_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
    map_points_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    map_points_msg.mutable_header()->set_frameid("localmap");
    auto* channels = map_points_msg.add_channels();
    channels->set_name("rgb");
    for (auto p : map_points) {
      auto* points_ = map_points_msg.add_points();
      points_->set_x(p.x());
      points_->set_y(p.y());
      points_->set_z(p.z());
    }
    util::RvizAgent::Instance().Publish(topic, map_points_msg);
  }
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
