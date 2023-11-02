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
#include "depend/map/hdmap/hdmap.h"
#include "depend/proto/soc/sensor_image.pb.h"
#include "interface/adsfi_proto/viz/sensor_msgs.pb.h"
#include "modules/local_mapping/types/common.h"
#include "modules/local_mapping/utils/map_manager.h"
#include "modules/util/include/util/geo.h"
#include "modules/util/include/util/rviz_agent/rviz_agent.h"
#include "modules/util/include/util/temp_log.h"

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
   * @param T_W_V translation from vehicle to world
   * @param pts sample points
   * @return
   */
  static void SampleLanePoints(std::shared_ptr<const Lane> lane,
                               std::vector<Eigen::Vector3d>* pts,
                               const double& sample_interval) {
    pts->clear();
    double x, y;
    for (x = lane->x_start_vrf_; x <= lane->x_end_vrf_; x += sample_interval) {
      y = f(x, lane->lane_fit_d_, lane->lane_fit_c_, lane->lane_fit_b_,
            lane->lane_fit_a_);
      Eigen::Vector3d pt(x, y, 0);
      pts->emplace_back(pt);
    }
  }

  static void SampleEKFPoints(const LaneCubicSpline& cubic_curve,
                              std::shared_ptr<std::vector<Eigen::Vector2d>> pts,
                              float gap = 1.0) {
    double a = cubic_curve.c0_;
    double b = cubic_curve.c1_;
    double c = cubic_curve.c2_;
    double d = cubic_curve.c3_;
    double start_x = cubic_curve.start_point_x_;
    double end_x = cubic_curve.end_point_x_;

    for (int curr_x = 0; curr_x <= 50; ++curr_x) {
      if (curr_x > end_x) {
        break;
      }
      float curr_y =
          a * curr_x * curr_x * curr_x + b * curr_x * curr_x + c * curr_x + d;
      Eigen::Vector2d pt(curr_x, curr_y);
      pts->push_back(pt);
    }
  }

  static LaneCubicSpline MapVehicleLaneTolane(
      std::shared_ptr<const Lane> cur_lane) {
    LaneCubicSpline cur_lane_func;
    cur_lane_func.c0_ = cur_lane->lane_fit_a_;
    cur_lane_func.c1_ = cur_lane->lane_fit_b_;
    cur_lane_func.c2_ = cur_lane->lane_fit_c_;
    cur_lane_func.c3_ = cur_lane->lane_fit_d_;
    cur_lane_func.start_point_x_ = cur_lane->x_start_vrf_;
    cur_lane_func.end_point_x_ = cur_lane->x_end_vrf_;
    return cur_lane_func;
  }

  /**
   * @brief lane from sample points
   *
   * @param pts target points
   * @param lane output lane func
   * @return
   */
  static void FitEKFLane(const std::vector<Eigen::Vector3d>& pts,
                         std::shared_ptr<LaneCubicSpline> lane) {
    int n = pts.size();
    Eigen::Matrix<double, Eigen::Dynamic, 4> A(n, 4);
    Eigen::VectorXd x(n);
    Eigen::VectorXd b(n);
    for (int i = 0; i < n; i++) {
      double xi = pts[i][0];
      double yi = pts[i][1];
      A(i, 0) = xi * xi * xi;
      A(i, 1) = xi * xi;
      A(i, 2) = xi;
      A(i, 3) = 1.0;
      b[i] = yi;
    }
    x = (A.transpose() * A).inverse() * A.transpose() * b;
    lane->c0_ = x[0];
    lane->c1_ = x[1];
    lane->c2_ = x[2];
    lane->c3_ = x[3];
    lane->start_point_x_ = pts[0][0];
    lane->end_point_x_ = pts[n - 1][0];
  }

  static void FitEKFLane(const std::vector<Eigen::Vector2d>& pts,
                         std::shared_ptr<LaneCubicSpline> lane) {
    int n = pts.size();
    Eigen::Matrix<double, Eigen::Dynamic, 4> A(n, 4);
    Eigen::VectorXd x(n);
    Eigen::VectorXd b(n);
    for (int i = 0; i < n; i++) {
      double xi = pts[i][0];
      double yi = pts[i][1];
      A(i, 0) = xi * xi * xi;
      A(i, 1) = xi * xi;
      A(i, 2) = xi;
      A(i, 3) = 1.0;
      b[i] = yi;
    }
    x = (A.transpose() * A).inverse() * A.transpose() * b;
    lane->c0_ = x[0];
    lane->c1_ = x[1];
    lane->c2_ = x[2];
    lane->c3_ = x[3];
    lane->start_point_x_ = pts[0][0];
    lane->end_point_x_ = pts[n - 1][0];
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
    double interval = 1;
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
   * @param lane output lane
   * @return
   */
  static void FitLanePoints(const std::vector<Eigen::Vector3d>& pts,
                            std::shared_ptr<Lane> lane) {
    int n = pts.size();
    Eigen::Matrix<double, Eigen::Dynamic, 4> A(n, 4);
    Eigen::VectorXd x(n);
    Eigen::VectorXd b(n);
    for (int i = 0; i < n; i++) {
      double xi = pts[i][0];
      double yi = pts[i][1];
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
    lane->x_start_vrf_ = pts[0][0];
    lane->x_end_vrf_ = pts[n - 1][0];
  }

  /**
   * @brief fit lane points to lane_param in local map
   *
   * @param local_map local_map
   * @return
   */
  static void CubicCurve(std::shared_ptr<LocalMap> local_map,
                         const double& sample_interval) {
    int tmp_num = 10 / sample_interval;
    for (size_t k = 0; k < local_map->local_map_lane_.size(); k++) {
      if (!local_map->local_map_lane_[k].need_fit_) continue;
      local_map->local_map_lane_[k].lane_param_.clear();
      local_map->local_map_lane_[k].fit_points_.clear();
      std::vector<Eigen::Vector3d> pts;
      for (size_t i = 0; i < local_map->local_map_lane_[k].points_.size();
           i++) {
        if (i % tmp_num == 0 ||
            i == local_map->local_map_lane_[k].points_.size() - 1) {
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
      for (size_t i = 0; i < n; i++) {
        LaneCubicSpline tmp;
        if (i != n - 1) {
          tmp.c0_ = pts[i].y();
          tmp.c1_ = b[i];
          tmp.c2_ = c[i];
          tmp.c3_ = d[i];
          tmp.start_point_x_ = pts[i].x();
          tmp.end_point_x_ = pts[i + 1].x();
        }
        int num = 0;
        for (size_t index = tmp_num * i;
             index < local_map->local_map_lane_[k].points_.size(); index++) {
          tmp.sample_x_.push_back(
              local_map->local_map_lane_[k].points_[index].x());
          num++;
          if (num == tmp_num) {
            num = 0;
            break;
          }
        }
        local_map->local_map_lane_[k].lane_param_.emplace_back(tmp);
      }
    }
    for (size_t i = 0; i < local_map->local_map_lane_.size(); i++) {
      if (!local_map->local_map_lane_[i].need_fit_) continue;
      for (size_t j = 0; j < local_map->local_map_lane_[i].lane_param_.size();
           ++j) {
        float start_x =
            local_map->local_map_lane_[i].lane_param_[j].start_point_x_;
        float end_x = local_map->local_map_lane_[i].lane_param_[j].end_point_x_;
        for (size_t k = 0;
             k < local_map->local_map_lane_[i].lane_param_[j].sample_x_.size();
             k++) {
          double x = local_map->local_map_lane_[i].lane_param_[j].sample_x_[k];
          double y =
              local_map->local_map_lane_[i].lane_param_[j].c0_ +
              local_map->local_map_lane_[i].lane_param_[j].c1_ * (x - start_x) +
              local_map->local_map_lane_[i].lane_param_[j].c2_ *
                  pow(x - start_x, 2) +
              local_map->local_map_lane_[i].lane_param_[j].c3_ *
                  pow(x - start_x, 3);
          local_map->local_map_lane_[i].fit_points_.push_back({x, y, 0});
        }
      }
    }
  }

  /**
   * @brief fit lane points to lane_param in local map
   *
   * @param local_map local_map
   * @return
   */
  static void CatmullRom(std::shared_ptr<LocalMap> local_map,
                         const double& sample_interval) {
    for (auto& lane : local_map->local_map_lane_) {
      if (!lane.need_fit_) continue;
      lane.lane_param_.clear();
      lane.fit_points_.clear();
      std::vector<Eigen::Vector3d> pts;
      for (size_t i = 0; i < lane.points_.size(); i++) {
        if (i % 10 == 0 || i == lane.points_.size() - 1) {
          pts.push_back(lane.points_[i]);
        }
      }
      Eigen::Vector3d p0, p1, p2, p3;
      if (pts.size() < 4) continue;
      p0 = pts[0];
      p1 = pts[1];
      p2 = pts[pts.size() - 2];
      p3 = pts[pts.size() - 1];
      // pts.push_back(2 * p3 - p2);
      // pts.insert(pts.begin(), 2 * p0 - p1);
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
          lane.fit_points_.push_back({px, py, 0});
        }
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
   * @param T_W_V translation from vehicle to world
   * @param lane world points
   * @return
   */
  static void PointsToWorld(
      std::shared_ptr<const std::vector<Eigen::Vector3d>> pts,
      const Sophus::SE3d& T_W_V,
      std::shared_ptr<std::vector<Eigen::Vector3d>> world_pts) {
    for (auto& pt : *pts) {
      Eigen::Vector3d v_pt(pt(0), pt(1), pt(2));
      Eigen::Vector3d w_tmp_pt = T_W_V * v_pt;
      Eigen::Vector3d w_pt(w_tmp_pt(0), w_tmp_pt(1), w_tmp_pt(2));
      world_pts->emplace_back(w_pt);
    }
  }

  /**
   * @brief pub odom in rviz
   *
   * @param T_W_V translation from vehicle to world
   * @param sec second in timestamp
   * @param nsec nsecond in timestamp
   * @param topic topic
   * @return
   */
  static void PubOdom(const Sophus::SE3d& T_W_V, const uint64_t& sec,
                      const uint64_t& nsec, const std::string& topic) {
    Eigen::Vector3d p = T_W_V.translation();
    Eigen::Quaterniond q = T_W_V.so3().unit_quaternion();
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
   * @param T_W_V translation from vehicle to world
   * @param sec second in timestamp
   * @param nsec nsecond in timestamp
   * @param topic topic
   * @return
   */
  static void PubTf(const Sophus::SE3d& T_W_V, const uint64_t& sec,
                    const uint64_t& nsec, const std::string& topic) {
    Eigen::Vector3d p = T_W_V.translation();
    Eigen::Quaterniond q = T_W_V.so3().unit_quaternion();
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
   * @param T_W_V translation from vehicle to world
   * @param sec second in timestamp
   * @param nsec nsecond in timestamp
   * @param topic topic
   * @return
   */
  static void PubPath(const Sophus::SE3d& T_W_V, const uint64_t& sec,
                      const uint64_t& nsec, const std::string& topic) {
    Eigen::Vector3d p = T_W_V.translation();
    Eigen::Quaterniond q = T_W_V.so3().unit_quaternion();
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
    // if (path_msg.poses().size() > 250) {
    //   path_msg.mutable_poses()->DeleteSubrange(0, 1);
    // }
    util::RvizAgent::Instance().Publish(topic, path_msg);
  }

  /**
   * @brief pub perception points in rviz
   *
   * @param T_W_V translation from vehicle to world
   * @param latest_lanes input lanes
   * @param sec second in timestamp
   * @param nsec nsecond in timestamp
   * @param topic topic
   * @return
   */
  static void PubPercepPoints(const Sophus::SE3d& T_W_V,
                              const std::shared_ptr<Lanes>& latest_lanes,
                              const uint64_t& sec, const uint64_t& nsec,
                              const std::string& topic,
                              const double& sample_interval) {
    static bool percep_flag = 1;
    if (percep_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::PointCloud>(topic);
      percep_flag = 0;
    }
    std::vector<Eigen::Vector3d> percep_points_all;
    for (size_t i = 0; i < latest_lanes->lanes_.size(); i++) {
      std::vector<Eigen::Vector3d> percep_points;
      CommonUtil::SampleLanePoints(
          std::make_shared<Lane>(latest_lanes->lanes_[i]), &percep_points,
          sample_interval);
      for (auto& point : percep_points) {
        point = T_W_V * point;
      }
      for (auto& p : percep_points) {
        percep_points_all.emplace_back(p);
      }
    }
    adsfi_proto::viz::PointCloud lane_points_msg;
    lane_points_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
    lane_points_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    lane_points_msg.mutable_header()->set_frameid("localmap");
    auto* channels = lane_points_msg.add_channels();
    channels->set_name("rgb");
    for (auto p : percep_points_all) {
      auto* points_ = lane_points_msg.add_points();
      points_->set_x(p.x());
      points_->set_y(p.y());
      points_->set_z(p.z());
    }
    util::RvizAgent::Instance().Publish(topic, lane_points_msg);
  }

  /**
   * @brief pub perception points in rviz
   *
   * @param T_W_V translation from vehicle to world
   * @param latest_lanes input lanes
   * @param sec second in timestamp
   * @param nsec nsecond in timestamp
   * @param topic topic
   * @return
   */
  static void PubEdgePoints(const Sophus::SE3d& T_W_V,
                            const std::shared_ptr<Lanes>& latest_lanes,
                            const uint64_t& sec, const uint64_t& nsec,
                            const std::string& topic,
                            const double& sample_interval) {
    static bool edge_flag = 1;
    if (edge_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::PointCloud>(topic);
      edge_flag = 0;
    }
    std::vector<Eigen::Vector3d> percep_points_all;
    for (size_t i = 0; i < latest_lanes->lanes_.size(); i++) {
      std::vector<Eigen::Vector3d> percep_points;
      CommonUtil::SampleLanePoints(
          std::make_shared<Lane>(latest_lanes->lanes_[i]), &percep_points,
          sample_interval);
      for (auto& point : percep_points) {
        point = T_W_V * point;
      }
      for (auto& p : percep_points) {
        percep_points_all.emplace_back(p);
      }
    }
    adsfi_proto::viz::PointCloud lane_points_msg;
    lane_points_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
    lane_points_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    lane_points_msg.mutable_header()->set_frameid("localmap");
    auto* channels = lane_points_msg.add_channels();
    channels->set_name("rgb");
    for (auto p : percep_points_all) {
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
                           const uint64_t& nsec, const std::string& topic,
                           const Sophus::SE3d& T_W_V) {
    static bool map_flag = 1;
    if (map_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::PointCloud>(topic);
      map_flag = 0;
    }
    adsfi_proto::viz::PointCloud map_points_msg;
    map_points_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
    map_points_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    map_points_msg.mutable_header()->set_frameid("localmap");
    auto* channels = map_points_msg.add_channels();
    channels->set_name("rgb");
    for (auto& lane : local_map.local_map_lane_) {
      for (auto& point : lane.fit_points_) {
        auto tmp = T_W_V * point;
        auto* points_ = map_points_msg.add_points();
        points_->set_x(tmp.x());
        points_->set_y(tmp.y());
        points_->set_z(tmp.z());
      }
    }
    util::RvizAgent::Instance().Publish(topic, map_points_msg);
  }
  /**
   * @brief pub map points parker in rviz
   *
   * @param local_map input local_map
   * @param sec second in timestamp
   * @param nsec nsecond in timestamp
   * @param topic topic
   * @return
   */
  static void PubMapPointsMarker(const LocalMap& local_map, const uint64_t& sec,
                                 const uint64_t& nsec, const std::string& topic,
                                 const Sophus::SE3d& T_W_V) {
    static bool map_marker_flag = 1;
    if (map_marker_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::MarkerArray>(
          topic);
      map_marker_flag = 0;
    }
    adsfi_proto::viz::MarkerArray markers;
    int id = 0;
    for (size_t i = 0; i < local_map.local_map_lane_.size(); i++) {
      if (local_map.local_map_lane_[i].fit_points_.size() == 0) continue;
      int tmp_n = local_map.local_map_lane_[i].fit_points_.size();
      auto end_points =
          T_W_V * local_map.local_map_lane_[i].fit_points_[tmp_n - 1];
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
      for (const auto& point : local_map.local_map_lane_[i].fit_points_) {
        auto tmp = T_W_V * point;
        auto pt = point_marker.add_points();
        pt->set_x(tmp.x());
        pt->set_y(tmp.y());
        pt->set_z(tmp.z());
      }
      if (point_marker.points().size() != 0) {
        markers.add_markers()->CopyFrom(point_marker);
      }

      adsfi_proto::viz::Marker track_id;
      track_id.set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
      track_id.set_action(adsfi_proto::viz::MarkerAction::ADD);
      track_id.set_id(id++);
      track_id.mutable_lifetime()->set_sec(0);
      // track_id.mutable_lifetime()->set_nsec(200000000);
      track_id.mutable_header()->mutable_timestamp()->set_sec(sec);
      track_id.mutable_header()->mutable_timestamp()->set_nsec(nsec);
      track_id.mutable_header()->set_frameid("localmap");
      track_id.mutable_pose()->mutable_position()->set_x(end_points.x());
      track_id.mutable_pose()->mutable_position()->set_y(end_points.y());
      track_id.mutable_pose()->mutable_position()->set_z(1);
      track_id.mutable_pose()->mutable_orientation()->set_x(0);
      track_id.mutable_pose()->mutable_orientation()->set_y(0);
      track_id.mutable_pose()->mutable_orientation()->set_z(0);
      track_id.mutable_pose()->mutable_orientation()->set_w(1);
      track_id.mutable_color()->set_r(0);
      track_id.mutable_color()->set_g(1);
      track_id.mutable_color()->set_b(0);
      track_id.mutable_color()->set_a(1);
      track_id.set_text(std::to_string(local_map.local_map_lane_[i].track_id_));
      track_id.mutable_scale()->set_x(1);
      track_id.mutable_scale()->set_y(1);
      track_id.mutable_scale()->set_z(1);
      markers.add_markers()->CopyFrom(track_id);

      adsfi_proto::viz::Marker lane_pos;
      lane_pos.set_type(adsfi_proto::viz::MarkerType::TEXT_VIEW_FACING);
      lane_pos.set_action(adsfi_proto::viz::MarkerAction::ADD);
      lane_pos.set_id(id++);
      lane_pos.mutable_lifetime()->set_sec(0);
      // lane_pos.mutable_lifetime()->set_nsec(200000000);
      lane_pos.mutable_header()->mutable_timestamp()->set_sec(sec);
      lane_pos.mutable_header()->mutable_timestamp()->set_nsec(nsec);
      lane_pos.mutable_header()->set_frameid("localmap");
      lane_pos.mutable_pose()->mutable_position()->set_x(end_points.x());
      lane_pos.mutable_pose()->mutable_position()->set_y(end_points.y());
      lane_pos.mutable_pose()->mutable_position()->set_z(2);
      lane_pos.mutable_pose()->mutable_orientation()->set_x(0);
      lane_pos.mutable_pose()->mutable_orientation()->set_y(0);
      lane_pos.mutable_pose()->mutable_orientation()->set_z(0);
      lane_pos.mutable_pose()->mutable_orientation()->set_w(1);
      lane_pos.mutable_color()->set_r(1);
      lane_pos.mutable_color()->set_g(0);
      lane_pos.mutable_color()->set_b(0);
      lane_pos.mutable_color()->set_a(1);
      lane_pos.set_text(std::to_string(local_map.local_map_lane_[i].lanepos_));
      lane_pos.mutable_scale()->set_x(1);
      lane_pos.mutable_scale()->set_y(1);
      lane_pos.mutable_scale()->set_z(1);
      markers.add_markers()->CopyFrom(lane_pos);
    }
    util::RvizAgent::Instance().Publish(topic, markers);
  }

  /**
   * @brief pub ori map points in rviz
   *
   * @param local_map input local_map
   * @param sec second in timestamp
   * @param nsec nsecond in timestamp
   * @param topic topic
   * @return
   */
  static void PubOriMapPoints(const LocalMap& local_map, const uint64_t& sec,
                              const uint64_t& nsec, const std::string& topic,
                              const Sophus::SE3d& T_W_V) {
    static bool map_flag = 1;
    if (map_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::PointCloud>(topic);
      map_flag = 0;
    }
    adsfi_proto::viz::PointCloud map_points_msg;
    map_points_msg.mutable_header()->mutable_timestamp()->set_sec(sec);
    map_points_msg.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    map_points_msg.mutable_header()->set_frameid("localmap");
    auto* channels = map_points_msg.add_channels();
    channels->set_name("rgb");
    for (auto& lane : local_map.local_map_lane_) {
      for (auto& point : lane.points_) {
        auto tmp = T_W_V * point;
        auto* points_ = map_points_msg.add_points();
        points_->set_x(tmp.x());
        points_->set_y(tmp.y());
        points_->set_z(tmp.z());
      }
    }
    util::RvizAgent::Instance().Publish(topic, map_points_msg);
  }

  /**
   * @brief pub hd map points in rviz
   *
   * @param local_map input local_map
   * @param sec second in timestamp
   * @param nsec nsecond in timestamp
   * @param topic topic
   * @return
   */
  static void PubHdMapPoints(const Sophus::SE3d& T_G_V,
                             const Sophus::SE3d& T_W_V,
                             std::shared_ptr<hozon::hdmap::HDMap> hdmap,
                             const uint64_t& sec, const uint64_t& nsec,
                             const std::string& topic,
                             std::vector<Eigen::Vector3d>* hq_pts = nullptr) {
    static bool hd_map_flag = 1;
    static Sophus::SE3d T_W_U =
        Sophus::SE3d(Eigen::Matrix<double, 4, 4>::Identity());
    Eigen::Vector3d p_U_V = Eigen::Vector3d::Identity();
    util::Geo::LatLonToUtmXy(51, T_G_V.translation().y(),
                             T_G_V.translation().x(), &p_U_V);
    p_U_V.z() = 0;
    Sophus::SE3d T_U_V = Sophus::SE3d(T_G_V.so3().unit_quaternion(), p_U_V);
    T_W_U = T_W_V * T_U_V.inverse();
    hozon::common::PointENU pos_u_v;
    pos_u_v.set_x(p_U_V.x());
    pos_u_v.set_y(p_U_V.y());
    pos_u_v.set_z(0);
    std::vector<hozon::hdmap::LaneInfoConstPtr> lanes;
    int result = hdmap->GetLanes(pos_u_v, 150, &lanes);
    if (result != 0) {
      return;
    }
    if (hd_map_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::MarkerArray>(
          topic);
      hd_map_flag = 0;
    }
    adsfi_proto::viz::MarkerArray markers;
    int id = 0;
    for (const auto& lane : lanes) {
      for (const auto& i : lane->lane().left_boundary().curve().segment()) {
        std::vector<Eigen::Vector3d> hq_points;
        for (const auto& point : i.line_segment().point()) {
          Eigen::Vector3d temp_point(point.x(), point.y(), 0);

          temp_point = T_W_U * temp_point;
          if (hq_pts) hq_pts->push_back(temp_point);

          hq_points.emplace_back(temp_point);
        }
        adsfi_proto::viz::Marker marker_msg;
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
        for (const auto& p : hq_points) {
          auto predict_pt = marker_msg.add_points();
          predict_pt->set_x(p[0]);
          predict_pt->set_y(p[1]);
          predict_pt->set_z(0);
        }
        if (!marker_msg.points().empty()) {
          markers.add_markers()->CopyFrom(marker_msg);
        }
      }
      for (const auto& i : lane->lane().right_boundary().curve().segment()) {
        std::vector<Eigen::Vector3d> hq_points;
        for (const auto& point : i.line_segment().point()) {
          Eigen::Vector3d temp_point(point.x(), point.y(), 0);

          temp_point = T_W_U * temp_point;
          hq_points.emplace_back(temp_point);
          if (hq_pts) hq_pts->push_back(temp_point);
        }

        static int id = 0;
        adsfi_proto::viz::Marker marker_msg;
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
        for (const auto& p : hq_points) {
          auto predict_pt = marker_msg.add_points();
          predict_pt->set_x(p[0]);
          predict_pt->set_y(p[1]);
          predict_pt->set_z(0);
        }
        if (!marker_msg.points().empty()) {
          markers.add_markers()->CopyFrom(marker_msg);
        }
      }
    }
    util::RvizAgent::Instance().Publish(topic, markers);
  }
  /**
   * @brief pub image in rviz
   *
   * @param local_map input local_map
   * @return
   */
  static void PubImage(
      const std::string& topic,
      const std::shared_ptr<const hozon::soc::CompressedImage>& sensor_img) {
    static bool img_marker_flag = 1;
    if (img_marker_flag) {
      util::RvizAgent::Instance().Register<adsfi_proto::viz::CompressedImage>(
          topic);
      img_marker_flag = 0;
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
  /**
   *@brief sample points from curve*
   * @param cubic_curve target cubic_curve
   * @param pts sample points
   * @param gap sample gap
   * @ return
   */
  static void SampleCurvePts(const LaneCubicSpline& cubic_curve,
                             std::vector<Eigen::Vector3d>* pts,
                             float gap = 1.0) {
    double a = cubic_curve.c0_;
    double b = cubic_curve.c1_;
    double c = cubic_curve.c2_;
    double d = cubic_curve.c3_;
    double start_x = cubic_curve.start_point_x_;
    double end_x = cubic_curve.end_point_x_;

    float fac = 1.0;
    for (int i = 0;; ++i) {
      float curr_x = start_x + static_cast<float>(i) * gap * fac;
      if ((fac > 0 && curr_x >= end_x) || (fac < 0 && curr_x <= end_x)) {
        break;
      }
      float curr_y =
          a * curr_x * curr_x * curr_x + b * curr_x * curr_x + c * curr_x + d;
      Eigen::Vector3d pt(curr_x, curr_y, 0);
      pts->push_back(pt);
    }
  }
};  // namespace lm

}  // namespace lm
}  // namespace mp
}  // namespace hozon
