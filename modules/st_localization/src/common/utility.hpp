/*
 * Copyright (C) 2019-2020 by SenseTime Group Limited. All rights reserved.
 * Wang Xiaofeng<wangxiaofeng@sensetime.com>
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>  // NOLINT
#include <map>
#include <string>
#include <vector>

#include <Sophus/se3.hpp>

namespace senseAD {
namespace localization {
// using Sophus::SE3;
// using Sophus::SO3;

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::Vector4d;

// class holds the time counter
class TicToc {
 public:
  TicToc() { tic(); }

  void tic() { start = std::chrono::system_clock::now(); }

  double toc() {
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    return elapsed_seconds.count() * 1000; /* ms */
  }

 private:
  std::chrono::time_point<std::chrono::system_clock> start, end;
};

// class holds the data structure convert or utility methods
class Utility {
 public:
  ///////////////////////////// Eigen related ///////////////////////////////
  static Eigen::Vector3d R2ypr(const Eigen::Matrix3d& R) {
    Eigen::Vector3d n = R.col(0);
    Eigen::Vector3d o = R.col(1);
    Eigen::Vector3d a = R.col(2);

    Eigen::Vector3d ypr(3);
    double y = atan2(n(1), n(0));
    double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
    double r =
        atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
    ypr(0) = y;
    ypr(1) = p;
    ypr(2) = r;

    return ypr;
  }

  static Eigen::Matrix3d ypr2R(const Eigen::Vector3d& ypr) {
    double y = ypr(0);
    double p = ypr(1);
    double r = ypr(2);

    Eigen::Matrix<double, 3, 3> Rz;
    Rz << cos(y), -sin(y), 0, sin(y), cos(y), 0, 0, 0, 1;

    Eigen::Matrix<double, 3, 3> Ry;
    Ry << cos(p), 0., sin(p), 0., 1., 0., -sin(p), 0., cos(p);

    Eigen::Matrix<double, 3, 3> Rx;
    Rx << 1., 0., 0., 0., cos(r), -sin(r), 0., sin(r), cos(r);

    return Rz * Ry * Rx;
  }

  static Eigen::Matrix3d MakePose2D(const double& x, const double& y,
                                    const double& yaw) {
    Eigen::Matrix3d pose = Eigen::Matrix3d::Identity();
    pose.block<2, 1>(0, 2) = Eigen::Vector2d(x, y);
    Eigen::Matrix2d R_wv;
    R_wv << std::cos(yaw), -std::sin(yaw), std::sin(yaw), std::cos(yaw);
    pose.block<2, 2>(0, 0) = R_wv;
    return pose;
  }

  static Eigen::Vector4d EigenQtoHamiltonVec(const Eigen::Quaterniond& q) {
    Eigen::Quaterniond qn = q.normalized();
    Eigen::Vector4d q_hamilton;
    q_hamilton << qn.w(), qn.x(), qn.y(), qn.z();
    return q_hamilton;
  }

  static Eigen::Quaterniond HamiltonVecToEigenQ(const Eigen::Vector4d& q_vec) {
    Eigen::Quaterniond q(q_vec(0), q_vec(1), q_vec(2), q_vec(3));
    q.normalize();
    return q;
  }

  static Eigen::Matrix3d SkewOfVec(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m.fill(0.);
    m(0, 1) = -v(2);
    m(0, 2) = v(1);
    m(1, 2) = -v(0);
    m(1, 0) = v(2);
    m(2, 0) = -v(1);
    m(2, 1) = v(0);
    return m;
  }

  // @brief: calculate the BCH jacobian(l) inverse
  static Eigen::Matrix3d BCHJacobianLInv(const Eigen::Vector3d& angle_axis) {
    double theta = angle_axis.norm();
    Eigen::Vector3d a = angle_axis * (1.0 / theta);
    theta /= 2;
    double cot = 1.0 / std::tan(theta);
    Eigen::Matrix3d Jl_inv = theta * cot * Eigen::Matrix3d::Identity() +
                             (1 - theta * cot) * a * (a.transpose()) -
                             theta * SkewOfVec(a);
    return Jl_inv;
  }

  // @brief: orthogonalize rotation matrix
  static Eigen::Matrix3d OrthoRotMatrix(const Eigen::Matrix3d& R) {
    Eigen::Quaterniond q(R);
    if (q.w() < 0) {
      q.coeffs() *= -1;
    }
    q.normalize();
    return q.toRotationMatrix();
  }

  static Sophus::SE3<double> SE3interpolate(const Sophus::SE3<double>& start,
                                            const Sophus::SE3<double>& end,
                                            double alpha) {
    // 插值旋转
    Sophus::SO3<double> interpolated_rotation =
        Sophus::SO3<double>::exp(alpha *
                                 (end.so3().log() - start.so3().log())) *
        start.so3();

    // 插值平移
    Eigen::Vector3d interpolated_translation =
        (1.0 - alpha) * start.translation() + alpha * end.translation();

    // 构建插值后的SE3变换
    Sophus::SE3<double> interpolated_SE3(interpolated_rotation,
                                         interpolated_translation);

    return interpolated_SE3;
  }
};

}  // namespace localization
}  // namespace senseAD
