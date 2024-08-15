/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： pose2d_error.h
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/

#include <depend/third_party/x86/ceres/include/ceres/ceres.h>

#include <depend/third_party/x86/Sophus/include/Sophus/se2.hpp>

#include "Eigen/Core"

#pragma once

namespace hozon {
namespace mp {
namespace loc {

template <typename T>
Eigen::Matrix<T, 3, 3> XYYaw2Matrix(T x, T y, T yaw) {
  const T cos_yaw = ceres::cos(yaw);
  const T sin_yaw = ceres::sin(yaw);
  Eigen::Matrix<T, 3, 3> transform;
  transform << cos_yaw, -sin_yaw, x, sin_yaw, cos_yaw, y, 0, 0, 1;
  return transform;
}

class Pose2DError {
 public:
  Pose2DError() {}
  Pose2DError(const Eigen::Vector2d& p_v, const Eigen::Vector2d& p_w,
              const double& weight)
      : p_v_(p_v.x(), p_v.y(), 1), p_w_(p_w.x(), p_w.y(), 1), weight_(weight) {}

  template <class T>
  bool operator()(const T* const x, const T* const y, const T* const yaw,
                  T* residuals_ptr) const {
    // 齐次坐标下的2d向量运算
    Eigen::Matrix<T, 3, 1> res =
        p_v_.cast<T>() - XYYaw2Matrix(*x, *y, *yaw).inverse() * p_w_.cast<T>();
    res.array() *= T(weight_);
    residuals_ptr[0] = res[1] * res[1];

    return true;
  }

  static ceres::CostFunction* CreateAutoDiff(const Eigen::Vector2d& p_v,
                                             const Eigen::Vector2d& p_w,
                                             const double& weight) {
    return (new ceres::AutoDiffCostFunction<Pose2DError, 1, 1, 1, 1>(
        new Pose2DError(p_v, p_w, weight)));
  }

  static ceres::CostFunction* CreateNumericDiff(const Eigen::Vector2d& p_v,
                                                const Eigen::Vector2d& p_w,
                                                const double& weight) {
    return (new ceres::NumericDiffCostFunction<Pose2DError, ceres::CENTRAL, 1,
                                               1, 1, 1>(
        new Pose2DError(p_v, p_w, weight)));
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  Eigen::Matrix<double, 3, 1> p_v_, p_w_;
  double weight_;
};

}  // namespace loc
}  // namespace mp
}  // namespace hozon
