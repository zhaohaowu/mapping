/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Fang Yu <fangyu1@senseauto.com>
 */

#pragma once

#include <ceres/ceres.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <memory>
#include <vector>

#include <Sophus/so3.hpp>

#include "common/utility.hpp"
#include "factor_optimizer/factor/edge.hpp"
#include "localization/data_type/smart_ptr.hpp"

namespace senseAD {
namespace localization {

struct RelativePose4DOFCostFunction {
  RelativePose4DOFCostFunction(const Eigen::Vector4d& relative_xyzyaw,
                               const Eigen::Matrix4d& relative_pose_cov,
                               const Eigen::Matrix4d& pose_i) {
    rela_t_ = relative_xyzyaw.segment<3>(0);
    rela_yaw_ = relative_xyzyaw(3);
    sqrt_info_ = Eigen::LLT<Eigen::Matrix4d>(relative_pose_cov.inverse())
                     .matrixL()
                     .transpose();
    Eigen::Vector3d ypr_i = Utility::R2ypr(pose_i.block<3, 3>(0, 0));
    pitch_i_ = ypr_i(1);
    roll_i_ = ypr_i(2);
  }

  template <typename T>
  bool operator()(const T* param_i, const T* param_j, T* residuals) const {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_i(param_i);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_j(param_j);
    T yaw_i = static_cast<T>(param_i[3]);
    T yaw_j = static_cast<T>(param_j[3]);
    T delta_yaw_ij = yaw_j - yaw_i;

    Eigen::Matrix<T, 3, 3> rot_i =
        YPRToRotation(yaw_i, static_cast<T>(pitch_i_), static_cast<T>(roll_i_));
    Eigen::Matrix<T, 3, 1> i_delta_t_ij = rot_i.transpose() * (t_j - t_i);

    Eigen::Map<Eigen::Matrix<T, 4, 1>> r(residuals);
    r.template segment<3>(0) = i_delta_t_ij - rela_t_.template cast<T>();
    r(3) = NormalizeAngle(delta_yaw_ij - static_cast<T>(rela_yaw_));
    r = sqrt_info_.template cast<T>() * r;
    return true;
  }

  // helpers
  template <typename T>
  T NormalizeAngle(const T& angle_rad) const {
    if (angle_rad > T(M_PI))
      return angle_rad - T(M_PI * 2);
    else if (angle_rad < T(-M_PI))
      return angle_rad + T(M_PI * 2);
    else
      return angle_rad;
  }

  template <typename T>
  Eigen::Matrix<T, 3, 3> YPRToRotation(const T y, const T p, const T r) const {
    Eigen::Matrix<T, 3, 3> rotation;
    rotation(0, 0) = ceres::cos(y) * ceres::cos(p);
    rotation(0, 1) = -ceres::sin(y) * ceres::cos(r) +
                     ceres::cos(y) * ceres::sin(p) * ceres::sin(r);
    rotation(0, 2) = ceres::sin(y) * ceres::sin(r) +
                     ceres::cos(y) * ceres::sin(p) * ceres::cos(r);
    rotation(1, 0) = ceres::sin(y) * ceres::cos(p);
    rotation(1, 1) = ceres::cos(y) * ceres::cos(r) +
                     ceres::sin(y) * ceres::sin(p) * ceres::sin(r);
    rotation(1, 2) = -ceres::cos(y) * ceres::sin(r) +
                     ceres::sin(y) * ceres::sin(p) * ceres::cos(r);
    rotation(2, 0) = -ceres::sin(p);
    rotation(2, 1) = ceres::cos(p) * ceres::sin(r);
    rotation(2, 2) = ceres::cos(p) * ceres::cos(r);
    return rotation;
  }

  Eigen::Matrix4d sqrt_info_;
  Eigen::Vector3d rela_t_;
  double rela_yaw_;
  double pitch_i_;
  double roll_i_;
};

class AutoDiffEdgeRelativePose4DOF
    : public AutoDiffEdgeBase<RelativePose4DOFCostFunction, 4, 4, 4> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  DEFINE_SMART_PTR(AutoDiffEdgeRelativePose4DOF)

  AutoDiffEdgeRelativePose4DOF(uint64_t id,
                               const Eigen::Vector4d& relative_xyzyaw,
                               const Eigen::Matrix4d& relative_pose_cov,
                               const Eigen::Matrix4d& pose_i)
      : AutoDiffEdgeBase<RelativePose4DOFCostFunction, 4, 4, 4>(id) {
    functor_.reset(new RelativePose4DOFCostFunction(relative_xyzyaw,
                                                    relative_pose_cov, pose_i));
    SetInfomation(relative_pose_cov.inverse());
  }
};

}  // namespace localization
}  // namespace senseAD
