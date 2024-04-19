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
#include "localization/common/log.hpp"
#include "localization/data_type/smart_ptr.hpp"

namespace senseAD {
namespace localization {

struct PriorPose4DOFCostFunction {
  PriorPose4DOFCostFunction(const Eigen::Matrix4d& pose_prior,
                            const Eigen::Matrix4d& prior_pose_cov) {
    prior_t_ = pose_prior.block<3, 1>(0, 3);
    prior_yaw_ = Utility::R2ypr(pose_prior.block<3, 3>(0, 0))(0);
    sqrt_info_ = Eigen::LLT<Eigen::Matrix4d>(prior_pose_cov.inverse())
                     .matrixL()
                     .transpose();
  }

  template <typename T>
  bool operator()(const T* parameters, T* residuals) const {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t(parameters);
    T yaw = static_cast<T>(parameters[3]);

    Eigen::Map<Eigen::Matrix<T, 4, 1>> r(residuals);
    r.template segment<3>(0) = t - prior_t_.template cast<T>();
    r(3) = NormalizeAngle(yaw - static_cast<T>(prior_yaw_));
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

  Eigen::Matrix4d sqrt_info_;
  Eigen::Vector3d prior_t_;
  double prior_yaw_;
};

class AutoDiffEdgePriorPose4DOF
    : public AutoDiffEdgeBase<PriorPose4DOFCostFunction, 4, 4> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  DEFINE_SMART_PTR(AutoDiffEdgePriorPose4DOF)

  AutoDiffEdgePriorPose4DOF(uint64_t id, const Eigen::Matrix4d& pose_prior,
                            const Eigen::Matrix4d& prior_pose_cov)
      : AutoDiffEdgeBase<PriorPose4DOFCostFunction, 4, 4>(id) {
    functor_.reset(new PriorPose4DOFCostFunction(pose_prior, prior_pose_cov));
    SetInfomation(prior_pose_cov.inverse());
  }
};

}  // namespace localization
}  // namespace senseAD
