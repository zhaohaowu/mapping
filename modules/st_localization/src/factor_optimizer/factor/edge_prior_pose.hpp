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

#include "factor_optimizer/factor/edge.hpp"
#include "localization/data_type/smart_ptr.hpp"

namespace senseAD {
namespace localization {

class EdgePriorPose : public EdgeBase<6, 6> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  DEFINE_SMART_PTR(EdgePriorPose)

  EdgePriorPose(uint64_t id, const Eigen::Matrix4d& pose_prior,
                const Eigen::Matrix<double, 6, 6>& prior_pose_cov)
      : EdgeBase<6, 6>(id), pose_prior_(pose_prior) {
    SetInfomation(prior_pose_cov.inverse());
  }

  bool InnerEvaluate(double const* const* parameters, double* residuals,
                     double** jacobians) const override {
    Eigen::Map<const Eigen::Vector3d> curr_trans(parameters[0]);
    Eigen::Map<const Eigen::Vector3d> curr_rot_so3(parameters[0] + 3);
    Eigen::Matrix3d curr_rot = SO3d::exp(curr_rot_so3).matrix();
    Eigen::Matrix3d err_deltaR =
        pose_prior_.block<3, 3>(0, 0).transpose() * curr_rot;
    Eigen::Vector3d err_deltaR_so3 = SO3d(err_deltaR).log();

    // residuals
    Eigen::Map<Eigen::Matrix<double, 6, 1>> residual(residuals);
    residual.segment<3>(0) = curr_trans - pose_prior_.block<3, 1>(0, 3);
    residual.segment<3>(3) = err_deltaR_so3;
    residual = sqrt_infomation_ * residual;

    // jacobian of residual w.r.t pose state
    if (jacobians && jacobians[0]) {
      Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>
          jacobian_curr_pose(jacobians[0]);
      jacobian_curr_pose.setZero();
      jacobian_curr_pose.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
      jacobian_curr_pose.block<3, 3>(3, 3) =
          SO3d::JacobianRInv(err_deltaR_so3) * curr_rot.transpose();
      jacobian_curr_pose = sqrt_infomation_ * jacobian_curr_pose;
    }

    return true;
  }

 private:
  Eigen::Matrix4d pose_prior_;
};

}  // namespace localization
}  // namespace senseAD
