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

class EdgeRelativePose : public EdgeBase<6, 6, 6> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  DEFINE_SMART_PTR(EdgeRelativePose)

  EdgeRelativePose(uint64_t id, const Eigen::Matrix4d& relative_pose,
                   const Eigen::Matrix<double, 6, 6>& relative_pose_cov)
      : EdgeBase<6, 6, 6>(id), relative_pose_(relative_pose) {
    SetInfomation(relative_pose_cov.inverse());
  }

  bool InnerEvaluate(double const* const* parameters, double* residuals,
                     double** jacobians) const override {
    Eigen::Map<const Eigen::Vector3d> last_trans(parameters[0]);
    Eigen::Map<const Eigen::Vector3d> last_rot_so3(parameters[0] + 3);
    Eigen::Map<const Eigen::Vector3d> curr_trans(parameters[1]);
    Eigen::Map<const Eigen::Vector3d> curr_rot_so3(parameters[1] + 3);
    Eigen::Matrix3d last_rot = SO3d::exp(last_rot_so3).matrix();
    Eigen::Matrix3d curr_rot = SO3d::exp(curr_rot_so3).matrix();
    Eigen::Matrix3d err_deltaR = relative_pose_.block<3, 3>(0, 0).transpose() *
                                 (last_rot.transpose() * curr_rot);
    Eigen::Vector3d err_deltaR_so3 = SO3d(err_deltaR).log();

    // residuals
    Eigen::Map<Eigen::Matrix<double, 6, 1>> residual(residuals);
    residual.segment<3>(0) = last_rot.transpose() * (curr_trans - last_trans) -
                             relative_pose_.block<3, 1>(0, 3);
    residual.segment<3>(3) = err_deltaR_so3;
    residual = sqrt_infomation_ * residual;

    // jacobian of residual w.r.t last pose
    if (jacobians) {
      Eigen::Matrix3d O33 = Eigen::Matrix3d::Zero();
      Eigen::Matrix3d jr_inv = SO3d::JacobianRInv(err_deltaR_so3);
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>
            jacobian_last_pose(jacobians[0]);
        jacobian_last_pose.block<3, 3>(0, 0) = -last_rot.transpose();
        jacobian_last_pose.block<3, 3>(0, 3) =
            SO3d::hat(last_rot.transpose() * (curr_trans - last_trans));
        jacobian_last_pose.block<3, 3>(3, 0) = O33;
        jacobian_last_pose.block<3, 3>(3, 3) =
            -jr_inv * curr_rot.transpose() * last_rot;
        jacobian_last_pose = sqrt_infomation_ * jacobian_last_pose;
      }

      // jacobian of residual w.r.t current pose
      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>
            jacobian_curr_pose(jacobians[1]);
        jacobian_curr_pose.block<3, 3>(0, 0) = last_rot.transpose();
        jacobian_curr_pose.block<3, 3>(0, 3) = O33;
        jacobian_curr_pose.block<3, 3>(3, 0) = O33;
        jacobian_curr_pose.block<3, 3>(3, 3) = jr_inv;
        jacobian_curr_pose = sqrt_infomation_ * jacobian_curr_pose;
      }
    }

    return true;
  }

 private:
  Eigen::Matrix4d relative_pose_;
};

}  // namespace localization
}  // namespace senseAD
