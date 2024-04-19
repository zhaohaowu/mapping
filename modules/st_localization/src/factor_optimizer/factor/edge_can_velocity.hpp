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

class EdgeCANVelocity : public EdgeBase<3, 6, 3> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  DEFINE_SMART_PTR(EdgeCANVelocity)

  EdgeCANVelocity(uint64_t id, const Eigen::Matrix3d& Rvb,
                  const Eigen::Vector3d& can_vel, const Eigen::Matrix3d& cov)
      : EdgeBase<3, 6, 3>(id), Rvb(Rvb), can_vel_(can_vel) {
    SetInfomation(cov.inverse());
  }

  bool InnerEvaluate(double const* const* parameters, double* residuals,
                     double** jacobians) const override {
    Eigen::Map<const Eigen::Vector3d> rot_so3_curr(parameters[0] + 3);
    Eigen::Matrix3d rot_curr = SO3d::exp(rot_so3_curr).matrix();
    Eigen::Map<const Eigen::Vector3d> vel_curr(parameters[1]);

    // residuals
    Eigen::Map<Eigen::Vector3d> residual(residuals);
    residual = Rvb * rot_curr.transpose() * vel_curr - can_vel_;
    residual = sqrt_infomation_ * residual;

    if (jacobians) {
      // jacobian of residual w.r.t current pose
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 3, 6, Eigen::RowMajor>>
            jacobian_curr_pose(jacobians[0]);
        jacobian_curr_pose.block<3, 3>(0, 0) = Eigen::Matrix3d::Zero();
        jacobian_curr_pose.block<3, 3>(0, 3) =
            Rvb * SO3d::hat(rot_curr.transpose() * vel_curr);
        jacobian_curr_pose = sqrt_infomation_ * jacobian_curr_pose;
      }

      // jacobian of residual w.r.t current velocity
      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>
            jacobian_curr_vel(jacobians[1]);
        jacobian_curr_vel = Rvb * rot_curr.transpose();
        jacobian_curr_vel = sqrt_infomation_ * jacobian_curr_vel;
      }
    }

    return true;
  }

 private:
  Eigen::Matrix3d Rvb;
  Eigen::Vector3d can_vel_;
};

class EdgeCANScaledVelocity : public EdgeBase<3, 6, 3, 1> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  DEFINE_SMART_PTR(EdgeCANScaledVelocity)

  EdgeCANScaledVelocity(uint64_t id, const Eigen::Matrix3d& Rvb,
                        const Eigen::Vector3d& can_vel,
                        const Eigen::Matrix3d& cov)
      : EdgeBase<3, 6, 3, 1>(id), Rvb(Rvb), can_vel_(can_vel) {
    SetInfomation(cov.inverse());
  }

  bool InnerEvaluate(double const* const* parameters, double* residuals,
                     double** jacobians) const override {
    Eigen::Map<const Eigen::Vector3d> rot_so3_curr(parameters[0] + 3);
    Eigen::Matrix3d rot_curr = SO3d::exp(rot_so3_curr).matrix();
    Eigen::Map<const Eigen::Vector3d> vel_curr(parameters[1]);
    double scale_curr = *parameters[2];

    // residuals
    Eigen::Map<Eigen::Vector3d> residual(residuals);
    residual = Rvb * rot_curr.transpose() * vel_curr - scale_curr * can_vel_;
    residual = sqrt_infomation_ * residual;

    if (jacobians) {
      // jacobian of residual w.r.t current pose
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 3, 6, Eigen::RowMajor>>
            jacobian_curr_pose(jacobians[0]);
        jacobian_curr_pose.block<3, 3>(0, 0) = Eigen::Matrix3d::Zero();
        jacobian_curr_pose.block<3, 3>(0, 3) =
            Rvb * SO3d::hat(rot_curr.transpose() * vel_curr);
        jacobian_curr_pose = sqrt_infomation_ * jacobian_curr_pose;
      }

      // jacobian of residual w.r.t current velocity
      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>
            jacobian_curr_vel(jacobians[1]);
        jacobian_curr_vel = Rvb * rot_curr.transpose();
        jacobian_curr_vel = sqrt_infomation_ * jacobian_curr_vel;
      }

      // jacobian of residual w.r.t can velocity scale
      if (jacobians[2]) {
        Eigen::Map<Eigen::Vector3d> jacobian_curr_scale(jacobians[2]);
        jacobian_curr_scale = -can_vel_;
        jacobian_curr_scale = sqrt_infomation_ * jacobian_curr_scale;
      }
    }

    return true;
  }

 private:
  Eigen::Matrix3d Rvb;
  Eigen::Vector3d can_vel_;
};

}  // namespace localization
}  // namespace senseAD
