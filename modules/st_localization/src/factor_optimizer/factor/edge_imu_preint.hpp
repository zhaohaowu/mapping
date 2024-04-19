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
#include "imu/imu_preint.hpp"
#include "localization/data_type/smart_ptr.hpp"

namespace senseAD {
namespace localization {

// class holds the imu preintegrate edge, note that imu-preintegrate (Measure)
// is 15-dimensional, and connected previous and current frames have 6d pose, 3d
// velocity, 6d bias for acc and gyro, namely PAVBaBg
class EdgeIMUPreint : public EdgeBase<15, 6, 3, 6, 6, 3, 6> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  DEFINE_SMART_PTR(EdgeIMUPreint)

  EdgeIMUPreint(uint64_t id, IMUPreintMeasurement::Ptr imu_preint,
                const Eigen::Vector6d& babg_last_base,
                const Eigen::Vector3d& gw)
      : EdgeBase<15, 6, 3, 6, 6, 3, 6>(id),
        imu_preint_(imu_preint),
        babg_last_base_(babg_last_base),  // last frame babg before opt
        gw_(gw) {
    SetInfomation(imu_preint_->GetCovarianceFull().inverse());
  }

  bool InnerEvaluate(double const* const* parameters, double* residuals,
                     double** jacobians) const override {
    // state for last frame
    Eigen::Map<const Eigen::Vector3d> pos_last(parameters[0]);
    Eigen::Map<const Eigen::Vector3d> rot_so3_last(parameters[0] + 3);
    Eigen::Matrix3d rot_last = SO3d::exp(rot_so3_last).matrix();
    Eigen::Map<const Eigen::Vector3d> vel_last(parameters[1]);
    Eigen::Map<const Eigen::Vector3d> Ba_last(parameters[2]);
    Eigen::Map<const Eigen::Vector3d> Bg_last(parameters[2] + 3);

    // state for current frame
    Eigen::Map<const Eigen::Vector3d> pos_curr(parameters[3]);
    Eigen::Map<const Eigen::Vector3d> rot_so3_curr(parameters[3] + 3);
    Eigen::Matrix3d rot_curr = SO3d::exp(rot_so3_curr).matrix();
    Eigen::Map<const Eigen::Vector3d> vel_curr(parameters[4]);
    Eigen::Map<const Eigen::Vector3d> Ba_curr(parameters[5]);
    Eigen::Map<const Eigen::Vector3d> Bg_curr(parameters[5] + 3);

    // preintegrate measurement corrected with delta bias
    double dt = imu_preint_->GetDeltaT();
    Eigen::Matrix3d pri_dR = imu_preint_->GetDeltaR();
    Eigen::Vector3d pri_dP = imu_preint_->GetDeltaP();
    Eigen::Vector3d pri_dV = imu_preint_->GetDeltaV();
    Eigen::Matrix3d jac_dR_dbg = imu_preint_->GetJacdRdbg();
    Eigen::Matrix3d jac_dP_dbg = imu_preint_->GetJacdPdbg();
    Eigen::Matrix3d jac_dP_dba = imu_preint_->GetJacdPdba();
    Eigen::Matrix3d jac_dV_dbg = imu_preint_->GetJacdVdbg();
    Eigen::Matrix3d jac_dV_dba = imu_preint_->GetJacdVdba();
    Eigen::Vector3d dBa_last = Ba_last - babg_last_base_.segment<3>(0);
    Eigen::Vector3d dBg_last = Bg_last - babg_last_base_.segment<3>(3);

    Eigen::Matrix3d corret_pri_dR =
        pri_dR * SO3d::exp(jac_dR_dbg * dBg_last).matrix();
    corret_pri_dR = Utility::OrthoRotMatrix(corret_pri_dR);
    Eigen::Vector3d corret_pri_dP =
        pri_dP + jac_dP_dbg * dBg_last + jac_dP_dba * dBa_last;
    Eigen::Vector3d corret_pri_dV =
        pri_dV + jac_dV_dbg * dBg_last + jac_dV_dba * dBa_last;

    // actual state
    Eigen::Matrix3d dR = rot_last.transpose() * rot_curr;
    Eigen::Vector3d dP =
        rot_last.transpose() *
        (pos_curr - pos_last - vel_last * dt - 0.5 * gw_ * dt * dt);
    Eigen::Vector3d dV =
        rot_last.transpose() * (vel_curr - vel_last - gw_ * dt);

    // build residuals
    Eigen::Map<Eigen::Vector15d> residual(residuals);
    Eigen::Vector3d errP = dP - corret_pri_dP;
    Eigen::Matrix3d errR33 = corret_pri_dR.transpose() * dR;
    Eigen::Vector3d errR = SO3d(errR33).log();
    Eigen::Vector3d errV = dV - corret_pri_dV;
    residual.segment<3>(O_P) = errP;
    residual.segment<3>(O_R) = errR;
    residual.segment<3>(O_V) = errV;
    residual.segment<3>(O_BA) = Ba_curr - Ba_last;
    residual.segment<3>(O_BG) = Bg_curr - Bg_last;
    residual = sqrt_infomation_ * residual;

    if (jacobians) {
      Eigen::Matrix3d jr_inv = SO3d::JacobianRInv(errR);
      // jacobian of residual w.r.t last pose
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 15, 6, Eigen::RowMajor>> jac_pr_last(
            jacobians[0]);
        jac_pr_last.setZero();
        jac_pr_last.block<3, 3>(O_P, O_P) = -rot_last.transpose();
        jac_pr_last.block<3, 3>(O_P, O_R) = SO3d::hat(dP);
        jac_pr_last.block<3, 3>(O_R, O_R) =
            -jr_inv * rot_curr.transpose() * rot_last;
        jac_pr_last.block<3, 3>(O_V, O_R) = SO3d::hat(dV);
        jac_pr_last = sqrt_infomation_ * jac_pr_last;
      }

      // jacobian of residual w.r.t last velocity
      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> jac_v_last(
            jacobians[1]);
        jac_v_last.setZero();
        jac_v_last.block<3, 3>(O_P, O_V - O_V) = -rot_last.transpose() * dt;
        jac_v_last.block<3, 3>(O_V, O_V - O_V) = -rot_last.transpose();
        jac_v_last = sqrt_infomation_ * jac_v_last;
      }

      // jacobian of residual w.r.t last bias
      if (jacobians[2]) {
        Eigen::Map<Eigen::Matrix<double, 15, 6, Eigen::RowMajor>> jac_bias_last(
            jacobians[2]);
        jac_bias_last.setZero();
        Eigen::Matrix3d expR = SO3d::exp(errR).matrix().transpose();
        Eigen::Matrix3d jr_b = SO3d::JacobianR(jac_dR_dbg * dBg_last);
        // pos error w.r.t dba and dbg
        jac_bias_last.block<3, 3>(O_P, O_BA - O_BA) = -jac_dP_dba;
        jac_bias_last.block<3, 3>(O_P, O_BG - O_BA) = -jac_dP_dbg;
        // rot error w.r.t. dbg
        jac_bias_last.block<3, 3>(O_R, O_BG - O_BA) =
            -jr_inv * expR * jr_b * jac_dR_dbg;
        // vel error w.r.t dba and dbg
        jac_bias_last.block<3, 3>(O_V, O_BA - O_BA) = -jac_dV_dba;
        jac_bias_last.block<3, 3>(O_V, O_BG - O_BA) = -jac_dV_dbg;
        // bias error w.r.t dba and dbg
        jac_bias_last.block<3, 3>(O_BA, O_BA - O_BA) =
            -Eigen::Matrix3d::Identity();
        jac_bias_last.block<3, 3>(O_BG, O_BG - O_BA) =
            -Eigen::Matrix3d::Identity();
        jac_bias_last = sqrt_infomation_ * jac_bias_last;
      }

      // jacobian of residual w.r.t curr pose
      if (jacobians[3]) {
        Eigen::Map<Eigen::Matrix<double, 15, 6, Eigen::RowMajor>> jac_pr_curr(
            jacobians[3]);
        jac_pr_curr.setZero();
        jac_pr_curr.block<3, 3>(O_P, O_P) = rot_last.transpose();
        jac_pr_curr.block<3, 3>(O_R, O_R) = jr_inv;
        jac_pr_curr = sqrt_infomation_ * jac_pr_curr;
      }

      // jacobian of residual w.r.t curr velocity
      if (jacobians[4]) {
        Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> jac_v_curr(
            jacobians[4]);
        jac_v_curr.setZero();
        jac_v_curr.block<3, 3>(O_V, O_V - O_V) = rot_last.transpose();
        jac_v_curr = sqrt_infomation_ * jac_v_curr;
      }

      // jacobian of residual w.r.t curr bias
      if (jacobians[5]) {
        Eigen::Map<Eigen::Matrix<double, 15, 6, Eigen::RowMajor>> jac_bias_curr(
            jacobians[5]);
        // bias error w.r.t dba and dbg
        jac_bias_curr.setZero();
        jac_bias_curr.block<3, 3>(O_BA, O_BA - O_BA) =
            Eigen::Matrix3d::Identity();
        jac_bias_curr.block<3, 3>(O_BG, O_BG - O_BA) =
            Eigen::Matrix3d::Identity();
        jac_bias_curr = sqrt_infomation_ * jac_bias_curr;
      }
    }

    return true;
  }

 private:
  // error order
  static constexpr int O_P = 0;
  static constexpr int O_R = 3;
  static constexpr int O_V = 6;
  static constexpr int O_BA = 9;
  static constexpr int O_BG = 12;

  IMUPreintMeasurement::Ptr imu_preint_;
  Eigen::Vector6d babg_last_base_;
  Eigen::Vector3d gw_;
};

}  // namespace localization
}  // namespace senseAD
