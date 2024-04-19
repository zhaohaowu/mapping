/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * Wang Xiaofeng <wangxiaofeng@sensetime.com>
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <Sophus/so3.hpp>

#include "common/transform_config.hpp"
#include "common/utility.hpp"
#include "imu/sins.hpp"
#include "msf/msf_factor/msf_factor_imu/msf_factor_imu.hpp"

namespace senseAD {
namespace localization {
namespace msf {

using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Vector6d = Eigen::Matrix<double, 6, 1>;

class MSFFactorIMUSMM : public MSFFactorIMU {
 public:
  MSFFactorIMUSMM() {
    H_.setZero();
    V_.setZero();
  }

  ~MSFFactorIMUSMM() = default;

  void EvaluateHMat(const VectorXd& nominal_state) override {
    // evaluation point of H at current predicted state
    assert(nominal_state.rows() == kiStateSize);
    Eigen::Vector3d lla = nominal_state.segment<3>(kiStatePosition);
    double sinLat = sin(lla(0));
    double cosLat = cos(lla(0));
    double sinLat2 = sinLat * sinLat;
    double Rn = WGS84::R * (1 - WGS84::e2) / pow(1 - WGS84::e2 * sinLat2, 1.5);
    double Re = WGS84::R / pow(1 - WGS84::e2 * sinLat2, 0.5);
    Rn = Rn + lla(2);
    Re = Re + lla(2);
    // position from radians-to-meters
    Tpr_(0, 0) = Rn;
    Tpr_(1, 1) = Re * cosLat;
    Tpr_(2, 2) = -1;
    H_.block<3, 3>(0, 0) = Tpr_;
    H_.block<3, 3>(3, 6) = I33;
    // update observation covaricane jacobian, from local to global
    Quaterniond qb_n =
        Utility::HamiltonVecToEigenQ(nominal_state.segment<4>(kiStateAttitude));
    Eigen::Matrix3d R = qb_n.toRotationMatrix();
    j_cov_.setZero();
    j_cov_.block<3, 3>(0, 0) = R;
    j_cov_.block<3, 3>(3, 3) = R;
  }

  void SetObservationState(const VectorXd& observation) override {
    assert(observation.rows() >= 7);
    obs_state_ = observation;
    Eigen::Vector4d q_vec = observation.segment<4>(3);
    Quaterniond q = Utility::HamiltonVecToEigenQ(q_vec);
    SE3d vehicle_pose(SO3d(q), observation.segment<3>(0));
    SINS::GlobalSE3toSINSPA(vehicle_pose, &p_obs_, &q_obs_);
  }

  void SetObservationCov(const MatrixXd& V) override {
    assert(V.rows() == 6);
    assert(V.cols() == 6);
    obs_cov_ = V;
  }

  Eigen::MatrixXd GetHMat() override { return H_; }

  Eigen::MatrixXd GetVMat() override {
    V_ = j_cov_ * obs_cov_ * j_cov_.transpose();
    return V_;
  }

  Eigen::VectorXd EvaluateRes(const VectorXd& current_state) override {
    assert(current_state.rows() == kiStateSize);
    // evaluate the error
    Eigen::Vector3d lla = current_state.segment<3>(kiStatePosition);
    Eigen::Vector3d dp = Tpr_ * (lla - p_obs_);
    Quaterniond qb_n =
        Utility::HamiltonVecToEigenQ(current_state.segment<4>(kiStateAttitude));
    Quaterniond dq = q_obs_ * qb_n.conjugate();
    // Eigen::Vector3d dtheta = SO3d::log(SO3d(dq)); old Sophus
    Eigen::Vector3d dtheta = SO3d(dq).log();
    Vector6d residual;
    residual << dp, dtheta;
    return residual;
  }

 private:
  // semantic map_matching only provides 2d position and heading
  // promote it to virtual 6d observation
  static constexpr int kObservationSize = 6;
  // jacobian of observation function w.r.t to error state
  Eigen::Matrix<double, kObservationSize, kErrorStateSize> H_;
  // observation noise matrix
  Eigen::Matrix<double, kObservationSize, kObservationSize> V_;
  // current observation information
  // imu position in lla
  Eigen::Vector3d p_obs_;
  // imu attitude(front-right-down) in ned
  Eigen::Quaterniond q_obs_;
  // transform parameters
  Eigen::Matrix3d Tpr_ = Eigen::Matrix3d::Zero();
  // covaricane propagation jacobian
  Eigen::Matrix<double, kObservationSize, kObservationSize> j_cov_;
};

}  // namespace msf
}  // namespace localization
}  // namespace senseAD
