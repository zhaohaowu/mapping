/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * Wang Xiaofeng <wangxiaofeng@sensetime.com>
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <Sophus/so3.hpp>

#include "common/utility.hpp"
#include "msf/msf_factor/msf_factor_imu/msf_factor_imu.hpp"

namespace senseAD {
namespace localization {
namespace msf {

class MSFFactorIMUMOTION : public MSFFactorIMU {
 public:
  MSFFactorIMUMOTION() {
    H_.setZero();
    V_.setZero();
  }

  ~MSFFactorIMUMOTION() = default;

  void EvaluateHMat(const VectorXd& nominal_state) override {
    assert(nominal_state.rows() == kiStateSize);
    Vector3d v_n = nominal_state.segment<3>(kiStateVelocity);
    Quaterniond Qb_n =
        Utility::HamiltonVecToEigenQ(nominal_state.segment<4>(kiStateAttitude));
    Matrix3d Cn_b = Qb_n.toRotationMatrix().transpose();
    H_.block<3, 3>(0, 6) = -Cn_b * SO3d::hat(v_n);
    H_.block<3, 3>(0, 3) = Cn_b;
    H_(0, kiErrorStateWheelScale) = -v_obs_[0];
  }

  void SetObservationState(const VectorXd& observation) override {
    assert(observation.rows() >= 3);
    obs_state_ = observation;
    // local velocity observation
    v_obs_ = observation.segment<3>(0);
  }

  void SetObservationCov(const MatrixXd& V) override {
    assert(V.rows() == 3);
    assert(V.cols() == 3);
    obs_cov_ = V;
  }

  Eigen::MatrixXd GetHMat() override { return H_; }

  Eigen::MatrixXd GetVMat() override {
    V_ = obs_cov_.block<3, 3>(0, 0);
    return V_;
  }

  Eigen::VectorXd EvaluateRes(const VectorXd& current_state) override {
    assert(current_state.rows() == kiStateSize);
    Vector3d v_n = current_state.segment<3>(kiStateVelocity);
    Quaterniond Qb_n =
        Utility::HamiltonVecToEigenQ(current_state.segment<4>(kiStateAttitude));
    double wheel_scale = current_state(kiStateWheelScale);
    Matrix3d Cb_n = Qb_n.toRotationMatrix();
    Vector3d Z = Cb_n.transpose() * v_n - (1 + wheel_scale) * v_obs_;
    return Z;
  }

 private:
  // motion factor provides 3d velocity (1d odom + 2d nhc) observation
  static constexpr int kObservationSize = 3;
  // jacobian of observation function w.r.t error state
  Eigen::Matrix<double, kObservationSize, kErrorStateSize> H_;
  // observation noise matrix
  Eigen::Matrix<double, kObservationSize, kObservationSize> V_;
  // current observation information
  Vector3d v_obs_;
};

}  // namespace msf
}  // namespace localization
}  // namespace senseAD
