/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * Wang Xiaofeng <wangxiaofeng@sensetime.com>
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "common/utility.hpp"
#include "msf/msf_factor/msf_factor_can/msf_factor_can.hpp"

namespace senseAD {
namespace localization {
namespace msf {

using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Vector6d = Eigen::Matrix<double, 6, 1>;

class MSFFactorCANFINS : public MSFFactorCAN {
 public:
  MSFFactorCANFINS() {
    Hx_.setZero();
    Hdx_.setZero();
    H_.setZero();
    V_.setZero();
  }

  ~MSFFactorCANFINS() = default;

  void EvaluateHMat(const VectorXd& nominal_state) override {
    // evaluation point of H at current predicted state
    Hx_.block<3, 3>(0, 0) = I33;
    Hx_.block<3, 3>(3, 3) = I33;
    assert(kTrueStateSize == kErrorStateSize);
    Hdx_ = Eigen::Matrix<double, kTrueStateSize, kErrorStateSize>::Identity();
    H_ = Hx_ * Hdx_;
  }

  void SetObservationState(const VectorXd& observation) override {
    assert(observation.rows() >= 7);
    obs_state_ = observation;
    // observation position
    p_obs_ = observation.segment<3>(0);
    // observation attitude
    Vector4d q_vec = observation.segment<4>(3);
    q_obs_ = Utility::HamiltonVecToEigenQ(q_vec);
  }

  void SetObservationCov(const MatrixXd& V) override {
    assert(V.rows() == 6);
    assert(V.cols() == 6);
    obs_cov_ = V;
  }

  Eigen::MatrixXd GetHMat() override { return H_; }

  Eigen::MatrixXd GetVMat() override {
    V_ = obs_cov_;
    return V_;
  }

  Eigen::VectorXd EvaluateRes(const VectorXd& current_state) override {
    assert(current_state.rows() == kcStateSize);
    // evaluate the error
    Vector3d dp = p_obs_ - current_state.segment<3>(kcStatePosition);
    Vector4d q_vec = current_state.segment<4>(kcStateQuat);
    Quaterniond q_predict = Utility::HamiltonVecToEigenQ(q_vec);
    Quaterniond dq = q_predict.conjugate() * q_obs_;
    // Vector3d dtheta = SO3d::log(SO3d(dq)); old Sophus
    Vector3d dtheta = SO3d(dq).log();
    Vector6d residual;
    residual << dp, dtheta;
    return residual;
  }

 private:
  // Fused INS provides 3d position and 3d attitude, in total 6d observation
  static constexpr int kObservationSize = 6;
  // jacobian of observation function w.r.t true state
  Eigen::Matrix<double, kObservationSize, kTrueStateSize> Hx_;
  // jacobian of true state w.r.t error state
  Eigen::Matrix<double, kTrueStateSize, kErrorStateSize> Hdx_;
  // jacobian of observation function w.r.t to error state
  Eigen::Matrix<double, kObservationSize, kErrorStateSize> H_;
  // observation noise matrix
  Eigen::Matrix<double, kObservationSize, kObservationSize> V_;
  // current observation information
  Vector3d p_obs_;
  Quaterniond q_obs_;
};

}  // namespace msf
}  // namespace localization
}  // namespace senseAD
