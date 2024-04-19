/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * Wang Xiaofeng <wangxiaofeng@sensetime.com>
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <Sophus/so3.hpp>

#include "common/utility.hpp"
#include "imu/sins.hpp"
#include "msf/msf_factor/msf_factor_imu/msf_factor_imu.hpp"

namespace senseAD {
namespace localization {
namespace msf {

using Eigen::Matrix3d;
using Eigen::Vector3d;
using Vector9d = Eigen::Matrix<double, 9, 1>;

class MSFFactorIMUMOCKFULL : public MSFFactorIMU {
 public:
  MSFFactorIMUMOCKFULL() {
    H_.setZero();
    V_.setZero();
  }

  ~MSFFactorIMUMOCKFULL() = default;

  void EvaluateHMat(const VectorXd& nominal_state) override {
    assert(nominal_state.rows() == kiStateSize);
    Vector3d lla = nominal_state.segment<3>(kiStatePosition);
    Quaterniond Qb_n =
        Utility::HamiltonVecToEigenQ(nominal_state.segment<4>(kiStateAttitude));
    Matrix3d Cb_n = Qb_n.toRotationMatrix();
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
    // set H
    H_.block<3, 3>(0, 0) = Tpr_;
    H_.block<3, 3>(3, 3) = Matrix3d::Identity();
    H_.block<3, 3>(6, 6).setIdentity();
  }

  void SetObservationState(const VectorXd& observation) override {
    assert(observation.rows() >= 10);
    obs_state_ = observation;
    v_obs_ = observation.segment<3>(3);
    Vector4d q_vec = observation.segment<4>(6);
    Quaterniond q = Utility::HamiltonVecToEigenQ(q_vec);
    SE3d vehicle_pose(SO3d(q), observation.segment<3>(0));
    SINS::GlobalSE3toSINSPA(vehicle_pose, &p_obs_, &q_obs_);
  }

  void SetObservationCov(const MatrixXd& V) override {
    assert(V.rows() == 9);
    assert(V.cols() == 9);
    obs_cov_ = V;
  }

  Eigen::MatrixXd GetHMat() override { return H_; }

  Eigen::MatrixXd GetVMat() override {
    V_ = obs_cov_;
    return V_;
  }

  Eigen::VectorXd EvaluateRes(const VectorXd& current_state) override {
    assert(current_state.rows() == kiStateSize);
    // evaluate the error
    Vector3d lla = current_state.segment<3>(kiStatePosition);
    Vector3d Vn = current_state.segment<3>(kiStateVelocity);
    Quaterniond Qb_n =
        Utility::HamiltonVecToEigenQ(current_state.segment<4>(kiStateAttitude));
    // attitude error
    Eigen::Matrix3d temp_matrix = q_obs_.matrix() * Qb_n.matrix().transpose();
    Eigen::Vector3d dAttitude = SO3d(temp_matrix).log();
    // Innovations for position with lever arm correction(in FRD frame)
    Vector3d dp = Tpr_ * (lla - p_obs_);  // + Cb_n * larm_;
    Vector3d dv = Vn - v_obs_;
    Vector9d residual;
    residual << dp, dv, dAttitude;
    return residual;
  }

 private:
  // MOCK_FULL provides 3d position, 3d velocity and 1d heading in 100 HZ
  static constexpr int kObservationSize = 9;
  // jacobian of observation function w.r.t to error state
  Eigen::Matrix<double, kObservationSize, kErrorStateSize> H_;
  // observation noise matrix
  Eigen::Matrix<double, kObservationSize, kObservationSize> V_;
  // current observation information
  Vector3d p_obs_;
  Vector3d v_obs_;
  Quaterniond q_obs_;
  // transform parameters
  Matrix3d Tpr_ = Matrix3d::Zero();
};

}  // namespace msf
}  // namespace localization
}  // namespace senseAD
