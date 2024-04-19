/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * Wang Xiaofeng <wangxiaofeng@sensetime.com>
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <Sophus/se3.hpp>
#include <opencv2/core/eigen.hpp>

#include "ad_scm/ad_transform.hpp"
#include "common/coordinate_converter.hpp"
#include "common/transform_config.hpp"
#include "common/utility.hpp"
#include "imu/sins.hpp"
#include "msf/msf_factor/msf_factor_imu/msf_factor_imu.hpp"

namespace senseAD {
namespace localization {
namespace msf {

using Eigen::Matrix3d;
using Eigen::Vector3d;
using typename senseAD::common::utils::CoordinateTransformUtility;

class MSFFactorIMUGNSS : public MSFFactorIMU {
 public:
  MSFFactorIMUGNSS() {
    H_.setZero();
    V_.setZero();
    larm_ = TransformConfig::GetLeverArm();
  }

  ~MSFFactorIMUGNSS() = default;

  void EvaluateHMat(const VectorXd& nominal_state) override {
    assert(nominal_state.rows() == kiStateSize);
    Vector3d lla = nominal_state.segment<3>(kiStatePosition);
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
    Eigen::Quaterniond Qb_n =
        Utility::HamiltonVecToEigenQ(nominal_state.segment<4>(kiStateAttitude));
    H_.block<3, 1>(0, kiErrorStateGnssBias) =
        Qb_n.toRotationMatrix().middleCols(1, 1);
    Eigen::Matrix3d Cb_n = Qb_n.toRotationMatrix();
    double c11 = Cb_n(0, 0);
    double c21 = Cb_n(1, 0);
    double c31 = Cb_n(2, 0);
    double den = c11 * c11 + c21 * c21;
    H_(3, kiErrorStateAttitude) = -c31 * c11 / den;
    H_(3, kiErrorStateAttitude + 1) = -c31 * c21 / den;
    H_(3, kiErrorStateAttitude + 2) = 1;
  }

  void SetObservationState(const VectorXd& observation) override {
    assert(observation.rows() >= 3);
    obs_state_ = observation;
    // observation position (lla)
    p_obs_ = observation.segment<3>(0);
    heading_obs_ = observation(3);
  }

  void SetObservationCov(const MatrixXd& V) override {
    assert(V.rows() == 6);
    assert(V.cols() == 6);
    obs_cov_ = V;
  }

  Eigen::MatrixXd GetHMat() override { return H_; }

  Eigen::MatrixXd GetVMat() override {
    V_ = obs_cov_.block<4, 4>(0, 0);
    return V_;
  }

  Eigen::VectorXd EvaluateRes(const VectorXd& current_state) override {
    assert(current_state.rows() == kiStateSize);
    Eigen::Vector3d lla = current_state.segment<3>(kiStatePosition);
    Eigen::Quaterniond Qb_n =
        Utility::HamiltonVecToEigenQ(current_state.segment<4>(kiStateAttitude));
    double gnss_bias = current_state(kiStateGnssBias);

    Eigen::Matrix3d Cb_n = Qb_n.toRotationMatrix();
    Eigen::Vector3d ypr_sins = Utility::R2ypr(Cb_n);
    Eigen::Vector3d global_gnss_bias = Cb_n.middleCols(1, 1) * gnss_bias;
    // Innovations for position with lever arm and gnss bias correction
    // (ALL in FRD frame)
    Eigen::Vector4d Z;
    Z.segment<3>(0) = Tpr_ * (lla - p_obs_) + Cb_n * larm_ + global_gnss_bias;
    // Innovations for heading
    double dHeading = heading_obs_ - ypr_sins(0);
    // process heading critical value -pi and pi
    if (heading_obs_ > 0.0 && dHeading > M_PI) {
      dHeading = dHeading - 2 * M_PI;
    } else if (heading_obs_ < 0.0 && dHeading < -M_PI) {
      dHeading = dHeading + 2 * M_PI;
    }
    Z(3) = dHeading;

    return Z;
  }

 private:
  // GNSS provides 4d position (lla and heading) in 1 HZ
  static constexpr int kObservationSize = 4;
  // jacobian of observation function w.r.t to error state
  Eigen::Matrix<double, kObservationSize, kErrorStateSize> H_;
  // observation noise matrix
  Eigen::Matrix<double, kObservationSize, kObservationSize> V_;
  // current observation information
  Vector3d p_obs_;
  float64_t heading_obs_;
  // lever arm parameters (IMU FRD frame)
  Vector3d larm_;
  // transform parameters
  Matrix3d Tpr_ = Matrix3d::Zero();
};

}  // namespace msf
}  // namespace localization
}  // namespace senseAD
