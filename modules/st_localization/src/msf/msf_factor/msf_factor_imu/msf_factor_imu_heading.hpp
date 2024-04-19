/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
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

using Eigen::Quaterniond;
using Vector1d = Eigen::Matrix<double, 1, 1>;

class MSFFactorIMUHeading : public MSFFactorIMU {
 public:
  MSFFactorIMUHeading() {
    H_.setZero();
    V_.setZero();
  }

  ~MSFFactorIMUHeading() = default;

  void EvaluateHMat(const VectorXd& nominal_state) override {
    // evaluation point of H at current predicted state
    assert(nominal_state.rows() == kiStateSize);
    Quaterniond Qb_n =
        Utility::HamiltonVecToEigenQ(nominal_state.segment<4>(kiStateAttitude));
    Matrix3d Cb_n = Qb_n.toRotationMatrix();
    // set H
    double c11 = Cb_n(0, 0);
    double c21 = Cb_n(1, 0);
    double c31 = Cb_n(2, 0);
    double den = c11 * c11 + c21 * c21;
    H_(0, 6) = -c31 * c11 / den;
    H_(0, 7) = -c31 * c21 / den;
    H_(0, 8) = 1;
  }

  void SetObservationState(const VectorXd& observation) override {
    assert(observation.rows() >= 1);
    obs_state_ = observation;
    heading_obs_ = observation(0);
  }

  void SetObservationCov(const MatrixXd& V) override {
    assert(V.rows() == 1);
    assert(V.cols() == 1);
    obs_cov_ = V;
  }

  Eigen::MatrixXd GetHMat() override { return H_; }

  Eigen::MatrixXd GetVMat() override {
    V_(0, 0) = obs_cov_(0, 0);
    return V_;
  }

  Eigen::VectorXd EvaluateRes(const VectorXd& current_state) override {
    assert(current_state.rows() == kiStateSize);
    // evaluate the error
    Quaterniond Qb_n =
        Utility::HamiltonVecToEigenQ(current_state.segment<4>(kiStateAttitude));
    // attitude error
    // Vector3d ypr_sins = SO3d(Qb_n).toYPR(); old Sophus
    Vector3d ypr_sins = Qb_n.toRotationMatrix().eulerAngles(2, 1, 0);
    double dHeading = heading_obs_ - ypr_sins(0);
    // process heading critical value -pi and pi
    if (heading_obs_ > 0.0 && dHeading > M_PI) {
      dHeading = dHeading - 2 * M_PI;
    } else if (heading_obs_ < 0.0 && dHeading < -M_PI) {
      dHeading = dHeading + 2 * M_PI;
    }
    Vector1d residual;
    residual << dHeading;
    return residual;
  }

 private:
  // virtual heading sensors only provides 1d heading
  static constexpr int kObservationSize = 1;
  // jacobian of observation function w.r.t to error state
  Eigen::Matrix<double, kObservationSize, kErrorStateSize> H_;
  // observation noise matrix
  Eigen::Matrix<double, kObservationSize, kObservationSize> V_;
  // current observation information
  float64_t heading_obs_;
};

}  // namespace msf
}  // namespace localization
}  // namespace senseAD
