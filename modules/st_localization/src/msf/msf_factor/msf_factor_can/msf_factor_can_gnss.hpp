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
#include "msf/msf_factor/msf_factor_can/msf_factor_can.hpp"

namespace senseAD {
namespace localization {
namespace msf {

using Eigen::Matrix3d;
using Eigen::Vector3d;
using typename senseAD::common::utils::CoordinateTransformUtility;

class MSFFactorCANGNSS : public MSFFactorCAN {
 public:
  MSFFactorCANGNSS() {
    Hx_.setZero();
    Hdx_.setZero();
    H_.setZero();
    V_.setZero();

    CoordinateTransformUtility gTfUtil =
        CoordinateTransformUtility::get_shared_instance();
    if (senseAD::SCM_SUCCESS != gTfUtil.init_with_configuration()) {
      throw std::runtime_error("Failed to init with configuration");
    }
    Transform cvTvg;
    if (!gTfUtil.get_transform("left_antenna", "car_center", cvTvg)) {
      throw std::runtime_error("Failed to read gnss to car center extrinsic");
    }
    Eigen::Matrix4d Tvg;
    cv::cv2eigen(cvTvg, Tvg);
    larm_ = Tvg.block<3, 1>(0, 3);
  }

  ~MSFFactorCANGNSS() = default;

  void EvaluateHMat(const VectorXd& nominal_state) override {
    // evaluation point of H at current predicted state
    Hx_.block<3, 3>(0, 0) = I33;
    assert(kTrueStateSize == kErrorStateSize);
    Hdx_ = Eigen::Matrix<double, kTrueStateSize, kErrorStateSize>::Identity();
    H_ = Hx_ * Hdx_;
  }

  void SetObservationState(const VectorXd& observation) override {
    assert(observation.rows() >= 3);
    obs_state_ = observation;
    // observation position
    Vector3d lla_vec = observation.segment<3>(0);
    PointLLH_t lla;
    lla.lat = lla_vec(0) * r2d;
    lla.lon = lla_vec(1) * r2d;
    lla.height = lla_vec(2);
    // from lla to enu
    PointENU_t enu;
    CoordinateConverter::GetInstance()->LLA2ENU(lla, &enu);
    p_obs_ << enu.x, enu.y, enu.z;
  }

  void SetObservationCov(const MatrixXd& V) override {
    assert(V.rows() == 6);
    assert(V.cols() == 6);
    obs_cov_ = V;
  }

  Eigen::MatrixXd GetHMat() override { return H_; }

  Eigen::MatrixXd GetVMat() override {
    V_ = obs_cov_.block<3, 3>(0, 0);
    return V_;
  }

  Eigen::VectorXd EvaluateRes(const VectorXd& current_state) override {
    assert(current_state.rows() == kcStateSize);
    // vehicle pose
    Vector3d t = current_state.segment<3>(kcStatePosition);
    Vector4d q = current_state.segment<4>(kcStateQuat);
    SE3d vehicle_pose(Utility::HamiltonVecToEigenQ(q), t);
    // Compensate gnss offset w.r.t lever-arm effect
    Matrix3d R = vehicle_pose.so3().matrix();
    Vector3d t_offset_global = R * larm_;
    p_obs_ = p_obs_ - t_offset_global;
    // evaluate the error in ENU
    Vector3d dp = p_obs_ - t;
    return dp;
  }

 private:
  // GNSS provides 3d position observation in 1 HZ
  static constexpr int kObservationSize = 3;
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
  // lever arm parameters (_Vehicle frame)
  Vector3d larm_;
};

}  // namespace msf
}  // namespace localization
}  // namespace senseAD
