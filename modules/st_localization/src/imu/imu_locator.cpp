/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * Wang Xiaofeng <wangxiaofeng@sensetime.com>
 */
#include "imu/imu_locator.hpp"

#include "common/transform_config.hpp"
#include "common/utility.hpp"
#include "localization/common/log.hpp"

namespace senseAD {
namespace localization {

adLocStatus_t IMULocator::Init(const LocalizationParam& param) {
  // SINS Initialization
  sins_.reset(new SINS());

  auto square = [](double a) { return a * a; };
  // state covariance initialization
  double sigma_init_ll = 5.0e-8;
  double sigma_init_v = 0.1;
  double sigma_init_a = 0.1;
  double sigma_init_acc_b = 0.01;
  double sigma_init_gyro_b = 0.01;
  double sigma_init_acc_s = 0.01;
  double sigma_init_gyro_s = 0.01;
  sins_->MakeErrorStateCov(square(sigma_init_ll), square(sigma_init_v),
                           square(sigma_init_a), square(sigma_init_acc_b),
                           square(sigma_init_gyro_b), square(sigma_init_acc_s),
                           square(sigma_init_gyro_s));

  // system noise covariance
  double sigma_acc = param.msf_param.imu_sigma_acc;
  double sigma_gyro = param.msf_param.imu_sigma_gyro;
  double sigma_acc_bias = param.msf_param.imu_sigma_acc_bias;
  double sigma_gyro_bias = param.msf_param.imu_sigma_gyro_bias;
  double sigma_acc_scale = param.msf_param.imu_sigma_acc_scale;
  double sigma_gyro_scale = param.msf_param.imu_sigma_gyro_scale;
  sins_->MakeSystemNoiseCov(square(sigma_acc), square(sigma_gyro),
                            square(sigma_acc_bias), square(sigma_gyro_bias),
                            square(sigma_acc_scale), square(sigma_gyro_scale));

  LC_LINFO(LOCALIZATION) << "Create IMULocator done";
  return LOC_SUCCESS;
}

adLocStatus_t IMULocator::SetState(const NavState& nav_state) {
  // TODO(wangxiaofeng) add process
  return LOC_SUCCESS;
}

adLocStatus_t IMULocator::GetState(NavState* nav_state, double* confidence) {
  if (nullptr == nav_state) {
    LC_LERROR(IMU) << "nullptr";
    return LOC_NULL_PTR;
  }

  nav_state->state_source = IMU;
  nav_state->timestamp = timestamp_;
  // Get PVA from sins
  Eigen::Vector3d sins_p = sins_->GetPosition();
  Eigen::Vector3d sins_v = sins_->GetVelocity();
  Eigen::Quaterniond sins_a = sins_->GetQuat();

  SE3d state_enu_vehicle;
  SINS::SINSPAtoGlobalSE3(sins_p, sins_a, &state_enu_vehicle);

  nav_state->pose = state_enu_vehicle;
  // velocity output relative to current vehicle frame
  {
    SE3d state_ned_vehicle;
    TransformConfig::FromRefENUToNED(state_enu_vehicle, &state_ned_vehicle,
                                     true);
    SO3d R = state_ned_vehicle.so3().inverse();
    nav_state->linear_speed = R.matrix() * sins_v;
  }
  // angular output relative to current vehicle frame
  {
    SO3d R = TransformConfig::GetTvb().so3();
    nav_state->angular_speed = R.matrix() * imu_reading_.segment<3>(3);
  }
  // acceleration output relative to current vehicle frame
  {
    Eigen::Vector3d an = sins_->GetINSData().an;
    SE3d state_ned_vehicle;
    TransformConfig::FromRefENUToNED(state_enu_vehicle, &state_ned_vehicle,
                                     true);
    SO3d R = state_ned_vehicle.so3().inverse();
    nav_state->linear_acceleration = R.matrix() * SmoothAccOutput(an);
  }

  // note that covariance tranform from lla to enu coordinate
  IMUPMat P = sins_->GetPMat();
  Eigen::Matrix3d Tpr =
      TransformConfig::GetTref().so3().matrix() * sins_->GetTpr();
  nav_state->pose_cov.block<3, 3>(0, 0) =
      Tpr * P.topLeftCorner(3, 3) * Tpr.transpose();
  nav_state->pose_cov.block<3, 3>(3, 3) = P.block<3, 3>(6, 6);

  return LOC_SUCCESS;
}

adLocStatus_t IMULocator::Process(uint64_t timestamp,
                                  std::shared_ptr<Imu> raw_imu) {
  if (nullptr == raw_imu) {
    return LOC_LOCALIZATION_ERROR;
  }
  if (!initial_state_set_) {
    LC_LERROR(IMU) << "waiting to set initial state";
    return LOC_LOCALIZATION_ERROR;
  }
  if (timestamp <= timestamp_) {
    LC_LERROR(IMU) << "imu data disorder" << timestamp << " " << timestamp_;
    return LOC_LOCALIZATION_ERROR;
  }

  timestamp_ = timestamp;
  imu_reading_ << raw_imu->linear_acceleration.x,
      raw_imu->linear_acceleration.y, raw_imu->linear_acceleration.z,
      raw_imu->angular_velocity.x, raw_imu->angular_velocity.y,
      raw_imu->angular_velocity.z;

  // time update
  sins_->PVAUpdate(timestamp, imu_reading_, true);

  return LOC_SUCCESS;
}

adLocStatus_t IMULocator::SetInitialState(double lat, double lon, double alt,
                                          const Eigen::Vector3d& ypr,
                                          const Eigen::Vector3d& velocity,
                                          double gnss_bias) {
  if (sins_->SetInitialState(lat, lon, alt, ypr, velocity, gnss_bias) ==
      LOC_SUCCESS) {
    initial_state_set_ = true;
    return LOC_SUCCESS;
  }

  return LOC_LOCALIZATION_ERROR;
}

Eigen::Vector3d IMULocator::SmoothAccOutput(const Eigen::Vector3d& an) {
  static constexpr size_t an_ws = 10;
  an_list_.push_back(an);
  if (an_list_.size() < an_ws) {
    an_list_.set_capacity(an_ws);
    return an;
  }

  Eigen::Vector3d an_ave = Eigen::Vector3d::Zero();
  for (const auto& P : an_list_) an_ave += P;
  an_ave /= an_list_.size();
  return an_ave;
}

}  // namespace localization
}  // namespace senseAD
