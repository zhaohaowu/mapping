/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Fang Yu <fangyu1@senseauto.com>
 */
#include "imu/imu_preint.hpp"

#include "localization/data_type/base.hpp"
namespace senseAD {
namespace localization {

IMUPreintMeasurement::IMUPreintMeasurement()
    : deltaR_(Eigen::Matrix3d::Identity()),
      deltaP_(Eigen::Vector3d::Zero()),
      deltaV_(Eigen::Vector3d::Zero()),
      deltaT_(0.0),
      covariance_(1e-10 * Eigen::Matrix15d::Identity()),
      jacobian_(Eigen::Matrix15d::Identity()) {
  std::vector<IMUMeta>().swap(imu_metas_);
}

IMUPreintMeasurement::IMUPreintMeasurement(const IMUPreintMeasurement& copy)
    : deltaR_(copy.deltaR_),
      deltaP_(copy.deltaP_),
      deltaV_(copy.deltaV_),
      deltaT_(copy.deltaT_),
      covariance_(copy.covariance_),
      jacobian_(copy.jacobian_),
      imu_metas_(copy.imu_metas_) {}

IMUPreintMeasurement& IMUPreintMeasurement::operator=(
    const IMUPreintMeasurement& copy) {
  deltaR_ = copy.deltaR_;
  deltaP_ = copy.deltaP_;
  deltaV_ = copy.deltaV_;
  deltaT_ = copy.deltaT_;
  covariance_ = copy.covariance_;
  jacobian_ = copy.jacobian_;
  imu_metas_ = copy.imu_metas_;
  return *this;
}

void IMUPreintMeasurement::Init() {
  deltaR_ = Eigen::Matrix3d::Identity();
  deltaP_.setZero();
  deltaV_.setZero();
  deltaT_ = 0.0;
  covariance_ = 1e-10 * Eigen::Matrix15d::Identity();
  jacobian_ = Eigen::Matrix15d::Identity();
  std::vector<IMUMeta>().swap(imu_metas_);
}

void IMUPreintMeasurement::Update(const Eigen::Vector3d& gyro, double gyro_n,
                                  double gyro_w, const Eigen::Vector3d& acc,
                                  double acc_n, double acc_w, double dt) {
  deltaT_ += dt;

  /////////////////////////// state propagation ///////////////////////////
  // get last state for midpoint preintegrate
  Eigen::Vector3d gyro_last = gyro, acc_last = acc;
  if (!imu_metas_.empty()) {
    gyro_last = imu_metas_.back().gyro;
    acc_last = imu_metas_.back().acc;
  }
  Eigen::Matrix3d deltaR_last = deltaR_;

  // update preintegrate rotation
  Eigen::Vector3d gyro_mid = 0.5 * (gyro_last + gyro);
  Eigen::Matrix3d inc_R = SO3d::exp(gyro_mid * dt).matrix();
  deltaR_ = Utility::OrthoRotMatrix(deltaR_last * inc_R);

  // update preintegrate position
  Eigen::Vector3d acc_mid = 0.5 * (deltaR_last * acc_last + deltaR_ * acc);
  deltaP_ = deltaP_ + deltaV_ * dt + 0.5 * acc_mid * dt * dt;

  // update preintegrate velocity
  deltaV_ = deltaV_ + acc_mid * dt;

  ///////////////////// covariance propagation /////////////////////
  Eigen::Matrix3d gyro_hat = SO3d::hat(gyro_mid);
  Eigen::Matrix3d acc_last_hat = SO3d::hat(acc_last);
  Eigen::Matrix3d acc_hat = SO3d::hat(acc);
  Eigen::Matrix3d I33 = Eigen::Matrix3d::Identity();

  Eigen::Matrix15d A;
  A.setZero();
  // state P
  A.block<3, 3>(O_P, O_P) = I33;
  A.block<3, 3>(O_P, O_R) =
      -0.25 * deltaR_last * acc_last_hat * dt * dt +
      -0.25 * deltaR_ * acc_hat * (I33 - gyro_hat * dt) * dt * dt;
  A.block<3, 3>(O_P, O_V) = I33 * dt;
  A.block<3, 3>(O_P, O_BA) = -0.25 * (deltaR_last + deltaR_) * dt * dt;
  A.block<3, 3>(O_P, O_BG) = 0.25 * deltaR_ * acc_hat * dt * dt * dt;
  // state R
  A.block<3, 3>(O_R, O_R) = I33 - gyro_hat * dt;
  A.block<3, 3>(O_R, O_BG) = -1.0 * I33 * dt;
  // state V
  A.block<3, 3>(O_V, O_R) =
      -0.5 * deltaR_last * acc_last_hat * dt +
      -0.5 * deltaR_ * acc_hat * (I33 - gyro_hat * dt) * dt;
  A.block<3, 3>(O_V, O_V) = I33;
  A.block<3, 3>(O_V, O_BA) = -0.5 * (deltaR_last + deltaR_) * dt;
  A.block<3, 3>(O_V, O_BG) = 0.5 * deltaR_ * acc_hat * dt * dt;
  // state BA and BG
  A.block<3, 3>(O_BA, O_BA) = I33;
  A.block<3, 3>(O_BG, O_BG) = I33;

  Eigen::Matrix<double, 15, 18> B;
  B.setZero();
  // state P
  B.block<3, 3>(O_P, 0) = 0.25 * deltaR_last * dt * dt;
  B.block<3, 3>(O_P, 3) = 0.25 * -deltaR_ * acc_hat * dt * dt * 0.5 * dt;
  B.block<3, 3>(O_P, 6) = 0.25 * deltaR_ * dt * dt;
  B.block<3, 3>(O_P, 9) = B.block<3, 3>(O_P, 3);
  // state R
  B.block<3, 3>(O_R, 3) = 0.5 * I33 * dt;
  B.block<3, 3>(O_R, 9) = 0.5 * I33 * dt;
  // state O_V
  B.block<3, 3>(O_V, 0) = 0.5 * deltaR_last * dt;
  B.block<3, 3>(O_V, 3) = 0.5 * -deltaR_ * acc_hat * dt * 0.5 * dt;
  B.block<3, 3>(O_V, 6) = 0.5 * deltaR_ * dt;
  B.block<3, 3>(O_V, 9) = B.block<3, 3>(O_V, 3);
  // state BA and BG
  B.block<3, 3>(O_BA, 12) = I33 * dt;
  B.block<3, 3>(O_BG, 15) = I33 * dt;

  Eigen::Matrix18d noise = Eigen::Matrix18d::Zero();
  noise.block<3, 3>(0, 0) = (acc_n * acc_n) * I33;
  noise.block<3, 3>(3, 3) = (gyro_n * gyro_n) * I33;
  noise.block<3, 3>(6, 6) = (acc_n * acc_n) * I33;
  noise.block<3, 3>(9, 9) = (gyro_n * gyro_n) * I33;
  noise.block<3, 3>(12, 12) = (acc_w * acc_w) * I33;
  noise.block<3, 3>(15, 15) = (gyro_w * gyro_w) * I33;

  // propagate
  jacobian_ = A * jacobian_;
  covariance_ = A * covariance_ * A.transpose() + B * noise * B.transpose();

  // buffer imu meta data
  imu_metas_.emplace_back(dt, gyro, acc, gyro_n, gyro_w, acc_n, acc_w);
}

void IMUPreintMeasurement::ReUpdate(const Eigen::Vector3d& dBg,
                                    const Eigen::Vector3d& dBa, bool use_jac) {
  if (use_jac) {
    deltaR_ = deltaR_ * SO3d::exp(GetJacdRdbg() * dBg).matrix();
    deltaV_ = deltaV_ + GetJacdVdbg() * dBg + GetJacdVdba() * dBa;
    deltaP_ = deltaP_ + GetJacdPdbg() * dBg + GetJacdPdba() * dBa;
  } else {
    std::vector<IMUMeta> imu_metas_backup;
    imu_metas_backup.swap(imu_metas_);
    Init();
    for (size_t i = 0; i < imu_metas_backup.size(); ++i) {
      const auto& data = imu_metas_backup[i];
      Update(data.gyro - dBg, data.gyro_n, data.gyro_w, data.acc - dBa,
             data.acc_n, data.acc_w, data.dt);
    }
  }
}

}  // namespace localization
}  // namespace senseAD
