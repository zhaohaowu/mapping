/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * Wang Xiaofeng <wangxiaofeng@sensetime.com>
 */

#include "imu/imu_static_detect.hpp"

#include "imu/sins.hpp"

namespace senseAD {
namespace localization {

StaticDetector::StaticDetector(size_t imu_freq) {
  imuz_.clear();
  mov_ave_.clear();
  imu_frequency_ = imu_freq;
  zupt_ws_ = imu_frequency_ * 1;  // about 1 second detection window
  imuz_.set_capacity(zupt_ws_);
  mov_ave_.set_capacity(mov_ws_);
}

adLocStatus_t StaticDetector::addIMU(const Imu& raw_imu) {
  Vector6d imu;
  imu << raw_imu.linear_acceleration.x, raw_imu.linear_acceleration.y,
      raw_imu.linear_acceleration.z, raw_imu.angular_velocity.x,
      raw_imu.angular_velocity.y, raw_imu.angular_velocity.z;

  // moving average window smooth
  mov_ave_.push_back(imu);
  Vector6d ave_imu = Vector6d::Zero();
  if (mov_ave_.size() == mov_ws_) {
    for (auto iter = mov_ave_.begin(); iter != mov_ave_.end(); ++iter) {
      ave_imu += *iter;
    }
    ave_imu /= mov_ws_;
  } else {
    ave_imu = imu;
  }

  imuz_.push_back(ave_imu);
  return LOC_SUCCESS;
}

// Acceleration-Moving Variance Detector
adLocStatus_t StaticDetector::detector_AMV(double var_threshold) {
  if (imuz_.size() < zupt_ws_) {
    LC_LERROR(LOCALIZATION) << "not full static detection window";
    return LOC_LOCALIZATION_ERROR;
  }
  Vector3d ave_a = Vector3d::Zero();
  for (auto iter = imuz_.begin(); iter != imuz_.end(); ++iter) {
    ave_a += iter->segment<3>(0);
  }
  ave_a /= imuz_.size();

  double var_a = 0;
  for (auto iter = imuz_.begin(); iter != imuz_.end(); ++iter) {
    var_a += (iter->segment<3>(0) - ave_a).squaredNorm();
  }
  var_a /= imuz_.size();

  LC_LDEBUG_EVERY_SEC(LOCALIZATION, 1)
      << "static detect AMV, var: " << std::sqrt(var_a);
  if (std::sqrt(var_a) > var_threshold) {
    return LOC_LOCALIZATION_ERROR;
  }

  return LOC_SUCCESS;
}

// Acceleration-Magnitude detector
adLocStatus_t StaticDetector::detector_AMAG() {
  if (imuz_.size() < zupt_ws_) {
    LC_LERROR(LOCALIZATION) << "not full static detection window";
    return LOC_LOCALIZATION_ERROR;
  }
  const double gn = SINS::g_norm_;
  double g_diff = 0.0;
  for (auto iter = imuz_.begin(); iter != imuz_.end(); ++iter) {
    g_diff += std::pow((iter->segment<3>(0).norm() - gn), 2);
  }
  g_diff /= imuz_.size();

  if (g_diff > 0.005) {
    return LOC_LOCALIZATION_ERROR;
  }

  return LOC_SUCCESS;
}

// Angular Rate Energy detector
adLocStatus_t StaticDetector::detector_ARE() {
  if (imuz_.size() < zupt_ws_) {
    LC_LERROR(LOCALIZATION) << "not full static detection window";
    return LOC_LOCALIZATION_ERROR;
  }
  double w_mag = 0.0;
  for (auto iter = imuz_.begin(); iter != imuz_.end(); ++iter) {
    w_mag += iter->segment<3>(3).squaredNorm();
  }
  w_mag /= imuz_.size();

  if (w_mag > 1.0e-5) {
    return LOC_LOCALIZATION_ERROR;
  }

  return LOC_SUCCESS;
}

// Generalized Likelihood Ratio Test
adLocStatus_t StaticDetector::detector_GLRT() {
  if (imuz_.size() < zupt_ws_) {
    LC_LERROR(LOCALIZATION) << "not full static detection window";
    return LOC_LOCALIZATION_ERROR;
  }

  Vector3d ave_a = Vector3d::Zero();
  for (auto iter = imuz_.begin(); iter != imuz_.end(); ++iter) {
    ave_a += iter->segment<3>(0);
  }
  ave_a /= imuz_.size();
  const double gn = SINS::g_norm_;

  double a_multi = 0.0;
  for (auto iter = imuz_.begin(); iter != imuz_.end(); ++iter) {
    a_multi += (iter->segment<3>(0) - gn * ave_a / ave_a.norm()).squaredNorm() +
               iter->segment<3>(3).squaredNorm();
  }
  a_multi /= imuz_.size();

  if (a_multi > 0.002) {
    return LOC_LOCALIZATION_ERROR;
  }

  return LOC_SUCCESS;
}

// increment Delta Position check
adLocStatus_t StaticDetector::detector_DP() {
  if (imuz_.size() < zupt_ws_) {
    LC_LERROR(LOCALIZATION) << "not full static detection window";
    return LOC_LOCALIZATION_ERROR;
  }

  Vector3d ave_a = Vector3d::Zero();
  for (const auto& imu : imuz_) {
    ave_a += imu.segment<3>(0);
  }
  ave_a /= imuz_.size();

  double gravity_norm = ave_a.norm();
  // ENU order
  Vector3d gravity_vector = Vector3d(0.0, 0.0, gravity_norm);

  Quaterniond Qnb = Quaterniond::FromTwoVectors(ave_a, gravity_vector);
  Matrix3d Rnb = Qnb.toRotationMatrix();

  Vector3d dv = Vector3d::Zero();
  Vector3d dp = Vector3d::Zero();
  static double dt = 1.0 / imu_frequency_;
  for (const auto& imu : imuz_) {
    Vector3d an = Rnb * imu.segment<3>(0) - gravity_vector;
    dv += an * dt;
    dp += dv * dt + 0.5 * an * dt * dt;
  }

  if (dp.norm() < 0.0005) {
    return LOC_SUCCESS;
  } else {
    LC_LERROR(LOCALIZATION) << "detected static, but vehicle is moving";
    return LOC_LOCALIZATION_ERROR;
  }

  return LOC_LOCALIZATION_ERROR;
}

}  // namespace localization
}  // namespace senseAD
