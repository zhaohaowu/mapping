/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： kalman_filter.cc
 *   author     ： zhangyu0435
 *   date       ： 2023.10
 ******************************************************************************/
#include "modules/location/fusion_center/lib/kalman_filter.h"
#include <yaml-cpp/yaml.h>

#include <boost/filesystem.hpp>

#include "modules/util/include/util/mapping_log.h"

namespace hozon {
namespace mp {
namespace loc {
namespace fc {

bool KalmanFilter::Init(const std::string& configfile) {
  boost::filesystem::path path(configfile);
  if (!boost::filesystem::exists(path)) {
    HLOG_ERROR << "kalman filter configfile error:" << configfile
               << " not exist";
    return false;
  }

  YAML::Node node = YAML::LoadFile(configfile);

  std::string key = "angle_sign_change_thr";
  if (!node[key]) {
    HLOG_ERROR << "can not find " << key;
    return false;
  }
  angle_sign_change_thr_ = node[key].as<double>();

  key = "P";
  if (!node[key]) {
    HLOG_ERROR << "can not find " << key;
    return false;
  }
  const auto P_raw = node[key].as<std::vector<double>>();
  if (P_raw.size() != 36) {
    HLOG_ERROR << key << " dimension should be 6x6";
    return false;
  }
  Eigen::Matrix<double, 6, 6> P_tmp(P_raw.data());
  P_ = P_tmp;

  key = "Q";
  if (!node[key]) {
    HLOG_ERROR << "can not find " << key;
    return false;
  }
  const auto Q_raw = node[key].as<std::vector<double>>();
  if (Q_raw.size() != 36) {
    HLOG_ERROR << key << " dimension should be 6x6";
    return false;
  }
  Eigen::Matrix<double, 6, 6> Q_tmp(Q_raw.data());
  Q_ = Q_tmp;

  key = "H";
  if (!node[key]) {
    HLOG_ERROR << "can not find " << key;
    return false;
  }
  const auto H_raw = node[key].as<std::vector<double>>();
  if (H_raw.size() != 36) {
    HLOG_ERROR << key << " dimension should be 6x6";
    return false;
  }
  Eigen::Matrix<double, 6, 6> H_tmp(H_raw.data());
  H_ = H_tmp;

  key = "R";
  if (!node[key]) {
    HLOG_ERROR << "can not find " << key;
    return false;
  }
  const auto R_raw = node[key].as<std::vector<double>>();
  if (R_raw.size() != 36) {
    HLOG_ERROR << key << " dimension should be 6x6";
    return false;
  }
  Eigen::Matrix<double, 6, 6> R_tmp(R_raw.data());
  R_ = R_tmp;

  configfile_ = configfile;

  return true;
}

void KalmanFilter::Reinit() {
  if (!init_ || !Init(configfile_)) {
    HLOG_ERROR << "kalman reinit failed";
    return;
  }

  init_ = false;
}

void KalmanFilter::SetInitialState(const Eigen::VectorXd& state) {
  state_ = state;
  init_ = true;
}

void KalmanFilter::SetEnuState(const Eigen::Vector3d& state) {
  state_[0] = state[0];
  state_[1] = state[1];
  state_[2] = state[2];
}

void KalmanFilter::SetF(const Eigen::MatrixXd& F) { F_ = F; }

void KalmanFilter::Predict(double t, double vx, double vy, double vz,
                           double avy) {
  // 车体系下线速度和角速度
  Eigen::Vector3d liner_vel_VRF = {vx, vy, vz};
  Eigen::Vector3d delta_dist = liner_vel_VRF * t;
  Eigen::Vector3d angular_vel_VRF = {0, 0, avy};
  Eigen::Vector3d delta_angle = angular_vel_VRF * t;
  // 1.状态预测
  Sophus::SO3d state_rot = Sophus::SO3d::exp(state_.tail(3));
  state_.head(3) = state_.head(3) + state_rot.matrix() * delta_dist;
  state_.tail(3) = (state_rot * Sophus::SO3d::exp(delta_angle)).log();

  // 2.计算F
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
  F.template block<3, 3>(0, 3) =
      -1 * state_rot.matrix() * SkewMatrix(angular_vel_VRF);
  F.template block<3, 1>(0, 3) = Eigen::Vector3d::Zero();
  F.template block<3, 1>(0, 4) = Eigen::Vector3d::Zero();
  Sophus::SO3d delta_rot = Sophus::SO3d::exp(angular_vel_VRF);
  Eigen::Vector3d JrSO3_input = (state_rot * delta_rot).log();
  F.template block<3, 3>(3, 3) =
      JrSO3(JrSO3_input).inverse() * delta_rot.matrix().transpose();
  F.template block<3, 1>(3, 3) = Eigen::Vector3d::Zero();
  F.template block<3, 1>(3, 4) = Eigen::Vector3d::Zero();
  F.template block<2, 1>(3, 5) = Eigen::Vector2d::Zero();
  SetF(F);

  // 3.P预测
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::MeasurementUpdate(const Eigen::VectorXd& z) {
  auto meas = z;

  Sophus::SO3d meas_rot = Sophus::SO3d::exp(meas.tail(3));
  Sophus::SO3d state_rot = Sophus::SO3d::exp(state_.tail(3));
  Eigen::VectorXd y_diff(6, 1);
  y_diff.template block<3, 1>(0, 0) = meas.head(3) - state_.head(3);
  y_diff.template block<3, 1>(3, 0) = (state_rot.inverse() * meas_rot).log();

  const auto S = H_ * P_ * H_.transpose() + R_;
  const auto K = P_ * H_.transpose() * S.inverse();
  auto K_ydiff = K * y_diff;
  state_.head(3) = state_.head(3) + K_ydiff.head(3);
  state_.tail(3) = (state_rot * Sophus::SO3d::exp(K_ydiff.tail(3))).log();
  const int size = state_.size();
  const auto I = Eigen::MatrixXd::Identity(size, size);
  P_ = (I - K * H_) * P_;
}

Eigen::VectorXd KalmanFilter::GetState() const { return state_; }

bool KalmanFilter::IsInitialized() const { return init_; }

Eigen::Matrix<double, 3, 3> KalmanFilter::JlSO3(
    const Eigen::Matrix<double, 3, 1>& w) {
  double theta = w.norm();
  if (theta < 1e-6) {
    return Eigen::MatrixXd::Identity(3, 3);
  } else {
    Eigen::Matrix<double, 3, 1> a = w / theta;
    Eigen::Matrix<double, 3, 3> J =
        sin(theta) / theta * Eigen::MatrixXd::Identity(3, 3) +
        (1 - sin(theta) / theta) * a * a.transpose() +
        ((1 - cos(theta)) / theta) * SkewMatrix(a);
    return J;
  }
}

Eigen::Matrix<double, 3, 3> KalmanFilter::JrSO3(
    const Eigen::Matrix<double, 3, 1>& w) {
  return JlSO3(-w);
}

Eigen::Matrix<double, 3, 3> KalmanFilter::SkewMatrix(Eigen::Vector3d v) {
  Eigen::Matrix<double, 3, 3> m;
  m << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  return m;
}

}  // namespace fc
}  // namespace loc
}  // namespace mp
}  // namespace hozon
