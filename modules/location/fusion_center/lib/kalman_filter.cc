/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： kalman_filter.cc
 *   author     ： zhangyu0435
 *   date       ： 2023.10
 ******************************************************************************/
#include "modules/location/fusion_center/lib/kalman_filter.h"

#include <yaml-cpp/yaml.h>

#include <boost/filesystem.hpp>

#include "modules/util/include/util/temp_log.h"

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
  if (P_raw.size() != 16) {
    HLOG_ERROR << key << " dimension should be 4x4";
    return false;
  }
  Eigen::Matrix<double, 4, 4> P_tmp(P_raw.data());
  P_ = P_tmp;

  key = "Q";
  if (!node[key]) {
    HLOG_ERROR << "can not find " << key;
    return false;
  }
  const auto Q_raw = node[key].as<std::vector<double>>();
  if (Q_raw.size() != 16) {
    HLOG_ERROR << key << " dimension should be 6x6";
    return false;
  }
  Eigen::Matrix<double, 4, 4> Q_tmp(Q_raw.data());
  Q_ = Q_tmp;

  key = "H";
  if (!node[key]) {
    HLOG_ERROR << "can not find " << key;
    return false;
  }
  const auto H_raw = node[key].as<std::vector<double>>();
  if (H_raw.size() != 16) {
    HLOG_ERROR << key << " dimension should be 6x6";
    return false;
  }
  Eigen::Matrix<double, 4, 4> H_tmp(H_raw.data());
  H_ = H_tmp;

  key = "R";
  if (!node[key]) {
    HLOG_ERROR << "can not find " << key;
    return false;
  }
  const auto R_raw = node[key].as<std::vector<double>>();
  if (R_raw.size() != 16) {
    HLOG_ERROR << key << " dimension should be 6x6";
    return false;
  }
  Eigen::Matrix<double, 4, 4> R_tmp(R_raw.data());
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

void KalmanFilter::SetF(const Eigen::MatrixXd& F) { F_ = F; }

void KalmanFilter::Predict(double t, double ve, double vn, double vu,
                           double avy) {
  const double delta_e = t * ve;
  const double delta_n = t * vn;
  const double delta_u = t * vu;
  const double delta_yaw = t * avy;

  Eigen::VectorXd u(4, 1);
  u << delta_e, delta_n, delta_u, delta_yaw;
  state_ = F_ * state_ + u;

  if (state_(3) > 360.0) {
    state_(3) -= 360.0;
  } else if (state_(3) < 0.0) {
    state_(3) += 360.0;
  }

  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::MeasurementUpdate(const Eigen::VectorXd& z) {
  auto meas = z;
  if (fabs(state_(3) - meas(3)) > angle_sign_change_thr_) {
    if (state_(3) < 180.0 && meas(3) > 180.0) {
      meas(3) -= 360.0;
    } else if (state_(3) > 180.0 && meas(3) < 180.0) {
      meas(3) += 360.0;
    }
  }

  const auto y = meas - H_ * state_;
  const auto S = H_ * P_ * H_.transpose() + R_;
  const auto K = P_ * H_.transpose() * S.inverse();
  state_ = state_ + (K * y);
  const int size = state_.size();
  const auto I = Eigen::MatrixXd::Identity(size, size);
  P_ = (I - K * H_) * P_;
}

Eigen::VectorXd KalmanFilter::GetState() const { return state_; }

bool KalmanFilter::IsInitialized() const { return init_; }

}  // namespace fc
}  // namespace loc
}  // namespace mp
}  // namespace hozon
