/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： kalman_filter.h
 *   author     ： zhangyu0435
 *   date       ： 2023.10
 ******************************************************************************/
#pragma once

#include <Eigen/Eigen>
#include <string>
#include <vector>

#include <Sophus/se3.hpp>

namespace hozon {
namespace mp {
namespace loc {
namespace fc {

class KalmanFilter {
 public:
  KalmanFilter() = default;
  ~KalmanFilter() = default;

  bool Init(const std::string& configfile);
  void Reinit();
  void SetInitialState(const Eigen::VectorXd& state);
  void SetEnuState(const Eigen::Vector3d& state);
  void SetF(const Eigen::MatrixXd& F);
  void Predict(double t, double vx, double vy, double vz, double avy);
  void MeasurementUpdate(const Eigen::VectorXd& z);
  bool IsInitialized() const;
  Eigen::VectorXd GetState() const;
  Eigen::VectorXd GetKydiff() const;
  Eigen::MatrixXd GetP() const;
  Eigen::Matrix<double, 3, 3> JlSO3(const Eigen::Matrix<double, 3, 1>& w);
  Eigen::Matrix<double, 3, 3> JrSO3(const Eigen::Matrix<double, 3, 1>& w);
  Eigen::Matrix<double, 3, 3> SkewMatrix(Eigen::Vector3d v);

 private:
  bool init_ = false;
  std::string configfile_ = "";
  double angle_sign_change_thr_ = 0.0;
  Eigen::VectorXd state_;
  Eigen::VectorXd Kydiff_;
  Eigen::MatrixXd F_;
  Eigen::MatrixXd P_;
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd H_;
  Eigen::MatrixXd R_;
};

}  // namespace fc
}  // namespace loc
}  // namespace mp
}  // namespace hozon
