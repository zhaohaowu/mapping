/******************************************************************************
 * Copyright (C) 2022 HOZON-AUTO Ltd. All rights reserved.
 * Licensed Hozon
 * Author: pengwei
 ******************************************************************************/
#pragma once

#include <deque>
#include <memory>
#include <numeric>
#include <queue>
#include <tuple>
#include <utility>
#include <vector>

#include "base/utils/log.h"
#include "modules/dr/include/odometry_base.h"

namespace hozon {
namespace mp {
namespace dr {

class Odometry2D : public OdometryBase {
 public:
  Odometry2D(Eigen::Vector3d extrinsic_a = {0, 0, 0},
             Eigen::Vector3d extrinsic_t = {1.810, 0, 0.380})
      : pos_({0, 0, 0}),
        vel_(0, 0, 0),
        acc_bias_(0, 0, 0),
        qat_(1, 0, 0, 0),
        gyro_bias_(0, 0, 0),
        gravity_(0, 0, 0),
        is_car_standstill_(true),
        car_standstill_counter_(0),
        car_standstill_omg_sum_(0, 0, 0),
        w_by_gyro_(0, 0, 0),
        acc_by_gyro_(0, 0, 0),
        w_by_wheel_(0),
        acc_by_wheel_(0) {
    // car_speed_corrector_._len_sw = 0;
    // car_speed_corrector_._chi_thr = 1.0;
    // car_speed_corrector_._gain_coef.setConstant(12, 1, 1.0);
    // car_speed_corrector_._gain_coef(8) = 0.0;
    // car_speed_corrector_._gain_coef(11) = 0.0;

    // yaw_cov_corrector_._len_sw = 0;
    // yaw_cov_corrector_._chi_thr = 1.0;

    // standstill_corrector_._len_sw = 0;
    // standstill_corrector_._chi_thr = 1.0;

    error_state_ = Eigen::VectorXd::Zero(STATE_NUMBER);
    error_state_cov_ = Eigen::MatrixXd::Identity(STATE_NUMBER, STATE_NUMBER);

    gravity_ << 0, 0, -CON_g0;

    extrinsic_Q_ = Eigen::AngleAxisd(extrinsic_a[2], Eigen::Vector3d::UnitZ()) *
                   Eigen::AngleAxisd(extrinsic_a[1], Eigen::Vector3d::UnitY()) *
                   Eigen::AngleAxisd(extrinsic_a[0], Eigen::Vector3d::UnitX());
    extrinsic_T_ = extrinsic_t;
  }

  ~Odometry2D() {}

  virtual bool update();
  bool CanInitialize() {
    return get_imu_data_size() >= 20 && get_wheel_data_size() > 20;
  }
  Eigen::MatrixX3d get_cross_mat(const Eigen::Vector3d& w) {
    double wx = w(0, 0);
    double wy = w(1, 0);
    double wz = w(2, 0);
    Eigen::MatrixXd a_mat(3, 3);
    a_mat << 0.0, -wz, wy, wz, 0.0, -wx, -wy, wx, 0.0;
    return a_mat;
  }

  bool Initialize();

  std::tuple<Eigen::Vector3d, double> UpdatePosByWheel(
      const WheelDataHozon& last, const WheelDataHozon& cur);
  void UpdateOrientationByIMU(const ImuDataHozon& last_imu,
                              const ImuDataHozon& cur_imu);
  // enum SlopeState UpdateSlopeState(const ImuDataHozon& cur_imu);
  void local_yaw_cov_update();
  // void local_car_speed_correct(Eigen::Vector3d vel);
  void local_state_update();
  bool EstimatedRollPitch();

 private:
  int mode_;
  Eigen::Vector3d pos_;
  Eigen::Vector3d vel_;
  Eigen::Vector3d acc_bias_;
  Eigen::Quaterniond qat_;
  Eigen::Vector3d gyro_bias_;

  Eigen::Vector3d gravity_;
  bool is_car_standstill_;
  unsigned int car_standstill_counter_;
  Eigen::Vector3d car_standstill_omg_sum_;

  Eigen::Vector3d w_by_gyro_;
  Eigen::Vector3d acc_by_gyro_;
  double w_by_wheel_;
  double acc_by_wheel_;
  std::deque<std::pair<double, double>> wheel_vel_buffer_;

  // KalmanFilterCorrector car_speed_corrector_;
  // KalmanFilterCorrector yaw_cov_corrector_;
  // KalmanFilterCorrector standstill_corrector_;
  // KalmanFilterCorrector plane_contrasint_corrector_;

  Eigen::VectorXd error_state_;      // vel, acc_bias, ru, gyro_bias
  Eigen::MatrixXd error_state_cov_;  // coviance

  Eigen::Quaterniond extrinsic_Q_;
  Eigen::Vector3d extrinsic_T_;

  Eigen::Quaterniond qat_3D_;
  std::vector<Eigen::Vector3d> imu_acc_buffer_;

  // bool enable_underground_garage_ = false;
  // enum SlopeState state_ = SlopeState::NONE;

  std::deque<Eigen::Vector3d> ins_buffer_;
  double avg_pitch_ = 0.0;
};

}  // namespace dr
}  // namespace mp
}  // namespace hozon
