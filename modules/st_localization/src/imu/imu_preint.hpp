/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Fang Yu <fangyu1@senseauto.com>
 */
#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <memory>
#include <vector>

#include <Sophus/so3.hpp>

#include "common/utility.hpp"

namespace Eigen {
using Matrix1d = Matrix<double, 1, 1>;
using Vector6d = Matrix<double, 6, 1>;
using Matrix6d = Matrix<double, 6, 6>;
using Vector9d = Matrix<double, 9, 1>;
using Matrix93d = Matrix<double, 9, 3>;
using Matrix96d = Matrix<double, 9, 6>;
using Matrix9d = Matrix<double, 9, 9>;
using Vector15d = Matrix<double, 15, 1>;
using Matrix153d = Matrix<double, 15, 3>;
using Matrix156d = Matrix<double, 15, 6>;
using Matrix159d = Matrix<double, 15, 9>;
using Matrix15d = Matrix<double, 15, 15>;
using Matrix18d = Matrix<double, 18, 18>;
}  // namespace Eigen

namespace senseAD {
namespace localization {

// class holds the imu preintegrate measurement
// implement refers to paper "IMU Preintegration on Manifold For Efficient
// Visual-Inertial Maximum-a-Posteriori Estimation"
class IMUPreintMeasurement {
 public:
  using Ptr = std::shared_ptr<IMUPreintMeasurement>;

  // structs holds the imu raw meta data
  struct IMUMeta {
    IMUMeta() {}
    IMUMeta(double dt, const Eigen::Vector3d& gyro, const Eigen::Vector3d& acc,
            double gyro_n, double gyro_w, double acc_n, double acc_w)
        : dt(dt),
          gyro(gyro),
          acc(acc),
          gyro_n(gyro_n),
          gyro_w(gyro_w),
          acc_n(acc_n),
          acc_w(acc_w) {}
    double dt;                            // delta time
    Eigen::Vector3d gyro, acc;            // raw measure data
    double gyro_n, gyro_w, acc_n, acc_w;  // noise data
  };

 public:
  IMUPreintMeasurement();
  IMUPreintMeasurement(const IMUPreintMeasurement& copy);
  ~IMUPreintMeasurement() = default;

  IMUPreintMeasurement& operator=(const IMUPreintMeasurement& copy);

  // @brief: init
  void Init();

  // @brief: update preintegrate
  void Update(const Eigen::Vector3d& gyro, double gyro_n, double gyro_w,
              const Eigen::Vector3d& acc, double acc_n, double acc_w,
              double dt);

  // @brief: re-update preintegrate, support jacobian update or repropogate
  // raw imu datas
  void ReUpdate(const Eigen::Vector3d& dBg, const Eigen::Vector3d& dBa,
                bool use_jac = false);

  // @brief: get preintegrate result
  Eigen::Matrix3d GetDeltaR() const { return deltaR_; }
  Eigen::Vector3d GetDeltaP() const { return deltaP_; }
  Eigen::Vector3d GetDeltaV() const { return deltaV_; }
  double GetDeltaT() const { return deltaT_; }
  Eigen::Matrix9d GetCovariance() const {
    return covariance_.topLeftCorner<9, 9>();
  }
  Eigen::Matrix6d GetBiasCovariance() const {
    return covariance_.bottomRightCorner<6, 6>();
  }
  Eigen::Matrix15d GetCovarianceFull() const { return covariance_; }

  // @brief: get jacobian wrt. gyro and acc bias
  Eigen::Matrix3d GetJacdRdbg() const {
    return jacobian_.block<3, 3>(O_R, O_BG);
  }
  Eigen::Matrix3d GetJacdVdbg() const {
    return jacobian_.block<3, 3>(O_V, O_BG);
  }
  Eigen::Matrix3d GetJacdVdba() const {
    return jacobian_.block<3, 3>(O_V, O_BA);
  }
  Eigen::Matrix3d GetJacdPdbg() const {
    return jacobian_.block<3, 3>(O_P, O_BG);
  }
  Eigen::Matrix3d GetJacdPdba() const {
    return jacobian_.block<3, 3>(O_P, O_BA);
  }

  // @brief: get raw imu meta datas
  const std::vector<IMUMeta>& GetIMUMetaDatas() const { return imu_metas_; }

 private:
  // state order
  static constexpr int O_P = 0;
  static constexpr int O_R = 3;
  static constexpr int O_V = 6;
  static constexpr int O_BG = 9;
  static constexpr int O_BA = 12;

  Eigen::Matrix3d deltaR_;       // preintegrate attitude
  Eigen::Vector3d deltaP_;       // preintegrate position
  Eigen::Vector3d deltaV_;       // preintegrate velocity
  double deltaT_;                // preintegrate time
  Eigen::Matrix15d covariance_;  // preintegrate covariance (R, P, V, Bg, Ba)
  Eigen::Matrix15d jacobian_;    // preintegrate jacobian (R, P, V, Bg, Ba)

  std::vector<IMUMeta> imu_metas_;  // buffer for imu meta datas
};

}  // namespace localization
}  // namespace senseAD
