/*
 * Copyright (C) 2021 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */
#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <deque>
#include <memory>
#include <vector>

#include "common/transform_config.hpp"
#include "localization/common/log.hpp"

namespace senseAD {
namespace localization {

// class holds the imu initialization that assumes the imu starts from standing
// still, will initialze the imu state, bias, gravity etc at stationary
class IMUStaticInitializer {
 public:
  using Ptr = std::shared_ptr<IMUStaticInitializer>;

  // struct holds the system related state at initialization for static init
  struct InitStateReport {
    Eigen::Matrix3d Rwi0;  // rotation w.r.t. world-frame at initialization
    Eigen::Vector3d Pwi0;  // position w.r.t. world-frame at initialization
    Eigen::Vector3d Vwi0;  // velocity w.r.t. world-frame at initialization
    Eigen::Vector3d Bg;    // gyro bias
    Eigen::Vector3d Ba;    // accelerate bias
    Eigen::Vector3d g;     // gravity
  };

 public:
  IMUStaticInitializer() = default;
  ~IMUStaticInitializer() = default;

  // @brief: set param
  void SetGravityNorm(double g_norm) { gravity_norm_ = g_norm; }
  void SetIMUExciteThre(double imu_excite_thre) {
    imu_excite_thre_ = imu_excite_thre;
  }
  void SetImuWindowLength(double window_length) {
    imu_window_length_ = window_length;
  }
  void SetAccFilterWindowLength(size_t window_length) {
    acc_filter_window_length_ = window_length;
  }
  // @brief: get initial state report
  const InitStateReport& GetInitStateReport() const {
    LC_LDEBUG(IMU) << "initial report: "
                   << "\n   Rwi0:\n"
                   << report_.Rwi0 << "\n   Pwi0: " << report_.Pwi0.transpose()
                   << "\n   Vwi0: " << report_.Vwi0.transpose()
                   << "\n   Bg: " << report_.Bg.transpose()
                   << "\n   Ba: " << report_.Ba.transpose();
    return report_;
  }

  // @brief: initialize with imu datas
  bool Initialize(const Eigen::Vector3d& gyro, const Eigen::Vector3d& acc,
                  double dt) {
    // acc mean filter to remove high frequency noise
    Eigen::Vector3d filtered_acc = AccMeanFilter(acc);

    accumulate_times_ += dt;
    imu_datas_.emplace_back(IMUMeta(dt, gyro, filtered_acc));

    // delete all measurements older than 1.2 of our initialization windows
    auto iter = imu_datas_.begin();
    while (iter != imu_datas_.end() &&
           accumulate_times_ > imu_window_length_ * 1.2) {
      iter = imu_datas_.erase(iter);
      accumulate_times_ -= iter->dt;
    }

    // not enough imu datas
    if (accumulate_times_ < imu_window_length_) return false;

    // calculate the sample variance for the slide window
    Eigen::Vector3d acc_mean;
    double acc_var;
    CalIMUAccMeanVariance(imu_datas_, &acc_mean, &acc_var);
    if (acc_var >= imu_excite_thre_) {
      LC_LDEBUG_EVERY_SEC(ODOM, 1)
          << "imu initialize failed, too much excitation, acc_var: " << acc_var;
      return false;
    }

    // extrinsic from imu to car-center
    Eigen::Matrix4d Tdi = TransformConfig::GetTvb().matrix();

    InitStateReport init_state_report;

    // gravity align (world-frame as ENU, locate at car-center)
    Eigen::Vector3d gw_axis(0, 0, -1);
    Eigen::Vector3d gi_axis = -acc_mean / acc_mean.norm();
    init_state_report.g = gi_axis * gravity_norm_;

    Eigen::Vector3d gd_axis = Tdi.block<3, 3>(0, 0) * gi_axis;
    Eigen::Vector3d rot_axis = gd_axis.cross(gw_axis);  // angle axis
    rot_axis.normalize();
    double alpha = std::acos(gd_axis.dot(gw_axis));  // angle scalar
    // rotation from first car-center-frame to world-frame
    Eigen::Matrix3d Rwd0 = Eigen::AngleAxisd(alpha, rot_axis).matrix();

    // PVA from first imu-frame to world-frame
    init_state_report.Rwi0 = Rwd0 * Tdi.block<3, 3>(0, 0);
    init_state_report.Pwi0 = Rwd0 * Tdi.block<3, 1>(0, 3);
    init_state_report.Vwi0 = Eigen::Vector3d::Zero();

    // accelerate bias refine
    init_state_report.Ba = Eigen::Vector3d::Zero();

    // gyro bias refine
    Eigen::Vector3d Bg_avg = Eigen::Vector3d::Zero();
    for (const auto& data : imu_datas_) Bg_avg += data.gyro;
    Bg_avg /= imu_datas_.size();
    init_state_report.Bg = Bg_avg;

    report_ = init_state_report;

    return true;
  }

 private:
  // structs holds the imu raw meta data
  struct IMUMeta {
    IMUMeta() {}
    IMUMeta(double dt, const Eigen::Vector3d& gyro, const Eigen::Vector3d& acc)
        : dt(dt), gyro(gyro), acc(acc) {}
    double dt;                  // delta time
    Eigen::Vector3d gyro, acc;  // raw measure data
  };

  // @brief: acc mean filter
  Eigen::Vector3d AccMeanFilter(const Eigen::Vector3d& acc) {
    acc_filter_deque_.emplace_back(acc);
    if (acc_filter_deque_.size() > acc_filter_window_length_)
      acc_filter_deque_.pop_front();
    // cal mean acc
    Eigen::Vector3d filtered_acc(0, 0, 0);
    for (const auto& item : acc_filter_deque_) filtered_acc += item;
    return filtered_acc / acc_filter_deque_.size();
  }

  // @brief: calculate accelerate data variance spanning a window
  void CalIMUAccMeanVariance(const std::vector<IMUMeta>& imu_datas,
                             Eigen::Vector3d* acc_mean, double* acc_variance) {
    // calculate acc mean
    Eigen::Vector3d acc_avg = Eigen::Vector3d::Zero();
    for (const auto& data : imu_datas) acc_avg += data.acc;
    acc_avg /= imu_datas.size();
    *acc_mean = acc_avg;

    // calculate acc variance
    double acc_var = 0;
    for (const auto& data : imu_datas) {
      acc_var += (data.acc - acc_avg).dot(data.acc - acc_avg);
    }
    acc_var = std::sqrt(acc_var / (imu_datas.size() - 1));
    *acc_variance = acc_var;
  }

 private:
  // param
  double gravity_norm_{9.80665};        // gravity norm, unit: m/s^2
  double imu_excite_thre_{0.02};        // accelerate var threshold
  double imu_window_length_{1.0};       // amount of imu window time, unit: s
  size_t acc_filter_window_length_{3};  // acc filter window length

  // initialize core variable
  double accumulate_times_{0};      // accumulated times, unit: s
  std::vector<IMUMeta> imu_datas_;  // buffer for history imu datas
  std::deque<Eigen::Vector3d> acc_filter_deque_;  // acc filter deque

  InitStateReport report_;  // final initialization report
};

}  // namespace localization
}  // namespace senseAD
