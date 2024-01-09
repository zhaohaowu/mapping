/******************************************************************************
 * Copyright (C) 2022 HOZON-AUTO Ltd. All rights reserved.
 * Licensed Hozon
 * Author: pw
 ******************************************************************************/
#pragma once

#include <math.h>

#include <Eigen/Core>
#include <deque>
#include <iostream>
#include <memory>
#include <mutex>  // NOLINT
#include <queue>
#include <string>
#include <vector>

#include "depend/common/time/lock/atomic_rw_lock.h"
#include "depend/perception-base/base/utils/log.h"
#include "modules/dr/include/slam_data_types.h"
#include "yaml-cpp/yaml.h"
namespace hozon {
namespace mp {
namespace dr {

using DRReadLockGuard =
    hozon::common::base::ReadLockGuard<hozon::common::base::AtomicRWLock>;
using DRWriteLockGuard =
    hozon::common::base::WriteLockGuard<hozon::common::base::AtomicRWLock>;

const unsigned int STATE_NUMBER = 12;
const double CON_g0 = 9.80645;
const double EARTH_RADIUS = 6378100.0;  // radius of the earth
const double RAD_2_DEG = 180.0 / 3.1415926;
const double DEG_2_RAD = 1.0 / RAD_2_DEG;

#define MAX_WHEEL_COUNT (32767)

class OdometryBase {
 public:
  struct wheel_intrinsic {
    double kr_;  // 每次脉冲,代表的长度(单位m)
    double kl_;
    double b_;                 // 轮距(1.695)
    double noise_factor_;      // 平面约束的噪声
    double roll_pitch_noise_;  // 水平方向噪声
    double z_noise_;           //
  };

  explicit OdometryBase(const std::string& conf_path)
      : initialized_(false),
        wheel_param_({1.0 / 22.26, 1.0 / 22.26, 1.705, 0.02, 1e-2, 1e-4}) {
    YAML::Node config = YAML::LoadFile(conf_path);

    if (!config) {
      HLOG_ERROR << "Open config File:" << conf_path << " failed.";
      wheel_param_.kr_ = 0.04459;
      wheel_param_.kl_ = 0.04459;
      wheel_param_.b_ = 1.78;
    } else {
      HLOG_INFO << "read param from conf file";
      wheel_param_.kr_ = config["wheel_param_kl"].as<double>();
      wheel_param_.kl_ = config["wheel_param_kr"].as<double>();
      wheel_param_.b_ = config["wheel_param_b"].as<double>();
    }
    HLOG_INFO << "wheel_param_.kr_:" << wheel_param_.kr_;
    HLOG_INFO << "wheel_param_.kl_:" << wheel_param_.kl_;
    HLOG_INFO << "wheel_param_.b_:" << wheel_param_.b_;
  }
  virtual ~OdometryBase() {}
  virtual bool update() = 0;

  void add_imu_data(const ImuDataHozon& imu_data);

  static Eigen::Vector3d filter_vel(const Eigen::Vector3d& cur_vel);
  static double filter_cnt(double cnt);
  bool add_wheel_data(const WheelDataHozon& wheel_data);

  std::vector<WheelDataHozon> get_oldest_two_wheel();
  bool get_imu_before_and_pop(double time,
                              std::vector<ImuDataHozon>& imu_datas);  // NOLINT
  void pop_front_wheel();
  WheelDataHozon get_front_wheel();
  int get_imu_data_size();
  int get_wheel_data_size();
  void clear_imu_wheel_datas();

  OdometryData get_st_odom_data();

  bool GetOdomByTimestamp(double time, WheelOdometry& odometry);  // NOLINT
  bool GetImuByTimestamp(double time, ImuDataHozon& imu_data);    // NOLINT
  bool InterpolatePose(double time, OdometryData& odom_data);     // NOLINT

  OdometryData get_latest_odom_data();
  void AddOdomData(const OdometryData& new_odom /*, double delta_dis*/);

  static void Qat2EulerAngle(const Eigen::Quaterniond& q,
                             double& roll,   // NOLINT
                             double& pitch,  // NOLINT
                             double& yaw);   // NOLINT

  bool Is_Initialized() { return initialized_; }
  void Reset() {}

  // param
  bool initialized_;
  wheel_intrinsic wheel_param_;
  // input
  std::deque<ImuDataHozon> imu_datas_;
  std::deque<WheelDataHozon> wheel_datas_;

 private:
  hozon::common::base::AtomicRWLock imu_rw_lock_;
  hozon::common::base::AtomicRWLock wheel_rw_lock_;
  hozon::common::base::AtomicRWLock odom_rw_lock_;
  // dr
  std::deque<OdometryData> odom_datas_;
};
}  // namespace dr
}  // namespace mp
}  // namespace hozon
