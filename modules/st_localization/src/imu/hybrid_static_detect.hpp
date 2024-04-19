/*
 * Copyright (C) 2023 by SenseTime Group Limited. All rights reserved.
 * Lei Bing <leibing@senseauto.com>
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <list>
#include <memory>
#include <utility>

#include <boost/circular_buffer.hpp>

#include "ad_log/ad_log.hpp"
#include "common/msf_common.hpp"
#include "localization/data_type/base.hpp"
#include "localization/data_type/imu.hpp"
#include "localization/data_type/vehicle_info.hpp"

namespace senseAD {
namespace localization {

enum MotionState { DRIVING, BRAKING, PARKING, STARTING, SHAKING };

class HybridStaticDetector {
 public:
  explicit HybridStaticDetector(const size_t& imu_freq);
  ~HybridStaticDetector() = default;

  adLocStatus_t Init();

  adLocStatus_t SetCanData(const uint64_t& timestamp, const VehicleInfo& can);

  adLocStatus_t SetImuData(const uint64_t& timestamp, const Imu& imu);

  adLocStatus_t SetLocalizationSpeed(const double& speed);

  adLocStatus_t StaticDetect(const double& var_threshold);

 private:
  adLocStatus_t InitDataBuffer();

  adLocStatus_t SyncImuAndCanData();

  double ComputeGLR(const double& acc_sigma = 1, const double& gyro_sigma = 0.1,
                    const double& wss_sigma = 0.005);

  double ComputeAccSTD();

 private:
  // param
  const size_t kDataSizeUpperLimit = 20;          // can buffer size
  const uint64_t shaking_check_gap_ = 0.2 * 1e9;  // timegap to check shaking
  const int state_remain_num_ = 3;                // for safely detect
  const int parking_time_secs_ = 3;               // time for calculate acc std
  const int parking_std_est_secs_ = 2;            // time for estimating ave std
  const double lon_acc_threshold_ = 0.2;
  const double parking_std_coeff_ = 1.5;
  const double wsss_ = 0.05;  // wheel speed static speed
  const double ws_blind_zone_ = 0.2;
  const double glr_blind_coeff_ = 0.2;  // for detecet static timely
  const double glr_braking_threshold_ = 0.1;
  const double lon_acc_starting_threshold_ = 0.1;
  const double g_norm_ = 9.807;
  const double localization_speed_threshold_ = 0.2;

  double localization_speed_ = 0;

  size_t imu_freq_;  // imu frequency
  uint64_t cur_timestamp_ = 0;
  uint64_t last_timestamp_ = 0;

  const size_t mov_ave_size_ = 3;  // size of calculating average
  boost::circular_buffer<Eigen::Matrix<double, 6, 1>> imu_mov_buffer_;
  boost::circular_buffer<double> wss_mov_buffer_;

  size_t detect_buffer_size_;  // detect buffer size
  boost::circular_buffer<Eigen::Matrix<double, 6, 1>> imu_detect_buffer_;
  boost::circular_buffer<double> wss_detect_buffer_;

  // motion state
  MotionState motion_state_ = DRIVING;
  MotionState last_motion_state_ = DRIVING;

  // braking param
  int in_braking_num_ = 0;

  // parking param
  int in_parking_num_ = 0;
  double std_ave_parking_last_ = 0.05;
  std::list<double> parking_std_list_;
  bool parking_bias_est_once_ = false;
  Eigen::Matrix<double, 6, 1> cur_parking_bias_;
  Eigen::Matrix<double, 6, 1> last_parking_bias_;

  // shaking param
  int in_shaking_num_ = 0;
  uint64_t last_shaking_timestamp_ = 0;
  uint64_t start_shaking_timestamp_ = 0;
  double start_shaking_acc_y_ = 0;
  bool constant_shaking_ = false;

  // buffer can data, for zero-speed detection
  std::mutex can_mutex_;
  boost::circular_buffer<std::pair<uint64_t, VehicleInfo>> can_data_list_;

  // imu data, for zero-speed detection
  std::mutex imu_mutex_;
  std::pair<uint64_t, std::shared_ptr<Imu>> cur_imu_ =
      std::make_pair(0, nullptr);
};

}  // namespace localization
}  // namespace senseAD
