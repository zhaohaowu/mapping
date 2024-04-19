/*
 * Copyright (C) 2023 by SenseTime Group Limited. All rights reserved.
 * Lei Bing <leibing@senseauto.com>
 */

#include "imu/hybrid_static_detect.hpp"

#include <numeric>

#include "common/utility.hpp"
#include "localization/common/log.hpp"

namespace senseAD {
namespace localization {

HybridStaticDetector::HybridStaticDetector(const size_t& imu_freq) {
  imu_freq_ = imu_freq;
  detect_buffer_size_ = imu_freq_;
  imu_mov_buffer_.clear();
  imu_detect_buffer_.clear();
  wss_mov_buffer_.clear();
  wss_detect_buffer_.clear();
  last_parking_bias_.setZero();
}

adLocStatus_t HybridStaticDetector::Init() {
  InitDataBuffer();
  return LOC_SUCCESS;
}

adLocStatus_t HybridStaticDetector::InitDataBuffer() {
  can_data_list_.set_capacity(kDataSizeUpperLimit);
  imu_mov_buffer_.set_capacity(mov_ave_size_);
  wss_mov_buffer_.set_capacity(mov_ave_size_);
  imu_detect_buffer_.set_capacity(detect_buffer_size_);
  wss_detect_buffer_.set_capacity(detect_buffer_size_);
  return LOC_SUCCESS;
}

adLocStatus_t HybridStaticDetector::SetCanData(const uint64_t& timestamp,
                                               const VehicleInfo& can) {
  std::lock_guard<std::mutex> lock(can_mutex_);
  can_data_list_.push_back(std::make_pair(timestamp, can));
  return LOC_SUCCESS;
}

adLocStatus_t HybridStaticDetector::SetImuData(const uint64_t& timestamp,
                                               const Imu& imu) {
  std::lock_guard<std::mutex> lock(imu_mutex_);
  std::shared_ptr<Imu> imu_data = std::make_shared<Imu>(imu);
  cur_imu_ = std::make_pair(timestamp, imu_data);
  return LOC_SUCCESS;
}

adLocStatus_t HybridStaticDetector::SetLocalizationSpeed(const double& speed) {
  localization_speed_ = speed;
  return LOC_SUCCESS;
}

adLocStatus_t HybridStaticDetector::SyncImuAndCanData() {
  uint64_t timestamp = 0;
  std::shared_ptr<Imu> imu = std::make_shared<Imu>();
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    if (cur_imu_.second == nullptr) {
      LC_LDEBUG(IMU) << "imu data unvalid.";
      return LOC_NULL_PTR;
    }
    timestamp = cur_imu_.first;
    imu = cur_imu_.second;
    // reset imu ptr
    cur_imu_.second = nullptr;
  }
  std::shared_ptr<VehicleInfo> can = std::make_shared<VehicleInfo>();
  {
    std::lock_guard<std::mutex> lock(can_mutex_);
    auto iter = can_data_list_.rbegin();
    double timegap =
        static_cast<double>(iter->first) - static_cast<double>(timestamp);
    while (timegap > 0.1 / kNanoSecToSec) {  // +100ms
      ++iter;
      if (iter == can_data_list_.rend()) {
        LC_LDEBUG(IMU) << "can time ahead too much.";
        return LOC_TIME_AHEAD;
      }
      timegap =
          static_cast<double>(iter->first) - static_cast<double>(timestamp);
    }
    if (timegap < -0.1 / kNanoSecToSec) {  // -100 ms
      LC_LDEBUG(IMU) << "can time dylay too much.";
      return LOC_TIME_DELAY;
    }
    *can = iter->second;
  }

  // update imu timestamp
  cur_timestamp_ = timestamp;

  // handle imu
  // y is vehicle's longitudinal direction
  Eigen::Matrix<double, 6, 1> imu_vec;
  imu_vec << imu->linear_acceleration.x, imu->linear_acceleration.y,
      imu->linear_acceleration.z, imu->angular_velocity.x,
      imu->angular_velocity.y, imu->angular_velocity.z;
  imu_mov_buffer_.push_back(imu_vec);
  Eigen::Matrix<double, 6, 1> ave_imu = Eigen::Matrix<double, 6, 1>::Zero();
  if (imu_mov_buffer_.size() < mov_ave_size_) {
    ave_imu = imu_vec;
  } else {
    ave_imu = std::accumulate(imu_mov_buffer_.begin(), imu_mov_buffer_.end(),
                              ave_imu) /
              mov_ave_size_;
  }
  imu_detect_buffer_.push_back(ave_imu);

  // handle can
  double wss = (can->wheel_speed_rl + can->wheel_speed_rr) / 2.0;
  wss_mov_buffer_.push_back(wss);
  double ave_wss = 0;
  if (wss_mov_buffer_.size() < mov_ave_size_) {
    ave_wss = wss;
  } else {
    ave_wss =
        std::accumulate(wss_mov_buffer_.begin(), wss_mov_buffer_.end(), wss) /
        mov_ave_size_;
  }
  wss_detect_buffer_.push_back(ave_wss);

  return LOC_SUCCESS;
}

adLocStatus_t HybridStaticDetector::StaticDetect(const double& var_threshold) {
  // sync data
  adLocStatus_t sync_status = SyncImuAndCanData();
  if (LOC_NULL_PTR == sync_status) {
    LC_LDEBUG(IMU) << "waitting for valid imu data.";
    return LOC_INVALID;
  }
  if (imu_detect_buffer_.size() < detect_buffer_size_ ||
      cur_timestamp_ <= last_timestamp_) {
    return LOC_LOCALIZATION_ERROR;
  }
  bool is_degency = false;
  if (LOC_TIME_DELAY == sync_status || LOC_TIME_AHEAD == sync_status) {
    is_degency = true;  // if only imu
    LC_LDEBUG(IMU) << "failed to sync imu and can.";
  }

  last_timestamp_ = cur_timestamp_;

  // acc std
  double acc_std = ComputeAccSTD();

  if (is_degency && localization_speed_ < localization_speed_threshold_) {
    if (acc_std < var_threshold) {
      motion_state_ = PARKING;
      return LOC_SUCCESS;
    } else {
      motion_state_ = DRIVING;
      return LOC_LOCALIZATION_ERROR;
    }
  }

  // glr
  double glr_ratio = ComputeGLR();

  double cur_wss = wss_detect_buffer_.back();
  auto cur_imu = imu_detect_buffer_.back();

  if (motion_state_ == DRIVING || motion_state_ == BRAKING) {
    // for detect static timely
    if (std::fabs(cur_wss) < ws_blind_zone_) glr_ratio *= glr_blind_coeff_;
    // use AMW to detect in case bias is large
    if (acc_std < var_threshold && std::fabs(cur_wss) < wsss_ &&
        in_braking_num_ == 0) {
      // bias change too much or vehicle is not horizontal
      motion_state_ = BRAKING;
      last_parking_bias_ = cur_imu;
      glr_ratio = ComputeGLR();
    }
  }

  // brake: glr should be small enough in driving
  if (motion_state_ == DRIVING && glr_ratio <= glr_braking_threshold_) {
    if (std::fabs(cur_imu[1] - last_parking_bias_[1]) < lon_acc_threshold_) {
      motion_state_ = BRAKING;
    }
  }

  if (motion_state_ == BRAKING) {
    if (glr_ratio <= glr_braking_threshold_) {
      ++in_braking_num_;
    } else {
      motion_state_ = DRIVING;
      in_braking_num_ = 0;
    }
    if (in_braking_num_ > state_remain_num_ && std::fabs(cur_wss) < wsss_) {
      last_parking_bias_ = cur_imu;
      motion_state_ = PARKING;
    }
  }

  if (motion_state_ == PARKING) {
    if (std::fabs(cur_wss) > wsss_) {
      motion_state_ = DRIVING;
    } else if (acc_std > std_ave_parking_last_ * parking_std_coeff_ &&
               std::fabs(cur_imu[1] - last_parking_bias_[1]) >
                   lon_acc_threshold_) {
      motion_state_ = SHAKING;
      start_shaking_timestamp_ = cur_timestamp_;
      start_shaking_acc_y_ = cur_imu[1] - last_parking_bias_[1];
      in_shaking_num_ = 0;
      if (start_shaking_acc_y_ == 0) start_shaking_acc_y_ = 0.0001;
    }
  }

  if (motion_state_ == SHAKING) {
    if (std::fabs(cur_wss) > wsss_) {
      motion_state_ = DRIVING;
    } else {
      // handle high-freq shaking
      if (cur_timestamp_ < start_shaking_timestamp_ + shaking_check_gap_) {
        if (start_shaking_acc_y_ * (cur_imu[1] - last_parking_bias_[1]) < 0) {
          ++in_shaking_num_;
        }
      } else if (in_shaking_num_ == 0) {
        // if shaking constantly, cannot be judged as starting
        if (!constant_shaking_) {
          if (cur_imu[1] - last_parking_bias_[1] >
              lon_acc_starting_threshold_) {
            motion_state_ = STARTING;
          } else {
            motion_state_ = PARKING;
          }
        } else {
          // if shaking constantly, shaking timestamp should be
          // updated
          start_shaking_timestamp_ = cur_timestamp_;
          start_shaking_acc_y_ = cur_imu[1] - last_parking_bias_[1];
          in_shaking_num_ = 0;
          if (start_shaking_acc_y_ == 0) start_shaking_acc_y_ = 0.0001;
        }
      } else if (in_shaking_num_ > 0) {
        motion_state_ = PARKING;
      }
    }
  }

  if (motion_state_ == STARTING) {
    // recover parking
    if (acc_std < std_ave_parking_last_ * parking_std_coeff_) {
      motion_state_ = PARKING;
    }
    if (std::fabs(cur_wss) > 0) {
      motion_state_ = DRIVING;
    }
  }

  if (motion_state_ == PARKING) {
    ++in_parking_num_;
    // estimate imu bias using 3s data
    if (in_parking_num_ < imu_freq_ * parking_time_secs_ &&
        !parking_bias_est_once_) {
      // just compute once durding same parking
      Eigen::Matrix<double, 6, 1> sum =
          cur_parking_bias_ * (in_parking_num_ - 1) + cur_imu;
      cur_parking_bias_ = sum / in_parking_num_;
    } else if (!parking_bias_est_once_) {
      // if parking time is enough, update imu bias
      last_parking_bias_ = cur_parking_bias_;
      parking_bias_est_once_ = true;
    }

    // compute acc std constantly if parking
    parking_std_list_.emplace_back(acc_std);
    int std_list_size = static_cast<int>(parking_std_list_.size());
    if (std_list_size % (imu_freq_ * parking_time_secs_) == 0) {
      std_ave_parking_last_ = 0;
      parking_std_list_.sort();
      auto start = parking_std_list_.begin();
      auto end = std::next(start, imu_freq_ * parking_std_est_secs_);
      while (start != end) {
        std_ave_parking_last_ += *start;
        ++start;
      }
      std_ave_parking_last_ /= (imu_freq_ * parking_std_est_secs_);
      parking_std_list_.clear();
    }
  } else {  // reset bias-est param if not parking
    in_parking_num_ = 0;
    cur_parking_bias_.setZero();
    parking_std_list_.clear();
    parking_bias_est_once_ = false;
  }

  // shaking ->
  if (motion_state_ != SHAKING && last_motion_state_ == SHAKING) {
    last_shaking_timestamp_ = cur_timestamp_;
  }
  // -> shaking
  if (motion_state_ == SHAKING && last_motion_state_ != SHAKING) {
    constant_shaking_ =
        cur_timestamp_ - last_shaking_timestamp_ > 3 * 1 / imu_freq_ * 1e9
            ? false
            : true;
  }

  last_motion_state_ = motion_state_;

  // output
  switch (motion_state_) {
    case DRIVING:
      in_braking_num_ = 0;
      return LOC_LOCALIZATION_ERROR;
    case BRAKING:
      return LOC_LOCALIZATION_ERROR;
    case PARKING:
      return LOC_SUCCESS;
    case SHAKING:
      return LOC_SUCCESS;  // output starting
    case STARTING:
      in_braking_num_ = 0;
      return LOC_LOCALIZATION_ERROR;
    default:
      break;
  }
  return LOC_LOCALIZATION_ERROR;
}

double HybridStaticDetector::ComputeGLR(const double& acc_sigma,
                                        const double& gyro_sigma,
                                        const double& wss_sigma) {
  double acc_ratio = 0, gyro_ratio = 0, wss_ratio = 0;
  auto imu_iter = imu_detect_buffer_.begin();
  auto wss_iter = wss_detect_buffer_.begin();
  for (; imu_iter != imu_detect_buffer_.end(); ++imu_iter) {
    Eigen::Vector3d acc = imu_iter->segment<3>(0);
    Eigen::Vector3d gyro = imu_iter->segment<3>(3);
    acc_ratio += (acc - g_norm_ * acc / acc.norm()).squaredNorm();
    gyro_ratio += gyro.squaredNorm();
  }
  acc_ratio /= imu_detect_buffer_.size();
  gyro_ratio /= imu_detect_buffer_.size();

  for (; wss_iter != wss_detect_buffer_.end(); ++wss_iter) {
    wss_ratio += (*wss_iter) * (*wss_iter);
  }
  wss_ratio /= wss_detect_buffer_.size();
  return acc_ratio / acc_sigma + gyro_ratio / gyro_sigma +
         wss_ratio / wss_sigma;
}

double HybridStaticDetector::ComputeAccSTD() {
  Eigen::Vector3d ave_a = Eigen::Vector3d::Zero();
  double var_a = 0;
  for (const auto& p : imu_detect_buffer_) {
    ave_a += p.segment<3>(0);
  }
  ave_a /= imu_detect_buffer_.size();
  for (const auto& p : imu_detect_buffer_) {
    var_a += (p.segment<3>(0) - ave_a).squaredNorm();
  }
  var_a /= imu_detect_buffer_.size();
  return std::sqrt(var_a);
}

}  // namespace localization
}  // namespace senseAD
