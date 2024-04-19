/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */

#include "system/localization_state_monitor.hpp"
#include <pthread.h>

#include <chrono>
#include <limits>

#include "common/msf_common.hpp"
#include "localization/common/log.hpp"
#include "localization_manager_impl.hpp"
#include "system/localization_dead_reckoning.hpp"
#include "system/localization_frontend.hpp"

namespace senseAD {
namespace localization {

LocalizationStateMonitor::LocalizationStateMonitor(
    LocalizationManagerImpl* loc_manager)
    : loc_manager_(loc_manager) {}

LocalizationStateMonitor::~LocalizationStateMonitor() {
  if (thread_ && thread_->joinable()) thread_->join();
}

adLocStatus_t LocalizationStateMonitor::Init(const LocalizationParam& param) {
  param_ = param;
  auto& cp = param_.common_param;
  enable_dr_restart_ = cp.relative_localization && cp.enable_dr_restart;
  enable_loc_restart_ = cp.absolute_localization && cp.enable_loc_restart;
  enable_switch_origin_ = cp.absolute_localization && cp.enable_switch_origin;
  if (enable_dr_restart_ || enable_loc_restart_ || enable_switch_origin_) {
    // create monitor thread
    thread_.reset(
        new std::thread(&LocalizationStateMonitor::RunMonitorProc, this));
    if (nullptr == thread_) {
      LC_LERROR(MONITOR) << "failed to create monitor thread";
      return LOC_LOCALIZATION_ERROR;
    }
    SetThreadName();
  }
  // define sensor names
  std::vector<std::string> sensor_array{"can",  "imu", "ins",
                                        "gnss", "nav", "dualant"};
  for (const auto& camera_name : param_.smm_param.enable_camera_names) {
    sensor_array.emplace_back(camera_name);
  }
  {
    std::lock_guard<std::mutex> lock(st_mutex_);
    for (const auto& sensor : sensor_array) {
      sensor_timestamps_.insert({sensor, std::deque<uint64_t>()});
    }
  }
  return LOC_SUCCESS;
}

adLocStatus_t LocalizationStateMonitor::CheckCanData(
    uint64_t timestamp, const VehicleInfo& vehinfo_data) {
  // check data valid
  double rear_speed =
      0.5 * (vehinfo_data.wheel_speed_rl + vehinfo_data.wheel_speed_rr);
  if (std::fabs(rear_speed - vehinfo_data.vehicle_speed) > 2.0) {
    LC_LWARN_EVERY(MONITOR, 20)
        << "check can data failed, speed: " << vehinfo_data.vehicle_speed
        << ", " << rear_speed;
  }
  // check timestamp valid
  if (timestamp == 0 || timestamp == std::numeric_limits<uint64_t>::max()) {
    LC_LERROR_EVERY(MONITOR, 20)
        << "check can timestamp failed, timestamp: " << timestamp;
    return LOC_INVALID;
  }
  // check data consistency
  std::unique_lock<std::mutex> lock(imu_mutex_);
  if (latest_imu_data_.first != 0) {
    if ((latest_imu_data_.first * 1e-9 - timestamp * 1e-9) > 0.1) {
      LC_LWARN_EVERY(MONITOR, 20) << "can has large gap with imu.";
    }
  }
  lock.unlock();
  // check timestamp consistency
  auto status = CheckTimestampValid("can", timestamp);
  if (status != LOC_SUCCESS) return status;
  // check whether frame rate meets the minimum requirements
  if (is_can_full_) {
    std::lock_guard<std::mutex> lock(st_mutex_);
    double frame_rate = sensor_timestamps_["can"].size() / window_secs_;
    if (frame_rate < 10.0) {
      LC_LWARN_EVERY(MONITOR, 20) << "check can rate(HZ): " << frame_rate;
    }
  }
  {
    std::lock_guard<std::mutex> lock(can_mutex_);
    latest_can_data_ = std::make_pair(timestamp, vehinfo_data);
  }

  return LOC_SUCCESS;
}

adLocStatus_t LocalizationStateMonitor::CheckImuData(uint64_t timestamp,
                                                     const Imu& imu_data) {
  // TODO(zm): as ci-simulation set imu data abnormally
  // check data valid
  // if (imu_data.status != ImuStatus::GOOD) {
  //     LC_LERROR_EVERY(MONITOR, 20) << "check imu status is Invalid";
  //     return LOC_INVALID;
  // }
  // Eigen::Vector3d acc;
  // acc << imu_data.linear_acceleration.x, imu_data.linear_acceleration.y,
  //     imu_data.linear_acceleration.z;
  // if (acc.norm() < g_norm - 5.0) {  // consider has accelerate in z-direct
  //     LC_LERROR_EVERY(MONITOR, 20)
  //         << "check imu data failed, acc norm: " << acc.norm();
  //     return LOC_INVALID;
  // }
  // check timestamp valid
  if (timestamp == 0 || timestamp == std::numeric_limits<uint64_t>::max()) {
    LC_LERROR_EVERY(MONITOR, 20)
        << "check imu timestamp failed, timestamp: " << timestamp;
    return LOC_INVALID;
  }
  // check timestamp consistency
  auto status = CheckTimestampValid("imu", timestamp);
  if (status != LOC_SUCCESS) return status;
  // check whether frame rate meets the minimum requirements
  if (is_imu_full_) {
    std::lock_guard<std::mutex> lock(st_mutex_);
    double frame_rate = sensor_timestamps_["imu"].size() / window_secs_;
    if (frame_rate < 70.0) {
      LC_LWARN_EVERY(MONITOR, 20) << "check imu rate(HZ): " << frame_rate;
    }
  }
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    latest_imu_data_ = std::make_pair(timestamp, imu_data);
  }

  return LOC_SUCCESS;
}

adLocStatus_t LocalizationStateMonitor::CheckInsData(uint64_t timestamp,
                                                     const Ins& ins_data) {
  if (!param_.common_param.sub_ins_data) return LOC_SUCCESS;
  // check data valid
  if (ins_data.status != InsStatus::GOOD) {
    LC_LERROR_EVERY(MONITOR, 20) << "check ins status is Invalid.";
    return LOC_INVALID;
  }
  Eigen::Vector3d position;
  position << ins_data.position.lon, ins_data.position.lat,
      ins_data.position.height;
  if (position.norm() < 1e-10 || std::fabs(position.x()) > 180.0 ||
      std::fabs(position.y()) > 90.0) {
    LC_LERROR_EVERY(MONITOR, 20)
        << "check ins data failed, position: " << position.transpose();
    return LOC_INVALID;
  }
  Eigen::Vector3d velocity;
  velocity << ins_data.linear_velocity.x, ins_data.linear_velocity.y,
      ins_data.linear_velocity.z;
  if (velocity.norm() > 60.0) {
    LC_LERROR_EVERY(MONITOR, 1)
        << "check ins data failed, velocity: " << velocity.transpose();
    return LOC_INVALID;
  }
  // check timestamp valid
  if (timestamp == 0 || timestamp == std::numeric_limits<uint64_t>::max()) {
    LC_LERROR_EVERY(MONITOR, 20)
        << "check ins timestamp failed, timestamp: " << timestamp;
    return LOC_INVALID;
  }
  // check timestamp consistency
  auto status = CheckTimestampValid("ins", timestamp);
  if (status != LOC_SUCCESS) return status;
  {
    std::lock_guard<std::mutex> lock(ins_mutex_);
    latest_ins_data_ = std::make_pair(timestamp, ins_data);
  }

  return LOC_SUCCESS;
}

adLocStatus_t LocalizationStateMonitor::CheckGnssData(uint64_t timestamp,
                                                      const Gnss& gnss_data) {
  // check data valid
  // TODO(zm): affect the cost time of localization initialization
  // if (gnss_data.status == GnssStatus::INVALID) {
  //     LC_LERROR_EVERY(MONITOR, 1) << "check gnss status is Invalid.";
  //     return LOC_INVALID;
  // }
  Eigen::Vector3d position;
  position << gnss_data.position.lon, gnss_data.position.lat,
      gnss_data.position.height;
  if (position.cwiseAbs().norm() < 1e-10 || std::fabs(position.x()) > 180.0 ||
      std::fabs(position.y()) > 90.0) {
    LC_LERROR_EVERY(MONITOR, 1)
        << "check gnss data failed, position: " << position.transpose();
    return LOC_INVALID;
  }
  Eigen::Vector3d velocity;
  velocity << gnss_data.linear_velocity.x, gnss_data.linear_velocity.y,
      gnss_data.linear_velocity.z;
  if (velocity.norm() > 60.0) {
    LC_LERROR_EVERY(MONITOR, 1)
        << "check gnss data failed, velocity: " << velocity.transpose();
    return LOC_INVALID;
  }
  Eigen::Vector3d pos_sd, vel_sd;
  pos_sd << gnss_data.position_std_dev.x, gnss_data.position_std_dev.y,
      gnss_data.position_std_dev.z;
  vel_sd << gnss_data.linear_velocity_std_dev.x,
      gnss_data.linear_velocity_std_dev.y, gnss_data.linear_velocity_std_dev.z;
  if (pos_sd.x() <= 0 || pos_sd.y() <= 0 || pos_sd.z() <= 0 ||
      vel_sd.x() <= 0 || vel_sd.y() <= 0 || vel_sd.z() <= 0) {
    LC_LERROR_EVERY(MONITOR, 1)
        << "check gnss data failed, pos stddev: " << pos_sd.transpose()
        << ", and vel stddev: " << vel_sd.transpose();
    return LOC_INVALID;
  }
  // check timestamp valid
  if (timestamp == 0 || timestamp == std::numeric_limits<uint64_t>::max()) {
    LC_LERROR_EVERY(MONITOR, 1)
        << "check gnss timestamp failed, timestamp: " << timestamp;
    return LOC_INVALID;
  }
  // check data consistency
  std::unique_lock<std::mutex> lock(can_mutex_);
  if (latest_can_data_.first != 0) {
    if ((latest_can_data_.first * 1e-9 - timestamp * 1e-9) > 0.2) {
      LC_LWARN_EVERY(MONITOR, 1) << "gnss has large gap with can.";
    } else {
      double can_vel = latest_can_data_.second.vehicle_speed;
      double gnss_vel = velocity.head(2).norm();
      if (std::fabs(can_vel - gnss_vel) > 2.0) {
        LC_LWARN_EVERY(MONITOR, 1)
            << "gnss velocity has inconsistency with can, gnss_vel: "
            << gnss_vel << ", can_vel: " << can_vel;
      }
    }
  }
  lock.unlock();

  // check consistency with ins
  if (param_.common_param.sub_ins_data) {
    std::lock_guard<std::mutex> lock2(ins_mutex_);
    if (latest_ins_data_.first != 0) {
      if ((latest_ins_data_.first * 1e-9 - timestamp * 1e-9) > 0.1) {
        LC_LWARN_EVERY(MONITOR, 300) << "gnss has large gap with ins.";
      } else {
        auto& ins_pos = latest_ins_data_.second.position;
        if (std::fabs(ins_pos.lon - position.x()) > 1e-4 ||
            std::fabs(ins_pos.lat - position.y()) > 1e-4) {
          LC_LWARN_EVERY(MONITOR, 1)
              << "gnss position has inconsistency with ins, "
                 "gnss_pos: "
              << position.x() << "-" << position.y()
              << ", ins_pos: " << ins_pos.lon << "-" << ins_pos.lat;
        }
        Eigen::Vector3d ins_vel;
        ins_vel << latest_ins_data_.second.linear_velocity.x,
            latest_ins_data_.second.linear_velocity.y,
            latest_ins_data_.second.linear_velocity.z;
        if ((ins_vel - velocity).norm() > 2.0) {
          LC_LWARN_EVERY(MONITOR, 1)
              << "gnss velocity has inconsistency with ins, "
                 "gnss_vel: "
              << velocity.transpose() << ", ins_vel: " << ins_vel.transpose();
        }
      }
    }
  }

  // check timestamp consistency
  auto status = CheckTimestampValid("gnss", timestamp);
  if (status != LOC_SUCCESS) return status;
  {
    std::lock_guard<std::mutex> lock(gnss_mutex_);
    latest_gnss_data_ = std::make_pair(timestamp, gnss_data);
  }

  return LOC_SUCCESS;
}

adLocStatus_t LocalizationStateMonitor::CheckPerceptData(
    uint64_t timestamp, const std::shared_ptr<PerceptData>& percept_data) {
  // TODO(xx): check data valid
  // check timestamp valid
  if (timestamp == 0 || timestamp == std::numeric_limits<uint64_t>::max()) {
    LC_LERROR_EVERY(MONITOR, 1)
        << "check percept timestamp failed, timestamp: " << timestamp;
    return LOC_INVALID;
  }
  // check timestamp consistency
  auto status = CheckTimestampValid(percept_data->camera_name, timestamp);
  if (status != LOC_SUCCESS) return status;

  return LOC_SUCCESS;
}

adLocStatus_t LocalizationStateMonitor::CheckLocalMapData(
    uint64_t timestamp, const std::shared_ptr<RoadStructure>& local_map_data) {
  // TODO(xx): don't check, as localmap timestamp is 0 ?
  return LOC_SUCCESS;
}

adLocStatus_t LocalizationStateMonitor::CheckNavData(
    uint64_t timestamp, const NavState& nav_state) {
  // TODO(xx): check data valid
  // check timestamp valid
  if (timestamp == 0 || timestamp == std::numeric_limits<uint64_t>::max()) {
    LC_LERROR_EVERY(MONITOR, 20)
        << "check nav timestamp failed, timestamp: " << timestamp;
    return LOC_INVALID;
  }
  // check timestamp consistency
  auto status = CheckTimestampValid("nav", timestamp);
  if (status != LOC_SUCCESS) return status;

  return LOC_SUCCESS;
}

adLocStatus_t LocalizationStateMonitor::CheckDualAntHeadingData(
    uint64_t timestamp, const DualAntennaHeading& dual_ant_heading) {
  // check data valid
  // TODO(xx): status has always been invalid, not get in sensor ?
  if (dual_ant_heading.status != GnssStatus::RTK_INTEGER) {
    LC_LERROR_EVERY(MONITOR, 10) << "check dualant status is not RTK.";
    return LOC_INVALID;
  }
  Eigen::Vector2d dualant_data;
  dualant_data << dual_ant_heading.heading, dual_ant_heading.heading_std;
  if (std::fabs(dualant_data.x()) > M_PI || dualant_data.y() > 5.0 * d2r ||
      dualant_data.y() < 0.01 * d2r) {
    LC_LERROR_EVERY(MONITOR, 10)
        << "check dualant data failed, heading: " << dualant_data.x()
        << ", heading stddev: " << dualant_data.y();
    return LOC_INVALID;
  }
  // check timestamp valid
  if (timestamp == 0 || timestamp == std::numeric_limits<uint64_t>::max()) {
    LC_LERROR_EVERY(MONITOR, 10)
        << "check dualant timestamp failed, timestamp: " << timestamp;
    return LOC_INVALID;
  }
  // check timestamp consistency
  auto status = CheckTimestampValid("dualant", timestamp);
  if (status != LOC_SUCCESS) return status;

  return LOC_SUCCESS;
}

adLocStatus_t LocalizationStateMonitor::CheckNavStateInfoData(
    const NavStateInfo& info) {
  // check delay of localization processing
  uint64_t timestamp = info.measurement_time_ns;
  {
    std::lock_guard<std::mutex> lock(nav_mutex_);
    if (!nav_timestamps_.empty() && timestamp <= nav_timestamps_.back())
      return LOC_SUCCESS;
  }
  uint64_t cur_time = 0;
  {
    std::lock_guard<std::mutex> lock(st_mutex_);
    for (const auto& item : sensor_timestamps_) {
      const auto& dq = item.second;
      if (!dq.empty() && dq.back() > cur_time) cur_time = dq.back();
    }
  }
  double time_gap = timestamp * 1e-9 - cur_time * 1e-9;
  if (std::fabs(time_gap) > 0.2) {
    LC_LWARN_EVERY(MONITOR, 20)
        << "check output nav state process delay, time gap: " << time_gap
        << ", nav timestamp: " << timestamp << ", sys timestamp: " << cur_time;
  }

  // check whether frame rate meets the minimum requirements
  static bool is_nav_full = false;
  std::lock_guard<std::mutex> lock(nav_mutex_);
  nav_timestamps_.emplace_back(timestamp);
  if (timestamp - nav_timestamps_.front() > window_secs_ * 1e9) {
    nav_timestamps_.pop_front();
    if (!is_nav_full) is_nav_full = true;
  }
  if (is_nav_full) {
    double frame_rate = nav_timestamps_.size() / window_secs_;
    if (frame_rate < 90.0) {
      LC_LWARN_EVERY(MONITOR, 20)
          << "check output nav state rate(HZ): " << frame_rate;
    }
  }

  return LOC_SUCCESS;
}

adLocStatus_t LocalizationStateMonitor::CheckOdomStateInfoData(
    const OdomStateInfo& info) {
  // check delay of localization processing
  uint64_t timestamp = info.measurement_time_ns;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (!odom_timestamps_.empty() && timestamp <= odom_timestamps_.back())
      return LOC_SUCCESS;
  }
  uint64_t cur_time = 0;
  {
    std::lock_guard<std::mutex> lock(st_mutex_);
    for (const auto& item : sensor_timestamps_) {
      const auto& dq = item.second;
      if (!dq.empty() && dq.back() > cur_time) cur_time = dq.back();
    }
  }
  double time_gap = cur_time * 1e-9 - timestamp * 1e-9;
  if (std::fabs(time_gap) > 0.2) {
    LC_LWARN_EVERY(MONITOR, 20)
        << "check output odom state process delay, time gap: " << time_gap
        << ", nav timestamp: " << timestamp << ", sys timestamp: " << cur_time;
  }

  // check whether frame rate meets the minimum requirements
  static bool is_odom_full = false;
  std::lock_guard<std::mutex> lock(odom_mutex_);
  odom_timestamps_.emplace_back(timestamp);
  if (timestamp - odom_timestamps_.front() > window_secs_ * 1e9) {
    odom_timestamps_.pop_front();
    if (!is_odom_full) is_odom_full = true;
  }
  if (is_odom_full) {
    double frame_rate = odom_timestamps_.size() / window_secs_;
    if (frame_rate < 90.0) {
      LC_LWARN_EVERY(MONITOR, 20)
          << "check output odom state rate(HZ): " << frame_rate;
    }
  }

  return LOC_SUCCESS;
}

void LocalizationStateMonitor::RunMonitorProc() {
  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // check finish request
    if (CheckFinishRequest()) break;

    // check dr need restart
    if (enable_dr_restart_) {
      if (loc_manager_->loc_dead_reckoning_ &&
          loc_manager_->loc_dead_reckoning_->CheckNeedRestart()) {
        loc_manager_->DRRestartProc();
      }
    }

    // check localization need restart
    if (enable_loc_restart_) {
      if (loc_manager_->loc_frontend_ &&
          loc_manager_->loc_frontend_->CheckNeedRestart()) {
        loc_manager_->LocRestartProc();
      }
    }
    // check localization need switch origin
    if (enable_switch_origin_) {
      if (loc_manager_->loc_frontend_ &&
          loc_manager_->loc_frontend_->CheckNeedSwitchOrigin()) {
        loc_manager_->LocSwitchOriginProc();
      }
    }
  }
  SetFinish();
}

adLocStatus_t LocalizationStateMonitor::CheckTimestampValid(
    const std::string& data_tag, uint64_t timestamp) {
  std::lock_guard<std::mutex> lock(st_mutex_);
  // check self
  if (sensor_timestamps_[data_tag].empty()) {
    sensor_timestamps_[data_tag].emplace_back(timestamp);
    return LOC_SUCCESS;
  }
  double time_gap =
      timestamp * 1e-9 - sensor_timestamps_[data_tag].back() * 1e-9;
  if (timestamp <= sensor_timestamps_[data_tag].back() || time_gap > 1e6) {
    LC_LERROR_EVERY(MONITOR, 20)
        << "self check timestamp valid failed, source: " << data_tag
        << ", disorder timegap: " << time_gap;
    return LOC_INVALID;
  }
  sensor_timestamps_[data_tag].emplace_back(timestamp);
  if (timestamp - sensor_timestamps_[data_tag].front() > window_secs_ * 1e9) {
    sensor_timestamps_[data_tag].pop_front();
    if (!is_can_full_ && data_tag == "can") is_can_full_ = true;
    if (!is_imu_full_ && data_tag == "imu") is_imu_full_ = true;
  }
  // cross check other sensor
  for (const auto& item : sensor_timestamps_) {
    const auto& c_tag = item.first;
    if (c_tag == data_tag) continue;
    if (item.second.empty()) {
      // TODO(zm): as ci-simulation don't set can data
      // if (c_tag == "can" || c_tag == "imu") {
      //     LC_LERROR_EVERY(MONITOR, 20)
      //         << "cross check timestamp valid failed, wait can imu
      //         data.";
      //     return LOC_INVALID;
      // }
      continue;
    }
    double time_gap = timestamp * 1e-9 - item.second.back() * 1e-9;
    if (time_gap < -1.0) {
      LC_LERROR_EVERY_SEC(MONITOR, 1)
          << "cross check timestamp valid failed, source: " << data_tag
          << ", target: " << c_tag << ", timegap: " << time_gap;
      return LOC_INVALID;
    }
    if (time_gap > 5.0) {
      LC_LERROR_EVERY_SEC(MONITOR, 1)
          << "long missing data: " << c_tag << ", timegap: " << time_gap;
    }
  }
  return LOC_SUCCESS;
}

void LocalizationStateMonitor::SetThreadName() {
  pthread_setname_np(thread_->native_handle(), "LC_monitor");
}

}  // namespace localization
}  // namespace senseAD
