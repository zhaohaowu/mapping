/******************************************************************************
 * Copyright (C) 2022 HOZON-AUTO Ltd. All rights reserved.
 * Licensed Hozon
 * Author: pw
 ******************************************************************************/
#include "modules/dr/include/odometry_base.h"

namespace hozon {
namespace mp {
namespace dr {

void OdometryBase::add_imu_data(const ImuDataHozon& imu_data) {
  DRWriteLockGuard lock(imu_rw_lock_);
  if (imu_datas_.empty()) {
    imu_datas_.push_back(imu_data);
  } else {
    if (imu_data.timestamp <= imu_datas_.back().timestamp) {
      // HLOG_WARN << "error imu timestamp";
    } else {
      imu_datas_.push_back(imu_data);
      if (imu_datas_.size() > 600) {
        imu_datas_.pop_front();
      }
    }
  }
  // HLOG_DEBUG << "imu data size:" << imu_datas_.size();
}

double OdometryBase::filter_cnt(double cnt) {
  static double Qk = 0.01;  // 预测得误差
  static double Rk = 0.25;  // 观测得误差

  static double xk = 0;  // 第一次预测值
  static double pk = 1;  // 预测得方差

  double xk_pre = xk;       // 假设匀速运动
  double pk_pre = pk + Qk;  //

  double zk = cnt;
  double K = pk_pre * (pk_pre + Rk);
  xk = xk_pre + K * (zk - xk_pre);
  pk = (1 - K) * pk_pre;

  return xk;
}

Eigen::Vector3d OdometryBase::filter_vel(const Eigen::Vector3d& cur_vel) {
  static Eigen::Matrix3d Qk =
      Eigen::Matrix3d::Identity() * 0.001;  // 0.01,0.001,0.01,0.001
  static Eigen::Matrix3d Rk =
      Eigen::Matrix3d::Identity() * 0.1;  // 0.1,0.05,0.5,0.1

  static Eigen::Vector3d xk = {0, 0, 0};
  static Eigen::Matrix3d pk = Eigen::Matrix3d::Identity();

  Eigen::Vector3d xk_pre = xk;
  Eigen::Matrix3d pk_pre = pk + Qk;

  // Eigen::Vector3d zk = cur_vel;
  Eigen::Matrix3d K = pk_pre * (pk_pre + Rk).inverse();
  xk = xk_pre + K * (cur_vel - xk_pre);
  pk = (Eigen::Matrix3d::Identity() - K) * pk_pre;

  return xk;
}

bool OdometryBase::add_wheel_data(const WheelDataHozon& wheel_data) {
  DRWriteLockGuard lock(wheel_rw_lock_);

  HLOG_DEBUG << "wsj_rolling_conter_start" << wheel_data.rolling_counter
             << "wsj_rolling_conter_end";

  HLOG_DEBUG << "wheel_data.timestamp:" << wheel_data.timestamp;

  HLOG_DEBUG << "wheel_data.rear_left_wheel:" << wheel_data.rear_left_wheel;

  if (wheel_datas_.empty()) {
    wheel_datas_.push_back(wheel_data);
  } else {
    if (wheel_data.timestamp > wheel_datas_.back().timestamp) {
      WheelDataHozon last_data = wheel_datas_.back();
      if (fabs(wheel_data.rear_left_wheel - last_data.rear_left_wheel) < 1e-2 &&
          fabs(wheel_data.rear_right_wheel - last_data.rear_right_wheel) <
              1e-2 &&
          fabs(wheel_data.timestamp - last_data.timestamp) < 1 * 1e-3) {
        return false;
      }
      // TODO(zxl) 增加滤波器
      // auto filter_wheel = filter_wheel_data(wheel_data);
      // wheel_datas_.push_back(filter_wheel);
      wheel_datas_.push_back(wheel_data);
      if (wheel_datas_.size() > 600) {
        wheel_datas_.pop_front();
      }
    } else {
      return false;
    }
  }
  // HLOG_DEBUG << "==== init ==== wheel data size:" << wheel_datas_.size();
  return true;
}

std::vector<WheelDataHozon> OdometryBase::get_oldest_two_wheel() {
  DRReadLockGuard lock(wheel_rw_lock_);
  std::vector<WheelDataHozon> oldest_wheel_datas;
  if (wheel_datas_.size() < 2) {
    // HLOG_ERROR << "==== init ==== wheel data size:" << wheel_datas_.size();
    return oldest_wheel_datas;
  }
  oldest_wheel_datas.emplace_back(*(wheel_datas_.begin()));
  oldest_wheel_datas.emplace_back(*(wheel_datas_.begin() + 1));
  return oldest_wheel_datas;
}

bool OdometryBase::get_imu_before_and_pop(
    double time, std::vector<ImuDataHozon>& imu_datas) {
  {
    DRReadLockGuard lock(imu_rw_lock_);

    if (imu_datas_.empty()) {
      return false;
    }

    for (auto itr = imu_datas_.begin(); itr != imu_datas_.end(); itr++) {
      if (itr->timestamp < time) {
        imu_datas.emplace_back(*itr);
      } else {
        // HLOG_DEBUG << "==== init ====" << time << "   error time ";
        break;
      }
    }
  }
  if (imu_datas.size() > 1) {
    DRWriteLockGuard lock(imu_rw_lock_);
    imu_datas_.erase(imu_datas_.begin(), imu_datas_.begin() + imu_datas.size() -
                                             static_cast<size_t>(1));
  }
  return true;
}

void OdometryBase::pop_front_wheel() {
  DRWriteLockGuard lock(wheel_rw_lock_);
  wheel_datas_.pop_front();
}

WheelDataHozon OdometryBase::get_front_wheel() {
  DRReadLockGuard lock(wheel_rw_lock_);
  return wheel_datas_.front();
}

int OdometryBase::get_imu_data_size() {
  DRReadLockGuard lock(imu_rw_lock_);
  return static_cast<int>(imu_datas_.size());
}

int OdometryBase::get_wheel_data_size() {
  DRReadLockGuard lock(wheel_rw_lock_);
  return static_cast<int>(wheel_datas_.size());
}

void OdometryBase::clear_imu_wheel_datas() {
  {
    DRWriteLockGuard lock(wheel_rw_lock_);
    while (wheel_datas_.size() > 1) {
      wheel_datas_.pop_front();
    }
  }
  double wheel_time = wheel_datas_.front().timestamp;
  {
    DRWriteLockGuard lock(imu_rw_lock_);
    while (!imu_datas_.empty() &&
           imu_datas_.front().timestamp < wheel_time - 5.0e-3) {
      imu_datas_.pop_front();
    }
  }
}

OdometryData OdometryBase::get_latest_odom_data() {
  DRReadLockGuard lock(odom_rw_lock_);
  if (odom_datas_.empty()) {
    OdometryData initial_data;
    initial_data.timestamp = -1.0;
    initial_data.odometry = WheelOdometry(0, 0, 0, 1, 0, 0, 0);
    return initial_data;
  }
  return odom_datas_.back();
}

OdometryData OdometryBase::get_st_odom_data() {
  DRReadLockGuard lock(odom_rw_lock_);
  if (odom_datas_.empty()) {
    OdometryData initial_data;
    initial_data.timestamp = -1.0;
    return initial_data;
  }
  return odom_datas_.back();
}

bool OdometryBase::GetOdomByTimestamp(double time, WheelOdometry& odometry) {
  DRReadLockGuard lock(odom_rw_lock_);
  if (!Is_Initialized()) {
    return false;
  }
  if (odom_datas_.empty()) {
    // HLOG_ERROR << "DR odom data is empty";
    return false;
  }

  if (time > odom_datas_.back().timestamp) {
    // HLOG_ERROR << "image time is forward than dr time:"
    //  << time - odom_datas_.back().timestamp;
    return false;
  }
  if (time < odom_datas_.front().timestamp) {
    // HLOG_ERROR << "image time is delay than dr time:"
    //  << odom_datas_.front().timestamp - time;
    return false;
  }

  auto cmp = [](const OdometryData& x, const OdometryData& y) -> bool {
    return x.timestamp < y.timestamp;
  };

  auto iter_next =
      std::lower_bound(odom_datas_.begin(), odom_datas_.end(),
                       OdometryData({0, 0, time, 0, 0, WheelOdometry()}), cmp);

  if (iter_next == odom_datas_.end() || iter_next == odom_datas_.begin()) {
    return false;
  }
  auto iter_pre = std::prev(iter_next);
  auto ratio = (time - iter_pre->timestamp) /
               (iter_next->timestamp - iter_pre->timestamp);
  assert(ratio >= 0.0 && ratio <= 1.0);
  if (ratio < 1e-3) {
    odometry = iter_pre->odometry;
    return true;
  }

  if ((1 - ratio) < 1e-3) {
    odometry = iter_next->odometry;
    return true;
  }
  // 线性插值
  odometry.x =
      (1 - ratio) * iter_pre->odometry.x + ratio * iter_next->odometry.x;
  odometry.y =
      (1 - ratio) * iter_pre->odometry.y + ratio * iter_next->odometry.y;
  odometry.z =
      (1 - ratio) * iter_pre->odometry.z + ratio * iter_next->odometry.z;
  // 球面线性插值
  Eigen::Quaterniond pre_qat(iter_pre->odometry.qw, iter_pre->odometry.qx,
                             iter_pre->odometry.qy, iter_pre->odometry.qz);
  Eigen::Quaterniond next_qat(iter_next->odometry.qw, iter_next->odometry.qx,
                              iter_next->odometry.qy, iter_next->odometry.qz);
  Eigen::Quaterniond inter_qat = pre_qat.slerp(ratio, next_qat);

  odometry.qw = inter_qat.w();
  odometry.qx = inter_qat.x();
  odometry.qy = inter_qat.y();
  odometry.qz = inter_qat.z();
  return true;
}

// bool GetImuByTimestamp(double time, ImuDataHozon& imu_data) { return true; }

void OdometryBase::AddOdomData(
    const OdometryData& new_odom /*, double delta_dis*/) {
  {
    DRWriteLockGuard lock(odom_rw_lock_);
    odom_datas_.push_back(new_odom);
    while (odom_datas_.size() > 600) {
      odom_datas_.pop_front();
    }
  }
}

bool OdometryBase::InterpolatePose(double time, OdometryData& odom_data) {
  DRReadLockGuard lock(odom_rw_lock_);
  bool result = true;
  if (odom_datas_.empty()) {
    return false;
  }
  if (time < odom_datas_.front().timestamp) {
    return false;
  }
  if (time - odom_datas_.back().timestamp > 0.2) {
    return false;
  }

  if (time > odom_datas_.back().timestamp) {
    // predict
    OdometryData ref_data = odom_datas_.back();
    Eigen::Vector3d ref_pose = {ref_data.odometry.x, ref_data.odometry.y,
                                ref_data.odometry.z};
    Eigen::Quaterniond ref_qat =
        Eigen::Quaterniond(ref_data.odometry.qw, ref_data.odometry.qx,
                           ref_data.odometry.qy, ref_data.odometry.qz);

    Eigen::Vector3d ref_vel = ref_data.loc_vel;
    Eigen::Vector3d ref_omg = ref_data.loc_omg;
    Eigen::Vector3d ref_acc = ref_data.loc_acc;

    double delta_t = time - ref_data.timestamp;
    Eigen::Vector3d pose = ref_pose + ref_qat * (ref_vel * delta_t);
    Eigen::Quaterniond qat = ref_qat;
    Eigen::Vector3d det_ang = ref_omg * delta_t;
    if (det_ang.norm() > 1e-7) {
      qat = qat * Eigen::Quaterniond(
                      Eigen::AngleAxisd(det_ang.norm(), det_ang.normalized()));
      qat = qat.normalized();
    }
    odom_data.odometry.x = pose[0];
    odom_data.odometry.y = pose[1];
    odom_data.odometry.z = pose[2];
    odom_data.odometry.qx = qat.x();
    odom_data.odometry.qy = qat.y();
    odom_data.odometry.qz = qat.z();
    odom_data.odometry.qw = qat.w();
    odom_data.loc_vel = ref_vel;
    odom_data.loc_acc = ref_acc;
    odom_data.loc_omg = ref_omg;
    return true;
  }

  // interpolate
  auto cmp = [](const OdometryData& x, const OdometryData& y) -> bool {
    return x.timestamp < y.timestamp;
  };
  auto iter_next =
      std::lower_bound(odom_datas_.begin(), odom_datas_.end(),
                       OdometryData({0, 0, time, 0, 0, WheelOdometry()}), cmp);

  if (iter_next == odom_datas_.end() || iter_next == odom_datas_.begin()) {
    // std::cout << "maybe odom delay,not find" << std::endl;
    return false;
  }
  auto iter_pre = std::prev(iter_next);
  auto ratio = (time - iter_pre->timestamp) /
               (iter_next->timestamp - iter_pre->timestamp);

  // std::cout << "ratio:" << ratio << std::endl;
  assert(ratio >= 0.0 && ratio <= 1.0);
  if (ratio < 1e-2) {
    Eigen::Vector3d pose = {iter_pre->odometry.x, iter_pre->odometry.y,
                            iter_pre->odometry.z};
    Eigen::Quaterniond qat =
        Eigen::Quaterniond(iter_pre->odometry.qw, iter_pre->odometry.qx,
                           iter_pre->odometry.qy, iter_pre->odometry.qz);
    odom_data.odometry.x = pose[0];
    odom_data.odometry.y = pose[1];
    odom_data.odometry.z = pose[2];
    odom_data.odometry.qx = qat.x();
    odom_data.odometry.qy = qat.y();
    odom_data.odometry.qz = qat.z();
    odom_data.odometry.qw = qat.w();
    odom_data.loc_vel = iter_pre->loc_vel;
    odom_data.loc_acc = iter_pre->loc_acc;
    odom_data.loc_omg = iter_pre->loc_omg;
    return true;
  }

  if ((1 - ratio) < 1e-2) {
    Eigen::Vector3d pose = {iter_next->odometry.x, iter_next->odometry.y,
                            iter_next->odometry.z};
    Eigen::Quaterniond qat =
        Eigen::Quaterniond(iter_next->odometry.qw, iter_next->odometry.qx,
                           iter_next->odometry.qy, iter_next->odometry.qz);
    odom_data.odometry.x = pose[0];
    odom_data.odometry.y = pose[1];
    odom_data.odometry.z = pose[2];
    odom_data.odometry.qx = qat.x();
    odom_data.odometry.qy = qat.y();
    odom_data.odometry.qz = qat.z();
    odom_data.odometry.qw = qat.w();
    odom_data.loc_vel = iter_next->loc_vel;
    odom_data.loc_acc = iter_next->loc_acc;
    odom_data.loc_omg = iter_next->loc_omg;

    return true;
  }
  // 线性插值
  odom_data.odometry.x =
      (1 - ratio) * iter_pre->odometry.x + ratio * iter_next->odometry.x;
  odom_data.odometry.y =
      (1 - ratio) * iter_pre->odometry.y + ratio * iter_next->odometry.y;
  odom_data.odometry.z =
      (1 - ratio) * iter_pre->odometry.z + ratio * iter_next->odometry.z;

  odom_data.loc_vel[0] =
      (1 - ratio) * iter_pre->loc_vel[0] + ratio * iter_next->loc_vel[0];
  odom_data.loc_vel[1] =
      (1 - ratio) * iter_pre->loc_vel[1] + ratio * iter_next->loc_vel[1];
  odom_data.loc_vel[2] =
      (1 - ratio) * iter_pre->loc_vel[2] + ratio * iter_next->loc_vel[2];

  odom_data.loc_omg[0] =
      (1 - ratio) * iter_pre->loc_omg[0] + ratio * iter_next->loc_omg[0];
  odom_data.loc_omg[1] =
      (1 - ratio) * iter_pre->loc_omg[1] + ratio * iter_next->loc_omg[1];
  odom_data.loc_omg[2] =
      (1 - ratio) * iter_pre->loc_omg[2] + ratio * iter_next->loc_omg[2];

  odom_data.loc_acc[0] =
      (1 - ratio) * iter_pre->loc_acc[0] + ratio * iter_next->loc_acc[0];
  odom_data.loc_acc[1] =
      (1 - ratio) * iter_pre->loc_acc[1] + ratio * iter_next->loc_acc[1];
  odom_data.loc_acc[2] =
      (1 - ratio) * iter_pre->loc_acc[2] + ratio * iter_next->loc_acc[2];
  // 球面线性插值
  Eigen::Quaterniond pre_qat(iter_pre->odometry.qw, iter_pre->odometry.qx,
                             iter_pre->odometry.qy, iter_pre->odometry.qz);
  Eigen::Quaterniond next_qat(iter_next->odometry.qw, iter_next->odometry.qx,
                              iter_next->odometry.qy, iter_next->odometry.qz);
  Eigen::Quaterniond inter_qat = pre_qat.slerp(ratio, next_qat);
  odom_data.odometry.qx = inter_qat.x();
  odom_data.odometry.qy = inter_qat.y();
  odom_data.odometry.qz = inter_qat.z();
  odom_data.odometry.qw = inter_qat.w();
  return true;
}

void OdometryBase::Qat2EulerAngle(const Eigen::Quaterniond& q,
                                  double& roll,   // NOLINT
                                  double& pitch,  // NOLINT
                                  double& yaw) {  // NOLINT
  // roll (x-axis rotation)
  double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
  double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
  roll = atan2(sinr_cosp, cosr_cosp) * 180 / M_PI;

  // pitch (y-axis rotation)
  double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
  if (fabs(sinp) >= 1) {
    pitch = copysign(M_PI / 2, sinp) * 180 / M_PI;  //
  } else {
    pitch = asin(sinp) * 180 / M_PI;
  }
  // yaw (z-axis rotation)
  double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
  double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  yaw = atan2(siny_cosp, cosy_cosp) * 180 / M_PI;
  if (yaw < 0) {
    yaw += 360.0;
  }
}

}  // namespace dr
}  // namespace mp
}  // namespace hozon
