/******************************************************************************
 * Copyright (C) 2022 HOZON-AUTO Ltd. All rights reserved.
 * Licensed Hozon
 * Author: pengwei
 ******************************************************************************/
#include "modules/dr/include/wheel_odom.h"

namespace hozon {
namespace mp {
namespace dr {

bool WheelOdom::update() {
  initialized_ = true;
  if (wheel_datas_.size() < 2) {
    // std::cout << "too less wheel data" << std::endl;
    return false;
  }
  WheelDataHozon latest_wheel = *(wheel_datas_.begin() + 1);
  WheelDataHozon pre_latest_wheel = *(wheel_datas_.begin());
  process_wheel(pre_latest_wheel, latest_wheel);
  wheel_datas_.pop_front();
  return true;
}

bool WheelOdom::process_wheel(const WheelDataHozon& last,
                              const WheelDataHozon& cur) {
  // std::cout << "process wheel time:" << cur.timestamp << std::endl;
  if (cur.rear_left_dir == 0 || cur.rear_right_dir == 0) {
    //   std::cout << "error wheel direction " << std::endl;
    return false;
  }
  double begin_wl_1 = last.front_left_wheel;
  double begin_wr_1 = last.front_right_wheel;
  double end_wl_1 = last.rear_left_wheel;
  double end_wr_1 = last.rear_right_wheel;

  double begin_wl_2 = cur.front_left_wheel;
  double begin_wr_2 = cur.front_right_wheel;
  double end_wl_2 = cur.rear_left_wheel;
  double end_wr_2 = cur.rear_right_wheel;

  double end_wl_1_tmp = end_wl_1;
  double end_wl_2_tmp = end_wl_2;

  double end_wr_1_tmp = end_wr_1;
  double end_wr_2_tmp = end_wr_2;
  // 前后两次的脉冲差
  double left_diff = end_wl_2_tmp - end_wl_1_tmp;
  if (left_diff < 0) left_diff += MAX_WHEEL_COUNT;
  double right_diff = end_wr_2_tmp - end_wr_1_tmp;
  if (right_diff < 0) right_diff += MAX_WHEEL_COUNT;
  // 左轮运动距离
  double left_dist = left_diff * wheel_param_.kl_;
  if (cur.rear_left_dir == 2) {
    left_dist *= -1.0;
    // std::cout << "left back direction" << std::endl;
  }
  double right_dist = right_diff * wheel_param_.kr_;
  if (cur.rear_right_dir == 2) {
    right_dist *= -1.0;
    //  std::cout << "right back direction" << std::endl;
  }
  double delta_yaw = (right_dist - left_dist) / wheel_param_.b_;
  double delta_dist = (right_dist + left_dist) * 0.5;
  double delta_t = (cur.timestamp - last.timestamp);
  v_by_wheel_ = delta_dist / delta_t;
  w_by_wheel_ = delta_yaw / delta_t;

  Eigen::Quaterniond delta_qat;
  Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()));
  Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()));
  Eigen::AngleAxisd yawAngle(
      Eigen::AngleAxisd(delta_yaw, Eigen::Vector3d::UnitZ()));

  delta_qat = yawAngle * pitchAngle * rollAngle;
  const Eigen::Vector3d delta_p = Eigen::Vector3d(delta_dist, 0., 0.);
  Eigen::Matrix3d rotation = qat_.toRotationMatrix();

  pos_ = pos_ + rotation * delta_p;
  qat_ = qat_ * delta_qat;
  qat_ = qat_.normalized();

  OdometryData cur_odom_data;
  cur_odom_data.timestamp = cur.timestamp;
  // cur_odom_data.odometry.position << pos_[0], pos_[1], pos_[2];

  // cur_odom_data.odometry.quaternion.w = qat_.w();
  // cur_odom_data.odometry.quaternion.x = qat_.x();
  // cur_odom_data.odometry.quaternion.y = qat_.y();
  // cur_odom_data.odometry.quaternion.z = qat_.z();

  cur_odom_data.odometry  = WheelOdometry(pos_[0], pos_[1], pos_[2], qat_.w(),
   qat_.x(), qat_.y(), qat_.z());
  cur_odom_data.loc_vel = {v_by_wheel_, 0, 0};
  cur_odom_data.loc_omg = {0, 0, w_by_wheel_};
  cur_odom_data.loc_acc = {0, 0, 0};

  AddOdomData(cur_odom_data, delta_dist);

  double yaw = 2 * atan2(qat_.z(), qat_.w());
  if (yaw > M_PI) yaw -= 2 * M_PI;
  if (yaw < -M_PI) yaw += 2 * M_PI;

  v_by_wheel_ = delta_dist / delta_t;
  w_by_wheel_ = delta_yaw / delta_t;

  /*  std::cout << "delta dis:" << delta_dist << ",delta angle:" << delta_yaw
             << std::endl;
   std::cout << "x,y,yaw:" << pos_[0] << "," << pos_[1] << "," << yaw
             << std::endl; */

  return true;
}

}  // namespace dr
}  // namespace mp
}  // namespace hozon
