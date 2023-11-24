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
  // 静止状态不更新
  if (cur.rear_left_dir == 2 || cur.rear_right_dir == 2) {
      std::cout << "error wheel direction " << std::endl;
    return false;
  }
  // double begin_wl_1 = last.front_left_wheel;
  // double begin_wr_1 = last.front_right_wheel;
  double end_wl_1 = last.rear_left_wheel;
  double end_wr_1 = last.rear_right_wheel;

  // double begin_wl_2 = cur.front_left_wheel;
  // double begin_wr_2 = cur.front_right_wheel;
  double end_wl_2 = cur.rear_left_wheel;
  double end_wr_2 = cur.rear_right_wheel;

  double end_wl_1_tmp = end_wl_1;
  double end_wl_2_tmp = end_wl_2;

  double end_wr_1_tmp = end_wr_1;
  double end_wr_2_tmp = end_wr_2;
  // 前后两次的脉冲差
  double left_diff = end_wl_2_tmp - end_wl_1_tmp;
  if (left_diff < 0) {
    left_diff += MAX_WHEEL_COUNT;
  }
  double right_diff = end_wr_2_tmp - end_wr_1_tmp;
  if (right_diff < 0) {
    right_diff += MAX_WHEEL_COUNT;
  }
  // 左轮运动距离
  double left_dist = left_diff * wheel_param_.kl_;
  if (cur.rear_left_dir == 1) {
    left_dist *= -1.0;
    // std::cout << "left back direction" << std::endl;
  }
  double right_dist = right_diff * wheel_param_.kr_;
  if (cur.rear_right_dir == 1) {
    right_dist *= -1.0;
    //  std::cout << "right back direction" << std::endl;
  }
  left_diff = WheelOdom::filter_cnt(left_diff);
  right_diff = WheelOdom::filter_cnt(right_diff);

  double delta_yaw = (right_diff - left_diff) / wheel_param_.b_;
  double delta_dist = (right_dist + left_dist) * 0.5;
  double delta_t = (cur.timestamp - last.timestamp);
  // std::cout  << "wheel dist: " << delta_dist << " time: " << delta_t
  //           << std::endl; // <<std::setprecision(16)
  v_by_wheel_ = delta_dist / delta_t;
  w_by_wheel_ = delta_yaw / delta_t;

  /////////////////////////////////////
  static std::deque<double> wheel_w = {};
  if (wheel_w.size() > 20) {
    wheel_w.pop_front();
  } else {
    wheel_w.push_back(w_by_wheel_);
  }
  double total_w = 0.0;
  for (auto i : wheel_w) {
    total_w += i;
  }
  double avg_w = 0.0;
  avg_w = total_w / static_cast<double>(wheel_w.size());

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

  cur_odom_data.odometry = WheelOdometry(pos_[0], pos_[1], pos_[2], qat_.w(),
                                         qat_.x(), qat_.y(), qat_.z());

  static std::deque<double> wheel_vel = {};
  if (wheel_vel.size() > 20) {
    wheel_vel.pop_front();
  } else {
    wheel_vel.push_back(v_by_wheel_);
  }
  double total_vel = 0.0;
  for (auto i : wheel_vel) {
    total_vel += i;
  }
  double avg_vel = total_vel / static_cast<double>(wheel_vel.size());

  /////////////////////////////////////
  static std::deque<double> wheel_vel2 = {};
  static double avg_vel2 = 0.0;
  double last_avg_vel = avg_vel2;
  if (wheel_vel2.size() > 20) {
    wheel_vel2.pop_front();
  } else {
    wheel_vel2.push_back(cur.rear_right_speed);
  }
  double total_vel2 = 0.0;
  for (auto i : wheel_vel2) {
    total_vel2 += i;
  }

  avg_vel2 = total_vel2 / static_cast<double>(wheel_vel2.size());

  // Eigen::Vector3d eg_wheel = {v_by_wheel_, 0, 0};
  // cur_odom_data.loc_vel = filter_vel(eg_wheel);

  // Eigen::Vector3d fl_vel = WheelOdom::filter_vel({v_by_wheel_, 0, 0});

  // HLOG_INFO << "==== init ====  filtered vel !!! "
  //           << "origin vec," << v_by_wheel_ << " ,avg wheel, " << avg_vel
  //           << ",filter vel," << fl_vel(0) << " ,wheel vel,"
  //           << cur.rear_right_speed <<" ,avg2,"<<avg_vel2<<"
  //           ,lastvel,"<<last_avg_vel<<",acc,"<<(avg_vel2 - last_avg_vel) /
  //           delta_t;

  cur_odom_data.loc_vel = {avg_vel, 0, 0};
  cur_odom_data.loc_omg = {0, 0, avg_w};
  double acc_x = (avg_vel2 - last_avg_vel) / delta_t;

  /////////////////////////////////////
  static std::deque<double> wheel_acc = {};
  if (wheel_acc.size() > 20) {
    wheel_acc.pop_front();
  } else {
    if (acc_x < 10.0) {
      wheel_acc.push_back(acc_x);
    }
  }
  double total_acc = 0.0;
  for (auto i : wheel_acc) {
    total_acc += i;
  }
  double avg_acc = 0.0;
  if (wheel_acc.size() > 18) {
    avg_acc = total_acc / static_cast<double>(wheel_acc.size());
  } else {
    avg_acc = 0.0;
  }

  cur_odom_data.loc_acc = {avg_acc, 0, 0};

  // HLOG_INFO << "==== init ====  filtered vel !!! "
  //           << "left_diff," << left_diff << " ,right_diff, " << right_diff
  //           << ",filter vel," << fl_vel(0) << " ,wheel vel,"
  //           << cur.rear_right_speed << " ,avg2," << avg_vel2 << " ,acc,"
  //           << avg_acc << ",yaw," << delta_yaw << ",avg yaw," << avg_w;

  AddOdomData(cur_odom_data/*, delta_dist*/);

  double yaw = 2 * atan2(qat_.z(), qat_.w());
  if (yaw > M_PI) {
    yaw -= 2 * M_PI;
  }
  if (yaw < -M_PI) {
    yaw += 2 * M_PI;
  }

  // v_by_wheel_ = delta_dist / delta_t;
  // w_by_wheel_ = delta_yaw / delta_t;

  /*  std::cout << "delta dis:" << delta_dist << ",delta angle:" << delta_yaw
             << std::endl;
   std::cout << "x,y,yaw:" << pos_[0] << "," << pos_[1] << "," << yaw
             << std::endl; */

  return true;
}

}  // namespace dr
}  // namespace mp
}  // namespace hozon
