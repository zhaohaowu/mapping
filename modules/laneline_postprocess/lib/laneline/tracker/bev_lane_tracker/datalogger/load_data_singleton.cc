/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: likuan
 *Date: 2023-09-06
 *****************************************************************************/
#include "modules/laneline_postprocess/lib/laneline/tracker/bev_lane_tracker/datalogger/load_data_singleton.h"

namespace hozon {
namespace mp {
namespace environment {

InputDataSingleton::InputDataSingleton() { CHECK_EQ(this->Init(), true); }

bool InputDataSingleton::Init() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (inited_) {
    return true;
  }
  dr_data_buffer_.set_capacity(2);
  lanes_buffer_.set_capacity(2);
  roadedges_buffer_.set_capacity(2);

  dr_data_buffer_.clear();
  lanes_buffer_.clear();
  roadedges_buffer_.clear();

  inited_ = true;
  return true;
}

bool InputDataSingleton::IsStaticState(StaticStrategyParam config) {
  const auto& cur_pose = dr_data_buffer_.back()->pose;
  const auto& velocity = dr_data_buffer_.back()->linear_velocity;
  const auto& angular_velocity = dr_data_buffer_.back()->angular_velocity;

  if (!config.use_idle_strategy()) return false;

  // 判断map更新的pose与当前判断静止时的pose的差值
  Eigen::Affine3d delta_pose = cur_pose.inverse() * map_update_pose_;
  // 从 delta_pose 中提取旋转矩阵
  Eigen::Matrix3d rotation_matrix = delta_pose.rotation();
  // 提取欧拉角 (顺序：X, Y, Z，即 Roll, Pitch, Yaw)
  Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(0, 1, 2);
  if (std::abs(euler_angles[2]) > config.angle_limit()) {
    // 角度大， 则退出静止策略
    return false;
  }
  if (delta_pose.translation().norm() > config.dist_limit()) {
    // 距离大， 则退出静止策略
    return false;
  }

  bool is_static_state = false;
  if (std::abs(velocity.x()) > config.radial_velocity_limit()) {
    // 径向速度大， 则退出静止策略
    is_static_state = false;
  } else if (std::abs(angular_velocity.z()) > config.angular_velocity_limit()) {
    // 角速度大， 则退出静止策略
    is_static_state = false;
  } else if (static_dist_accu_ > config.dist_limit()) {
    // 静止状态运行长度超出阈值， 则退出静止策略
    is_static_state = false;
  } else if (static_angular_accu_ > config.angle_limit()) {
    // 静止状态运行角度超出阈值， 则退出静止策略
    is_static_state = false;
  } else {
    is_static_state = true;
  }

  if (is_static_state) {  // 静止状态， 角度和长度做积分
    static_dist_accu_ += std::abs(velocity.x()) * 0.1;
    static_angular_accu_ += std::abs(angular_velocity.z()) * 0.1;
  } else {
    static_dist_accu_ = 0;
    static_angular_accu_ = 0;
  }

  return is_static_state;
}

bool InputDataSingleton::IsTurnState(StaticStrategyParam config) {
  if (dr_data_buffer_.buffer_size() < 2) {
    turn_count_ = 0;
    turn_state_ = false;
    return false;
  }
  float yaw_rate = dr_data_buffer_.back()->angular_velocity[2];
  if (std::abs(yaw_rate) > config.u_turn_angle_velocity_limit()) {
    turn_count_++;
    if (turn_count_ >= turn_count_threshold_) {
      turn_state_ = true;
    } else {
      turn_state_ = false;
    }
  } else {
    turn_count_ = 0;
    turn_state_ = false;
  }
  return turn_state_;
}

bool InputDataSingleton::GetTurnState() { return turn_state_; }

void InputDataSingleton::SetMapUpdatePose(const Eigen::Affine3d& pose) {
  map_update_pose_ = pose;
}

}  // namespace environment
}  // namespace mp
}  // namespace hozon
