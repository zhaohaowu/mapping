/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： dr_data_manager.cc
 *   author     ： chenlongxi
 *   date       ： 2023.09
 ******************************************************************************/

#include "modules/map_fusion_02/data_manager/dr_data_manager.h"

namespace hozon {
namespace mp {
namespace mf {

namespace perception_lib = hozon::perception::lib;
DrDataManager::DrDataManager() { CHECK_EQ(this->Init(), true); }

MessageBuffer<DrData::ConstPtr>& DrDataManager::GetDrBuffer() {
  return local_dr_buffer_;
}

bool DrDataManager::Init() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (inited_) {
    return true;
  }

  origin_dr_buffer_.set_capacity(1000);
  local_dr_buffer_.set_capacity(2);

  origin_dr_buffer_.clear();
  local_dr_buffer_.clear();

  auto* config_manager = perception_lib::ConfigManager::Instance();
  const perception_lib::ModelConfig* model_config = nullptr;
  if (!config_manager->GetModelConfig("static_strategy_param", &model_config)) {
    HLOG_ERROR << "Parse static_strategy_param config failed.";
    return false;
  }

  if (!model_config->get_value("use_idle_strategy", &use_idle_strategy_)) {
    HLOG_ERROR << "Get use_idle_strategy failed!";
    return false;
  }

  if (!model_config->get_value("radial_velocity_limit",
                               &radial_velocity_limit_)) {
    HLOG_ERROR << "Get radial_velocity_limit failed!";
    return false;
  }

  if (!model_config->get_value("angular_velocity_limit",
                               &angular_velocity_limit_)) {
    HLOG_ERROR << "Get angular_velocity_limit failed!";
    return false;
  }

  if (!model_config->get_value("dist_limit", &dist_limit_)) {
    HLOG_ERROR << "Get dist_limit failed!";
    return false;
  }

  if (!model_config->get_value("angle_limit", &angle_limit_)) {
    HLOG_ERROR << "Get angle_limit failed!";
    return false;
  }

  if (!model_config->get_value("u_turn_angle_velocity_limit",
                               &u_turn_angle_velocity_limit_)) {
    HLOG_ERROR << "Get u_turn_angle_velocity_limit failed!";
    return false;
  }

  inited_ = true;
  return true;
}

DrData::ConstPtr DrDataManager::GetDrPoseByTimeStamp(double timestamp) {
  DrData::ConstPtr before = nullptr;
  DrData::ConstPtr after = nullptr;
  origin_dr_buffer_.get_messages_around(timestamp, before, after);

  if (before == nullptr && after == nullptr) {
    HLOG_ERROR << "GetDrPoseForTime is null: " << timestamp;
    return nullptr;
  }

  if (before == nullptr) {
    DrData::Ptr dr_ptr = std::make_shared<DrData>();

    double delta_t = timestamp - after->timestamp;
    dr_ptr->timestamp = timestamp;
    dr_ptr->translation =
        after->translation +
        after->quaternion * (after->linear_velocity * delta_t);
    dr_ptr->quaternion = after->quaternion;
    Eigen::Vector3d delta_ang = after->angular_velocity * delta_t;
    if (delta_ang.norm() > 1e-12) {
      dr_ptr->quaternion =
          dr_ptr->quaternion * Eigen::Quaterniond(Eigen::AngleAxisd(
                                   delta_ang.norm(), delta_ang.normalized()));
      dr_ptr->quaternion = dr_ptr->quaternion.normalized();
    }
    dr_ptr->pose = Eigen::Translation3d(dr_ptr->translation) *
                   Eigen::Affine3d(dr_ptr->quaternion);
    dr_ptr->linear_velocity = after->linear_velocity;
    dr_ptr->angular_velocity = after->angular_velocity;
    dr_ptr->acceleration = after->acceleration;
    dr_ptr->gear = after->gear;
    return dr_ptr;
  }

  if (before->timestamp == timestamp && after->timestamp == timestamp) {
    return before;
  }

  // before time == after time is not happend for this if
  if (after->timestamp <= before->timestamp) {
    HLOG_ERROR << "GetDrPoseForTime: after->timestamp <= before->timestamp: "
               << after->timestamp << " < " << before->timestamp;
    return nullptr;
  }

  double ratio =
      (timestamp - before->timestamp) / (after->timestamp - before->timestamp);
  auto dr_pose_state =
      std::make_shared<DrData>(before->Interpolate(ratio, *after, timestamp));
  return dr_pose_state;
}

void DrDataManager::SetTimeStampDrPose(DrData::ConstPtr dr_pose_ptr) {
  if (dr_pose_ptr) {
    cur_T_w_v_ = dr_pose_ptr->pose;
    T_cur_last_ = cur_T_w_v_.inverse() * last_T_w_v_;
    last_T_w_v_ = cur_T_w_v_;
  }
}

bool DrDataManager::PushDrData(const LocInfo::ConstPtr& latest_localization) {
  HLOG_DEBUG << "latest_localization timestamp: " << SET_PRECISION(20)
             << latest_localization->timestamp;
  if (origin_dr_buffer_.is_empty() ||
      origin_dr_buffer_.back()->timestamp < latest_localization->timestamp) {
    DrData::Ptr dr_data_ptr = std::make_shared<DrData>();
    dr_data_ptr->timestamp = latest_localization->timestamp;
    dr_data_ptr->translation = latest_localization->position;
    dr_data_ptr->quaternion = latest_localization->quaternion;
    dr_data_ptr->angular_velocity = latest_localization->angular_vrf;
    dr_data_ptr->linear_velocity = latest_localization->linear_vrf;
    dr_data_ptr->pose = Eigen::Translation3d(dr_data_ptr->translation) *
                        Eigen::Affine3d(dr_data_ptr->quaternion);
    origin_dr_buffer_.push_new_message(latest_localization->timestamp,
                                       dr_data_ptr);
    return true;
  }
  return false;
}

void DrDataManager::PushLocalDrData(double timestamp,
                                    const DrData::ConstPtr& local_pose) {
  local_dr_buffer_.push_new_message(timestamp, local_pose);
}

bool DrDataManager::IsStaticState() {
  const auto& velocity = origin_dr_buffer_.back()->linear_velocity;
  const auto& angular_velocity = origin_dr_buffer_.back()->angular_velocity;

  if (!use_idle_strategy_) {
    return false;
  }

  bool is_static_state = false;
  if (std::abs(velocity.x()) > radial_velocity_limit_) {
    // 径向速度大， 则退出静止策略
    is_static_state = false;
  } else if (std::abs(angular_velocity.z()) > angular_velocity_limit_) {
    // 角速度大， 则退出静止策略
    is_static_state = false;
  } else if (static_dist_accu_ > dist_limit_) {
    // 静止状态运行长度超出阈值， 则退出静止策略
    is_static_state = false;
  } else if (static_angular_accu_ > angle_limit_) {
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

bool DrDataManager::IsTurnState() {
  if (origin_dr_buffer_.buffer_size() < 2) {
    turn_count_ = 0;
    turn_state_ = false;
    return false;
  }
  double yaw_rate = origin_dr_buffer_.back()->angular_velocity[2];
  if (std::abs(yaw_rate) > u_turn_angle_velocity_limit_) {
    turn_count_++;
    turn_state_ = (turn_count_ >= turn_count_threshold_);
  } else {
    turn_count_ = 0;
    turn_state_ = false;
  }
  return turn_state_;
}

bool DrDataManager::GetTurnState() const { return turn_state_; }

Eigen::Affine3d DrDataManager::GetDeltaPose() { return T_cur_last_; }

Eigen::Affine3d DrDataManager::GetCurrentPose() { return cur_T_w_v_; }

}  // namespace mf
}  // namespace mp
}  // namespace hozon
