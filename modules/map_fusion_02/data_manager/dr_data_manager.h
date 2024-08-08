/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： dr_data_manager.h
 *   author     ： chenlongxi
 *   date       ： 2023.09
 ******************************************************************************/
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "base/utils/log.h"
#include "modules/map_fusion_02/base/element_map.h"
#include "modules/map_fusion_02/data_manager/data_buffer.h"
#include "perception-base/base/utils/macros.h"
#include "perception-lib/lib/config_manager/config_manager.h"

namespace hozon {
namespace mp {
namespace mf {

class DrData {
 public:
  DEFINE_PTR(DrData)
  DEFINE_CONST_PTR(DrData)
 public:
  double timestamp = -1.0;
  Eigen::Vector3d translation;       // 平移
  Eigen::Quaterniond quaternion;     // 旋转
  Eigen::Affine3d pose;              // 平移+旋转
  Eigen::Vector3d linear_velocity;   // 线速度
  Eigen::Vector3d angular_velocity;  // 角速度
  Eigen::Vector3d acceleration;      // 加速度
  int gear = 100;

  DrData Interpolate(const double scale, const DrData& end,
                     double timestamp) const {
    DrData res;
    res.timestamp = timestamp;
    res.translation =
        this->translation + (end.translation - this->translation) * scale;
    res.quaternion = this->quaternion.slerp(scale, end.quaternion);
    res.pose =
        Eigen::Translation3d(res.translation) * Eigen::Affine3d(res.quaternion);

    if (scale >= 0.5) {
      res.linear_velocity = end.linear_velocity;
      res.angular_velocity = end.angular_velocity;
      res.acceleration = end.acceleration;
      res.gear = end.gear;
    } else {
      res.linear_velocity = this->linear_velocity;
      res.angular_velocity = this->angular_velocity;
      res.acceleration = this->acceleration;
      res.gear = this->gear;
    }
    return res;
  }
};

class DrDataManager {
 public:
  bool Init();
  MessageBuffer<DrData::ConstPtr>& GetDrBuffer();
  DrData::ConstPtr GetDrPoseByTimeStamp(double timestamp);
  Eigen::Affine3d GetDeltaPose();
  Eigen::Affine3d GetCurrentPose();
  bool IsStaticState();
  bool IsTurnState();
  bool GetTurnState() const;
  bool PushDrData(const LocInfo::ConstPtr& latest_localization);
  void PushLocalDrData(double timestamp, const DrData::ConstPtr& local_pose);
  void SetTimeStampDrPose(DrData::ConstPtr);
  ~DrDataManager() = default;

 private:
  /** @brief sensor data buffer. */
  MessageBuffer<DrData::ConstPtr> origin_dr_buffer_;
  MessageBuffer<DrData::ConstPtr> local_dr_buffer_;

  // 静止策略状态统计量
  int turn_count_ = 0;
  int turn_count_threshold_ = 4;
  bool turn_state_ = false;
  double static_dist_accu_ = 0.0;
  double static_angular_accu_ = 0.0;

  bool use_idle_strategy_ = true;
  float u_turn_angle_velocity_limit_;
  float radial_velocity_limit_;
  float angular_velocity_limit_;
  float dist_limit_;
  float angle_limit_;
  // 类初始化相关
  std::mutex mutex_;
  bool inited_ = false;
  // 上一帧到当前帧位姿
  Eigen::Affine3d T_cur_last_;
  Eigen::Affine3d cur_T_w_v_;
  Eigen::Affine3d last_T_w_v_;

  // get instance by Instance()
  DECLARE_SINGLETON_PERCEPTION(DrDataManager)
};

#define DR_MANAGER DrDataManager::Instance()

}  // namespace mf
}  // namespace mp
}  // namespace hozon
