/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: chenlongxi
 *Date: 2023-09-06
 *****************************************************************************/
#pragma once

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "modules/local_mapping/base/location/dr.h"
#include "modules/local_mapping/base/location/location.h"
#include "modules/local_mapping/lib/datalogger/data_buffer.h"
#include "perception-base/base/utils/macros.h"

namespace hozon {
namespace mp {
namespace lm {

class PoseManager {
 public:
  bool Init();
  MessageBuffer<DrDataConstPtr>& GetDrBuffer();
  DrDataConstPtr GetDrPoseByTimeStamp(double timestamp);
  Eigen::Affine3d GetDeltaPose();
  Eigen::Affine3d GetCurrentPose();
  bool IsStaticState();
  bool IsTurnState();
  bool GetTurnState() const;
  bool PushDrData(const LocationConstPtr& latest_localization);
  void PushLocalDrData(double timestamp, const DrDataConstPtr& local_pose);
  void SetTimeStampDrPose(DrDataConstPtr);
  ~PoseManager() = default;

 private:
  /** @brief sensor data buffer. */
  MessageBuffer<DrDataConstPtr> origin_dr_buffer_;
  MessageBuffer<DrDataConstPtr> local_dr_buffer_;

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
  DECLARE_SINGLETON_PERCEPTION(PoseManager)
};

#define POSE_MANAGER PoseManager::Instance()

}  // namespace lm
}  // namespace mp
}  // namespace hozon
