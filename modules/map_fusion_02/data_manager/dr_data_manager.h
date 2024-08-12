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

class DrDataManager {
 public:
  bool Init();
  MessageBuffer<LocInfo::ConstPtr>& GetDrBuffer();
  LocInfo::ConstPtr GetDrPoseByTimeStamp(double timestamp);
  Eigen::Affine3d GetDeltaPose();
  Eigen::Affine3d GetCurrentPose();
  bool IsStaticState();
  bool IsTurnState();
  bool GetTurnState() const;
  bool PushDrData(const LocInfo::ConstPtr& latest_localization);
  void PushLocalDrData(double timestamp, const LocInfo::ConstPtr& local_pose);
  void SetTimeStampDrPose(const LocInfo::ConstPtr& dr_pose_ptr);
  ~DrDataManager() = default;

 private:
  // 存储原始的location数据信息
  MessageBuffer<LocInfo::ConstPtr> origin_dr_buffer_;
  // 存储接收到local_map时刻的location数据信息
  MessageBuffer<LocInfo::ConstPtr> local_dr_buffer_;

  // 静止策略状态统计量
  int turn_count_{0};
  int turn_count_threshold_{4};
  bool turn_state_ = false;
  double static_dist_accu_{0.0};
  double static_angular_accu_{0.0};

  bool use_idle_strategy_ = true;
  float u_turn_angle_velocity_limit_{0.0};
  float radial_velocity_limit_{0.0};
  float angular_velocity_limit_{0.0};
  float dist_limit_{0.0};
  float angle_limit_{0.0};
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
