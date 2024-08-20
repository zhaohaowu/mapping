/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： junction_status_manager.cc
 *   author     ： chenlongxi
 *   date       ： 2023.09
 ******************************************************************************/

#include "modules/map_fusion_02/data_manager/junction_status_manager.h"

namespace hozon {
namespace mp {
namespace mf {

bool JuncStatusManager::Init() {
  if (inited_) {
    return true;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  junc_status.road_scene_type = RoadSceneType::GENERAL_ROAD;
  junc_status.last_road_scene_type = RoadSceneType::GENERAL_ROAD;
  junc_status.along_path_vec = Eigen::Vector3d(0, 0, 0);
  junc_status.cross_before_lane_num = 0;
  junc_status.cross_after_lane_num = 0;
  junc_status.next_satisefy_lane_seg.clear();
  inited_ = true;
  return true;
}

void JuncStatusManager::UpdateStatus(const JunctionInfo& junc_info) {
  std::lock_guard<std::mutex> lock(mutex_);
  junc_status = junc_info;
}

JunctionInfo JuncStatusManager::GetJuncStatus() {
  std::lock_guard<std::mutex> lock(mutex_);
  return junc_status;
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
