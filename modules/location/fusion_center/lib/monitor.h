/******************************************************************************
 *   Copyright (C) 2024 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： monitor.h
 *   author     ： zhangyu0435
 *   date       ： 2024.01
 ******************************************************************************/

#pragma once
#include <deque>
#include <memory>
#include <shared_mutex>
#include <string>
#include <Sophus/se3.hpp>
#include "modules/location/fusion_center/lib/defines.h"

namespace hozon {
namespace mp {
namespace loc {
namespace fc {

struct MonitorParams {
  bool use_fault = false;
  uint32_t ins_deque_max_size = 100;
  uint32_t fc_deque_max_size = 100;

  // fault-128
  bool fault_128 = true;
  double fc_ts_maxdiff = 0.1;    // s
  double fc_vel_max = 33.3;       // Longitudinal speed (m/s)---120KM/h
  double fc_acc_max = 3;          // Longitudinal acceleration (m/s2)
  double fc_yaw_maxdiff = 0.001;  // (deg)
  double fc_minvel = 0.1;
  double fc_max_float_ratio_vert = 0.5;
  double fc_max_single_dist_hori = 0.2;  // unit: m
};

// 错误状态值判断
struct FaultState {
  bool pecep_lane_error = false;      // fault(123)---无有效感知车道线
  bool map_lane_error = false;        // fault(124)---无有效地图车道线
  bool fc_single_jump_error = false;  // fault(128)---fc单帧跳变
  bool fc_map_lane_match_error = false;  // fault(130)

  void Reset() {
    pecep_lane_error = false;
    map_lane_error = false;
    fc_single_jump_error = false;
    fc_map_lane_match_error = false;
  }
};

template <typename T>
void ShrinkQueue(T* const queue, uint32_t maxsize) {
  if (!queue) {
    return;
  }
  while (queue->size() > maxsize) {
    queue->pop_front();
  }
}

class Monitor {
 public:
  bool Init(const std::string& configfile);
  void OnIns(const Node& node);
  void OnFc(const Node& node);
  bool MonitorFault();
  void Clear();

 private:
  static Sophus::SE3d Node2SE3(const Node& node);
  bool PoseJumpSingleFrameVehicle();

 public:
  FaultState fault_state;

 private:
  MonitorParams params_;
  std::shared_mutex ins_deque_mutex_;
  std::deque<std::shared_ptr<Node>> ins_deque_;
  std::shared_mutex fc_deque_mutex_;
  std::deque<std::shared_ptr<Node>> fc_deque_;
};

}  // namespace fc
}  // namespace loc
}  // namespace mp
}  // namespace hozon
