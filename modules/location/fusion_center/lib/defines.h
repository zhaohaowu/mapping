/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： defines.h
 *   author     ： lilanxing
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <Eigen/Eigen>
#include <utility>
#include <vector>

#include "proto/localization/node_info.pb.h"
#include "proto/soc/sensor_imu_ins.pb.h"

namespace hozon {
namespace mp {
namespace loc {
namespace fc {

struct Params {
  bool recv_imu = false;
  bool recv_ins = false;
  bool recv_dr = false;
  bool recv_pe = false;
  uint32_t imu_deque_max_size = 1000;
  uint32_t ins_deque_max_size = 1000;
  uint32_t dr_deque_max_size = 1000;
  uint32_t pe_deque_max_size = 500;
  bool passthrough_ins = false;
  bool smooth_outputs = false;
  bool use_smooth_yaw = false;
  uint32_t search_state_cnt = 30;
  double no_mm_min_time = 0.3;
  double no_mm_max_time = 3;
  bool use_ins_predict_mm = true;
  bool use_dr_measurement = false;
  uint32_t run_fusion_interval_ms = 10;
  uint32_t window_size = 30;
  bool require_local_pose = false;
  bool use_debug_txt = false;
  bool lateral_error_compensation = false;
  double max_dr_pe_horizontal_dist_error = 1.0;
  double max_dr_pe_heading_error = 2.5;
  double max_fc_pe_horizontal_dist_error = 0.1;
  double refpoint_update_dist = 20000.0;

  std::vector<std::pair<uint32_t, uint32_t>> ins_init_status;
};

enum NodeType { NONE = -1, INS = 0, DR = 1, MM = 2, POSE_ESTIMATE = 3 };

struct Node {
  uint32_t seq = 0;
  double ticktime = -1;
  NodeType type = NodeType::NONE;
  Eigen::Vector3d refpoint = Eigen::Vector3d::Zero();
  Eigen::Vector3d blh = Eigen::Vector3d::Zero();
  Eigen::Vector3d enu = Eigen::Vector3d::Zero();
  Eigen::Vector3d orientation = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d linear_vel_VRF = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d linear_accel = Eigen::Vector3d::Zero();
  Eigen::Vector3d b_a = Eigen::Vector3d::Zero();
  Eigen::Vector3d b_g = Eigen::Vector3d::Zero();
  Eigen::Quaterniond quaternion;
  Eigen::Matrix<double, 15, 15> cov = Eigen::Matrix<double, 15, 15>::Zero();
  uint32_t sys_status = 0;
  uint32_t rtk_status = 0;
  uint32_t location_state = 0;
  double heading = 0;
  bool state = false;
};

struct Context {
  hozon::soc::ImuIns imuins;
  hozon::localization::HafNodeInfo ins;
  Node ins_node;
  Node dr_node;
  Node global_node;
  Node local_node;
};

}  // namespace fc
}  // namespace loc
}  // namespace mp
}  // namespace hozon
