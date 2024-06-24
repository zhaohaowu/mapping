/******************************************************************************
 *   Copyright (C) 2024 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： monitor.cc
 *   author     ： zhangyu0435
 *   date       ： 2024.01

******************************************************************************/
#include "modules/location/fusion_center/lib/monitor.h"
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>
#include "modules/util/include/util/mapping_log.h"
namespace hozon {
namespace mp {
namespace loc {
namespace fc {
bool Monitor::Init(const std::string& configfile) {
  boost::filesystem::path path(configfile);
  if (!boost::filesystem::exists(path)) {
    HLOG_ERROR << "LOC FC Monitor conf:" << configfile << " not exist";
    return false;
  }
  YAML::Node config_parser = YAML::LoadFile(configfile);
  params_.ins_deque_max_size =
      config_parser["ins_deque_max_size"].as<uint32_t>();
  params_.pe_fault_deque_max_size =
      config_parser["pe_fault_deque_max_size"].as<uint32_t>();
  params_.fc_deque_max_size = config_parser["fc_deque_max_size"].as<uint32_t>();
  params_.use_fault = config_parser["use_fault"].as<bool>();

  // fault-pe-123,124,130
  params_.pe_fault = config_parser["pe_fault"].as<bool>();
  // fault-128---PoseJump
  params_.fault_128 = config_parser["fault_128"].as<bool>();
  params_.fc_ts_maxdiff = config_parser["fc_ts_maxdiff"].as<double>();
  params_.fc_yaw_maxdiff = config_parser["fc_yaw_maxdiff"].as<double>();
  params_.fc_max_float_ratio_vert =
      config_parser["fc_max_float_ratio_vert"].as<double>();
  params_.fc_max_single_dist_hori =
      config_parser["fc_max_single_dist_hori"].as<double>();
  params_.fc_minvel = config_parser["fc_minvel"].as<double>();

  HLOG_INFO << "LOC-FC-Monitor Init Successfully";
  return true;
}
void Monitor::Clear() {
  {
    std::unique_lock<std::shared_mutex> lock(ins_deque_mutex_);
    ins_deque_.clear();
  }
  {
    std::unique_lock<std::shared_mutex> lock(fc_deque_mutex_);
    fc_deque_.clear();
  }
  {
    std::unique_lock<std::shared_mutex> lock(pe_fault_deque_mutex_);
    pe_fault_deque_.clear();
  }
  {
    std::unique_lock<std::shared_mutex> lock(fault_state_mutex_);
    fault_state.Reset();
  }
}

void Monitor::OnIns(const Node& node) {
  std::unique_lock<std::shared_mutex> lock(ins_deque_mutex_);
  ins_deque_.push_back(std::make_shared<Node>(node));
  ShrinkQueue(&ins_deque_, params_.ins_deque_max_size);
}

void Monitor::OnFc(const Node& node) {
  std::unique_lock<std::shared_mutex> lock(fc_deque_mutex_);
  fc_deque_.push_back(std::make_shared<Node>(node));
  ShrinkQueue(&fc_deque_, params_.fc_deque_max_size);
}

void Monitor::OnPeFault(const HafNodeInfo& msg) {
  std::unique_lock<std::shared_mutex> lock(pe_fault_deque_mutex_);

  FaultNode fnode;
  fnode.ticktime = msg.header().publish_stamp();
  fnode.fault = msg.warn_info();

  pe_fault_deque_.push_back(std::make_shared<FaultNode>(fnode));
  ShrinkQueue(&pe_fault_deque_, params_.pe_fault_deque_max_size);
}

bool Monitor::MonitorFault() {
  if (!params_.use_fault) {
    return false;
  }
  std::unique_lock<std::shared_mutex> lock(fault_state_mutex_);
  // pe传输过来的故障----fault 123,124,130
  if (params_.pe_fault) {
    uint32_t pe_fault_state = 0;

    pe_fault_state = MergePeFault();
    if (pe_fault_state == 123) {
      fault_state.pecep_lane_error = true;
      HLOG_DEBUG << "pe fault(123):PE No Peception lane";
    } else {
      fault_state.pecep_lane_error = false;
    }

    if (pe_fault_state == 124) {
      fault_state.map_lane_error = true;
      HLOG_DEBUG << "pe fault(124):PE No Map lane";
    } else {
      fault_state.map_lane_error = false;
    }

    if (pe_fault_state == 130) {
      fault_state.fc_map_lane_match_error = true;
      HLOG_DEBUG << "fc fault(130):Map and Peception Lane Mismatch With FC";
    } else {
      fault_state.fc_map_lane_match_error = false;
    }
  }

  // fault(128)
  if (params_.fault_128) {
    if (PoseJumpSingleFrameVehicle()) {
      fault_state.fc_single_jump_error = true;
      HLOG_DEBUG << "fc fault(128):FC SingleFrame Mutation";
    } else {
      fault_state.fc_single_jump_error = false;
    }
  }
  return true;
}

bool Monitor::PoseJumpSingleFrameVehicle() {
  Node curr_node, prev_node;
  {
    std::unique_lock<std::shared_mutex> lock(fc_deque_mutex_);
    int size = static_cast<int>(fc_deque_.size());
    if (size < 2) {
      return false;
    }
    curr_node = *fc_deque_.at(size - 1);
    prev_node = *fc_deque_.at(size - 2);
  }

  const double delta_t = fabs(curr_node.ticktime - prev_node.ticktime);
  if (delta_t > params_.fc_ts_maxdiff) {
    HLOG_ERROR << "fc ticktime diff error:" << delta_t;
    return false;
  }

  const auto prev_rot = Sophus::SO3d::exp(prev_node.orientation);
  const auto prev_body_vel = prev_rot.inverse() * prev_node.velocity;
  // 低速不判断
  if (prev_body_vel(0) < params_.fc_minvel &&
      prev_body_vel(1) < params_.fc_minvel) {
    return false;
  }

  const auto prev_body_acc = prev_node.linear_accel * 9.8;
  const double delta_estimate_vert =
      prev_body_vel(0) * delta_t + 0.5 * prev_body_acc(0) * delta_t * delta_t;

  const auto pose_diff = Node2SE3(prev_node).inverse() * Node2SE3(curr_node);

  double vert_float_ratio = 0.0;
  if (fabs(delta_estimate_vert) > 1e-6) {
    vert_float_ratio =
        fabs((pose_diff.translation().x() - delta_estimate_vert) /
             delta_estimate_vert);
  }

  const double yaw_diff = fabs(pose_diff.so3().log().z());

  if (fabs(pose_diff.translation().y()) > params_.fc_max_single_dist_hori ||
      vert_float_ratio > params_.fc_max_float_ratio_vert ||
      fabs(pose_diff.so3().log().z()) > params_.fc_yaw_maxdiff) {
    HLOG_INFO << "Vehicle y diff:" << pose_diff.translation().y()
               << " larger than thr:" << params_.fc_max_single_dist_hori
               << " or x ratio:" << vert_float_ratio
               << " larger than thr:" << params_.fc_max_float_ratio_vert
               << " or yaw diff:" << pose_diff.so3().log().z()
               << " larger than thr:" << params_.fc_yaw_maxdiff;
    return true;
  }

  return false;
}

Sophus::SE3d Monitor::Node2SE3(const Node& node) {
  return Sophus::SE3d(Sophus::SO3d::exp(node.orientation), node.enu);
}

uint32_t Monitor::MergePeFault() {
  std::unique_lock<std::shared_mutex> lock(pe_fault_deque_mutex_);
  static uint32_t res = 0;

  if (pe_fault_deque_.empty()) {
    return res;
  }

  // 故障判断
  FaultNode fnode;
  fnode = *pe_fault_deque_.back();
  if (abs(pre_pe_ticktime_ - fnode.ticktime) > 0.001) {
    res = fnode.fault;
    pre_pe_ticktime_ = fnode.ticktime;
  }

  return res;
}

}  // namespace fc
}  // namespace loc
}  // namespace mp
}  // namespace hozon
