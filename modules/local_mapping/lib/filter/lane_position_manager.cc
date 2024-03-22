// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Hozon (hozon@hozon.com)
// @file: lane_position_manager.cc
// @brief: lane pos for lane tracker

#include "modules/local_mapping/lib/filter/lane_position_manager.h"

#include <algorithm>

#include "modules/local_mapping/utils/lane_utils.h"

namespace hozon {
namespace mp {
namespace lm {

void LanePositionManager::Init() { inited_ = true; }
std::vector<bool> LanePositionManager::CheckLanesDistance(
    const std::vector<LaneLinePtr>* tracked_lanelines) {
  std::vector<bool> is_far_laneline(tracked_lanelines->size(), false);
  for (int i = 0; i < tracked_lanelines->size(); ++i) {
    auto vehicle_min =
        static_cast<float>(tracked_lanelines->at(i)->vehicle_points.front()[0]);
    auto vehicle_max =
        static_cast<float>(tracked_lanelines->at(i)->vehicle_points.back()[0]);
    if (vehicle_max <= dist_separation_point_) {
      is_far_laneline[i] = false;
    } else if ((vehicle_max > dist_separation_point_) &&
               (vehicle_min <= dist_separation_point_)) {
      is_far_laneline[i] = false;
    } else {
      is_far_laneline[i] = true;
    }
  }
  return is_far_laneline;
}
bool LanePositionManager::IsMainRoadAboutDisappear(
    const std::vector<LaneLinePtr>* tracked_lanelines,
    const std::vector<bool>& far_line_index) {
  // 获取近处的车道线
  std::vector<LaneLinePtr> near_lanelines;
  for (int i = 0; i < far_line_index.size(); ++i) {
    if (!far_line_index[i]) {
      auto laneline = tracked_lanelines->at(i);
      near_lanelines.push_back(laneline);
    }
  }

  // const auto& lane_tracker_pipeline_param =
  //     lane_post_process_param.lane_tracker_pipeline_param();
  float ref_min = 0;
  // lane_tracker_pipeline_param.lane_pose_setter_param().ref_min();
  float ref_length = 5;
  // lane_tracker_pipeline_param.lane_pose_setter_param().ref_length();
  int sample_num = 5;
  // lane_tracker_pipeline_param.lane_pose_setter_param().sample_point_num();

  // 近处的车道线进行位置赋值，并找到主车道线
  SetLanePosition(ref_min, ref_length, sample_num, near_lanelines);
  float ego_left_loc, ego_right_loc = std::numeric_limits<float>::min();
  for (auto& laneline : near_lanelines) {
    if (laneline->te_position == LaneLinePosition::EGO_LEFT) {
      ego_left_loc = laneline->vehicle_points.back()[0];
    } else if (laneline->te_position == LaneLinePosition::EGO_RIGHT) {
      ego_right_loc = laneline->vehicle_points.back()[0];
    } else {
      continue;
    }
  }
  // 解决远端没有线，但是近端小于5m的case
  if (near_lanelines.size() == far_line_index.size()) {
    return false;
  }
  if (ego_left_loc < dist_separation_point_ &&
      ego_right_loc < dist_separation_point_) {
    // 如果主车道线都丢失， 会拿远端的车道线
    // 如果主车道线丢失其中一根，另一根远端点小于5米，拿远端的车道线。
    // 如果主车道线都存在， 远短点都小于5米， 拿远端的车道线。
    return true;
  }

  return false;
}
void LanePositionManager::revise_lanes_flag(
    const std::vector<LaneLinePtr>* tracked_lanelines,
    std::vector<bool>* far_lanes_flag) {
  std::vector<LaneLinePtr> near_lanelines;
  float near_start = std::numeric_limits<float>::max(),
        near_end = std::numeric_limits<float>::max();
  for (int i = 0; i < far_lanes_flag->size(); ++i) {
    if (!far_lanes_flag->at(i)) {
      auto laneline = tracked_lanelines->at(i);
      near_lanelines.emplace_back(laneline);
      near_start = std::min(
          static_cast<float>(laneline->vehicle_points.front()[0]), near_start);
      near_end = std::min(
          static_cast<float>(laneline->vehicle_points.back()[0]), near_end);
    }
  }
  for (int i = 0; i < far_lanes_flag->size(); ++i) {
    if (far_lanes_flag->at(i)) {
      const auto& laneline = tracked_lanelines->at(i);
      float overlap_start = std::max(
          near_start, static_cast<float>(laneline->vehicle_points.front()[0]));
      float overlap_end = std::min(
          near_end, static_cast<float>(laneline->vehicle_points.back()[0]));
      if (overlap_end - overlap_start > 10.0) {
        far_lanes_flag->at(i) = false;
      }
    }
  }
  return;
}
bool LanePositionManager::CheckStableFlag(
    const std::vector<LaneLinePtr>* tracked_lanelines, bool disappear_flag) {
  // 判断是否稳定前行
  bool stable_flag = false;
  int count_lane = 0;
  for (int i = 0; i < tracked_lanelines->size(); ++i) {
    LaneLinePtr lane_target = (*tracked_lanelines)[i];
    auto iter_d = lane_d_map.find(lane_target->id);
    if (iter_d != lane_d_map.end()) {
      auto& d_error = std::get<1>(iter_d->second);
      if (d_error < d_change_threshold_) {
        count_lane++;
      }
    }
    if (count_lane >= 2 && disappear_flag == false) {
      stable_flag = true;
      break;
    }
  }
  return stable_flag;
}
bool LanePositionManager::CheckEgoPose(LaneLinePosition pose) {
  return ((pose == LaneLinePosition::EGO_LEFT) ||
          (pose == LaneLinePosition::EGO_RIGHT));
}

bool LanePositionManager::CheckStablePosFlag(
    const std::vector<LaneLinePtr>* tracked_lanelines) {
  bool stable_pos_flag = false;
  for (int i = 0; i < tracked_lanelines->size(); ++i) {
    LaneLinePtr lane_target = (*tracked_lanelines)[i];
    HLOG_DEBUG << "Position filter output:" << int(lane_target->te_position);
    auto iter_pos = lane_pos_map.find(lane_target->id);
    if (iter_pos != lane_pos_map.end()) {
      const auto& last_pos = iter_pos->second;
      if (last_pos == LaneLinePosition::OTHER &&
          CheckEgoPose(lane_target->te_position)) {
        // 过路口场景
        stable_pos_flag = false;
        break;
      } else if (std::abs(static_cast<int>(lane_target->te_position) -
                          static_cast<int>(last_pos)) == 2) {
        // 变道场景
        stable_pos_flag = false;
        break;
      } else if (std::abs(static_cast<int>(lane_target->te_position)) <
                 std::abs(static_cast<int>(last_pos))) {
        if (CheckEgoPose(lane_target->te_position)) {
          // 维持上一帧pos不变
          stable_pos_flag = true;
        }
      } else {
        continue;
      }
    }
  }
  return stable_pos_flag;
}
void LanePositionManager::RevisePose(
    const std::vector<LaneLinePtr>* tracked_lanelines, bool stable_flag,
    bool stable_pos_flag) {
  std::set<int> pos_set;
  pos_set.clear();
  int valid_flag = 0;
  for (int i = 0; i < tracked_lanelines->size(); ++i) {
    LaneLinePtr lane_target = (*tracked_lanelines)[i];
    auto iter_pos = lane_pos_map.find(lane_target->id);
    if (iter_pos != lane_pos_map.end()) {
      if (stable_flag && stable_pos_flag) {
        lane_target->te_position = iter_pos->second;
      } else {
        iter_pos->second = lane_target->te_position;
      }
    } else {
      lane_pos_map.emplace(
          std::make_pair(lane_target->id, lane_target->te_position));
    }
  }
  if (stable_flag && stable_pos_flag) {
    pos_stable_count_++;
  } else {
    pos_stable_count_ = 0;
  }

  for (int i = 0; i < tracked_lanelines->size(); ++i) {
    LaneLinePtr lane_target = (*tracked_lanelines)[i];
    HLOG_DEBUG << "Position filter output result:"
               << int(lane_target->te_position);
    if (abs(static_cast<int>(lane_target->te_position)) < 4) {
      pos_set.insert(static_cast<int>(lane_target->te_position));
      valid_flag++;
    }
  }
  if (pos_set.size() != valid_flag) {
    for (int i = 0; i < tracked_lanelines->size(); ++i) {
      LaneLinePtr lane_target = (*tracked_lanelines)[i];
      auto iter_pos = lane_result_pos_map.find(lane_target->id);
      if (iter_pos != lane_result_pos_map.end()) {
        lane_target->te_position = iter_pos->second;
      }
    }
  }
}
bool LanePositionManager::Process(const LaneLinesPtr& lanelinesptr) {
  std::vector<LaneLinePtr> tracked_lanelines;
  tracked_lanelines.clear();
  for (const auto& laneline : lanelinesptr->lanelines) {
    if (laneline->send_postlane) {
      tracked_lanelines.push_back(laneline);
    }
  }
  // const auto& lane_tracker_pipeline_param =
  //     lane_post_process_param.lane_tracker_pipeline_param();
  float ref_min = 0;
  // lane_tracker_pipeline_param.lane_pose_setter_param().ref_min();
  float ref_length = 5;
  // lane_tracker_pipeline_param.lane_pose_setter_param().ref_length();
  int sample_num = 5;
  // lane_tracker_pipeline_param.lane_pose_setter_param().sample_point_num();

  std::vector<bool> far_lanes_flag = CheckLanesDistance(&tracked_lanelines);
  bool disappear_flag =
      IsMainRoadAboutDisappear(&tracked_lanelines, far_lanes_flag);
  // 解决既有近处又有远处车道线但是都在一端赋值错误的case
  revise_lanes_flag(&tracked_lanelines, &far_lanes_flag);

  if (disappear_flag) {
    // 如果主车道线较短，即将消失，则针对远处车道线进行位置赋值,
    // 近端赋值为OTHER.
    SetLanePosition(ref_min, ref_length, sample_num, tracked_lanelines,
                    far_lanes_flag, &lane_d_map, &lane_result_pos_map, true);
  } else {
    // 如果主车道线长度一般，则针对近处车道线进行位置赋值, 远端赋值为OTHER.
    SetLanePosition(ref_min, ref_length, sample_num, tracked_lanelines,
                    far_lanes_flag, &lane_d_map, &lane_result_pos_map, false);
  }
  // 判断是否稳定，稳定时再判断当pos改变时是否要修改pos
  bool stable_flag = CheckStableFlag(&tracked_lanelines, disappear_flag);
  bool stable_pos_flag = CheckStablePosFlag(&tracked_lanelines);
  RevisePose(&tracked_lanelines, stable_flag, stable_pos_flag);
  return true;
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
