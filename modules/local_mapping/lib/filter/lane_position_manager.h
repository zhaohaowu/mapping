// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Hozon (hozon@hozon.com)
// @file: lane_position_manager.h
// @brief: lane_position for lane

#pragma once
#include <limits>
#include <set>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/local_mapping/base/scene/localmap.h"
#include "modules/local_mapping/lib/target/base_target.h"
#include "modules/local_mapping/lib/tracker/laneline_tracker.h"
namespace hozon {
namespace mp {
namespace lm {

class LanePositionManager {
 public:
  void Init();

  void Process(const LaneLinesPtr& laneline_ptrs);
  void SetLaneLinePosition(const std::vector<LaneLinePtr>& lane_lines);
  void SetC0(const LaneLinesPtr& laneline_ptrs);
  void SetReferC0(const LaneLinesPtr& laneline_ptrs);
  void IfCross(const LaneLinesPtr& laneline_ptrs);
  void SelectLaneLines(const std::vector<LaneLinePtr>& normal_lanelines,
                       std::vector<LaneLinePtr>* selected_lanelines);
  bool IsUnknownLaneline(const LaneLinePtr& laneline_ptr);

 private:
  bool inited_ = false;
  float dist_separation_point_ = 5.0;

  // pos 维持不变阈值设置
  int pos_change_count_threshold_ = 5;
  int pos_stable_count_ = 0;
  float d_change_threshold_ = 0.4;
  // <track_id, <lane_pose, count(lane_pose != current_pose)>>
  std::unordered_map<int, LaneLinePosition> lane_pos_map;
  std::unordered_map<int, LaneLinePosition> lane_result_pos_map;
  // <track_id, <last_d, d_error>> 横向位置变化
  std::unordered_map<int, std::tuple<float, float>> lane_d_map;
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
