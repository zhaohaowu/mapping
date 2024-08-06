// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Hozon (hozon@hozon.com)
// @file: mapping_position_manager.h
// @brief: mapping_position for local map

#pragma once
#include <float.h>

#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "modules/local_mapping/base/scene/laneline.h"
#include "modules/local_mapping/base/scene/localmap.h"
#include "modules/local_mapping/lib/target/base_target.h"
#include "modules/local_mapping/lib/tracker/laneline_tracker.h"
namespace hozon {
namespace mp {
namespace lm {

class MappingPositionManager {
 public:
  bool Init();
  void Process(const LaneLinesPtr& laneline_ptrs);
  void SetJunction(const LaneLinesPtr& laneline_ptrs);
  void SetC0(const LaneLinesPtr& laneline_ptrs);
  void SetReferC0(const LaneLinesPtr& laneline_ptrs);
  void IfCross(const LaneLinesPtr& laneline_ptrs);
  void DivideLaneLines(const std::vector<LaneLinePtr>& normal_lanelines,
                       std::vector<LaneLinePtr>* forward_lanelines,
                       std::vector<LaneLinePtr>* behind_lanelines);
  void SelectLaneLines(const std::vector<LaneLinePtr>& normal_lanelines,
                       std::vector<LaneLinePtr>* selected_lanelines);
  void GetMinDisPoint(const LaneLinesPtr& laneline_ptrs);
  bool IsUnknownLaneline(const LaneLinePtr& laneline_ptrs,
                         bool has_main_line_flag);
  void SetLaneLinePosition(const std::vector<LaneLinePtr>& lane_lines);

 private:
  bool inited_ = false;
  std::unordered_map<int, std::tuple<float, float>>* lane_d_map;
  std::vector<LaneLinePtr> lane_lines_ego_;
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
