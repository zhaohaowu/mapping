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

class PostRoadEdgePositionManager {
 public:
  void Init();

  bool Process(const RoadEdgesPtr& roadedges_ptr);

 private:
  bool inited_ = false;
  float dist_separation_point_ = 5.0;

  // pos 维持不变阈值设置
  int pos_change_count_threshold_ = 5;
  int pos_stable_count_ = 0;
  float d_change_threshold_ = 0.4;
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
