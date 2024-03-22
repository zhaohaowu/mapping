// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Hozon (hozon@hozon.com)
// @file: lane_position_manager.cc
// @brief: lane pos for lane tracker

#include "modules/local_mapping/lib/filter/post_roadedge_position_manager.h"

#include <algorithm>

#include "modules/local_mapping/utils/lane_utils.h"

namespace hozon {
namespace mp {
namespace lm {

void PostRoadEdgePositionManager::Init() { inited_ = true; }

bool PostRoadEdgePositionManager::Process(const RoadEdgesPtr& roadedges_ptr) {
  std::vector<RoadEdgePtr> tracked_roadedges;
  tracked_roadedges.clear();
  for (const auto& roadedge : roadedges_ptr->road_edges) {
    if (roadedge->send_postlane) {
      tracked_roadedges.push_back(roadedge);
    }
  }

  float ref_min = 0;
  float ref_length = 5;
  int sample_num = 5;

  SetLanePosition(ref_min, ref_length, sample_num, tracked_roadedges);

  return true;
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
