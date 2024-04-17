// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: LiGuo (liguo03@hozon.com)
// @file: lane_gatekeeper.cc
// @brief: gatekeeper for lane tracker

#include "modules/local_mapping/lib/gatekeeper/roadedge_gatekeeper.h"

#include <algorithm>

#include "modules/local_mapping/utils/lane_utils.h"
#include "modules/util/include/util/mapping_log.h"
#include "perception-lib/lib/config_manager/config_manager.h"

namespace hozon {
namespace mp {
namespace lm {

namespace perception_lib = hozon::perception::lib;

void RoadEdgeGatekeeper::Init() {
  // todo 配置赋值
  const auto& config_manager = perception_lib::ConfigManager::Instance();
  const perception_lib::ModelConfig* model_config = nullptr;
  if (!config_manager->GetModelConfig("LocalMapApp", &model_config)) {
    HLOG_ERROR << "Parse config failed! Name: LaneGatekeepper";
  }

  model_config->get_value("use_debug", &use_debug_mode_);
  inited_ = true;
}
bool RoadEdgeGatekeeper::AbleToOutput(
    const RoadEdgeTargetConstPtr& target) const {
  const auto& laneline = target->GetConstTrackedObject();
  if (target->IsTracked()) {
    return true;
  }
  // debug模式输出不成熟的元素
  // if (use_debug_mode_ && !target->IsTracked()) {
  //   return true;
  // }
  // donot output lost curbs
  if (target->IsLost()) {
    return false;
  }

  return false;
}

bool RoadEdgeGatekeeper::AbleToOutput(
    const RoadEdgeTargetConstPtr& target,
    const std::vector<RoadEdgeTargetConstPtr>& targets) const {
  if (!inited_) {
    return false;
  }
  if (!AbleToOutput(target)) {
    HLOG_DEBUG << "NOT AbleToOutput return false ";
    return false;
  }

  const auto& point_set1 = target->GetConstTrackedObject()->vehicle_points;

  // nms
  for (int i = 0; i < targets.size(); ++i) {
    if (target == targets[i]) {
      continue;
    }
    if (AbleToOutput(targets[i])) {
      const auto& point_set2 =
          targets[i]->GetConstTrackedObject()->vehicle_points;
      float distance = GetDistBetweenTwoLane(point_set1, point_set2);
      bool is_lateral_overlap = distance < output_nms_distance_;
      float len1 = GetLength(point_set1);
      float len2 = GetLength(point_set2);
      // length small enough or short track
      bool is_short = false;
      if (len1 < len2) {
        if (len1 / (len2 + 1e-9) < output_nms_length_ratio_) {
          is_short = true;
        } else {
          // short life_time track
          is_short = target->Count() <= targets[i]->Count();
        }
      } else {
        // short life_time track
        is_short = target->Count() < targets[i]->Count();
      }
      if (is_lateral_overlap && is_short) {
        HLOG_DEBUG << " RoadEdge LaneTarget " << target->Id()
                   << " is big_overlap with LaneTarget " << targets[i]->Id()
                   << ", len1:" << len1 << ", len2:" << len2
                   << ", distance: " << distance;
        return false;
      }
    }
  }

  return true;
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
