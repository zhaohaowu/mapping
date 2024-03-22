// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: LiGuo (liguo03@hozon.com)
// @file: lane_gatekeeper.cc
// @brief: gatekeeper for lane tracker

#include "modules/local_mapping/lib/gatekeeper/laneline_gatekeeper.h"

#include <algorithm>

#include "modules/local_mapping/utils/lane_utils.h"
#include "modules/util/include/util/mapping_log.h"
#include "perception-lib/lib/config_manager/config_manager.h"

namespace hozon {
namespace mp {
namespace lm {

namespace perception_lib = hozon::perception::lib;

void LaneGatekeeper::Init() {
  // todo 配置赋值
  const auto& config_manager = perception_lib::ConfigManager::Instance();
  const perception_lib::ModelConfig* model_config = nullptr;
  if (!config_manager->GetModelConfig("LocalMapApp", &model_config)) {
    HLOG_ERROR << "Parse config failed! Name: LaneGatekeepper";
  }

  model_config->get_value("use_debug", &use_debug_mode_);
  inited_ = true;
}

bool LaneGatekeeper::AbleToOutput(const LaneTargetConstPtr& target) const {
  const auto& laneline = target->GetConstTrackedObject();
  // 1. donot output short laneline which is not in the blind area
  float len = GetLength(laneline->vehicle_points);
  bool is_far_short =
      len < output_lane_length_ &&
      laneline->vehicle_points.front().x() > output_blind_start_;
  bool is_fork_or_converge = IsForkOrConverge(laneline->scene_type);
  if (!is_fork_or_converge && is_far_short) {
    HLOG_DEBUG << "LaneTarget " << target->Id() << " is far short,len:" << len;
    return false;
  }

  if (target->IsTracked()) {
    return true;
  }
  // debug模式输出不成熟的元素
  if (use_debug_mode_ && !target->IsTracked()) {
    return true;
  }

  // donot output lost curbs
  if (target->IsLost()) {
    return false;
  }

  return false;
}

bool LaneGatekeeper::AbleToOutput(
    const LaneTargetConstPtr& target,
    const std::vector<LaneTargetConstPtr>& targets) const {
  if (!inited_) {
    return false;
  }
  if (!AbleToOutput(target)) {
    HLOG_INFO << "NOT AbleToOutput return false ";
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

      bool is_not_intersection = !IsForkOrConvergePair(
          target->GetConstTrackedObject()->scene_type,
          targets[i]->GetConstTrackedObject()->scene_type);
      float overlap_start =
          std::max(point_set1.front().x(), point_set2.front().x());
      float overlap_end =
          std::min(point_set1.back().x(), point_set2.back().x());
      float iou = (overlap_end < overlap_start)
                      ? 0
                      : (overlap_end - overlap_start) / (len1 + 1e-9);

      bool is_longitudinal_overlap =
          iou > output_lane_longitudinal_overlap_iou_;
      if (is_lateral_overlap && is_short && is_not_intersection &&
          is_longitudinal_overlap) {
        HLOG_DEBUG << "MinningTimestamp: "
                   << std::to_string(target->GetLastestTrackedTimestamp())
                   << " LaneTarget " << target->Id()
                   << " is big_overlap with LaneTarget " << targets[i]->Id();

        HLOG_DEBUG << "LaneTarget " << target->Id()
                   << " is big_overlap with LaneTarget " << targets[i]->Id()
                   << ", len1:" << len1 << ", len2:" << len2
                   << ", is_not_intersection:" << is_not_intersection
                   << ", longitudinal_iou:" << iou;
        return false;
      }
    }
  }

  return true;
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
