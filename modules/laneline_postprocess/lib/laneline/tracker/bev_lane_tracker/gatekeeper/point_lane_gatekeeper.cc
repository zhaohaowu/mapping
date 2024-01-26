// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: LiGuo (liguo03@hozon.com)
// @file: lane_gatekeeper.cc
// @brief: gatekeeper for lane tracker

#include "modules/laneline_postprocess/lib/laneline/tracker/bev_lane_tracker/gatekeeper/point_lane_gatekeeper.h"

#include <algorithm>

#include "modules/laneline_postprocess/lib/laneline/utils/lane_utils.h"

namespace hozon {
namespace mp {
namespace environment {

void PointLaneGatekeeper::Init(
    const PointLaneGatekeeperInitOptions &init_options) {
  lane_gate_keeper_param_ = init_options.lane_gate_keeper_param;
  inited_ = true;
}

bool PointLaneGatekeeper::AbleToOutput(const LaneTargetConstPtr &target) const {
  auto &laneline = target->GetConstTrackedLaneLine();
  // 1. donot output short laneline which is not in the blind area
  float len = GetLength(laneline->point_set);
  bool is_far_short = len < lane_gate_keeper_param_.output_lane_length() &&
                      laneline->point_set.front().vehicle_point.x >
                          lane_gate_keeper_param_.output_blind_start();
  bool is_fork_or_converge = IsForkOrConverge(laneline->scene_type);
  if (!is_fork_or_converge && is_far_short) {
    HLOG_DEBUG << "LaneTarget " << target->Id() << " is far short,len:" << len;
    return false;
  }

  if (target->IsTracked()) {
    return true;
  }

  // donot output lost curbs
  if (lane_gate_keeper_param_.output_lost_lanes() && target->IsLost()) {
    return true;
  }

  HLOG_DEBUG << "target->IsLost()" << target->IsLost()
             << ", init: " << target->IsInit();

  return false;
}

bool PointLaneGatekeeper::AbleToOutput(
    const RoadEdgeTargetConstPtr &target) const {
  auto &laneline = target->GetConstTrackedLaneLine();
  // 1. donot output short laneline which is not in the blind area
  if (target->IsTracked()) {
    return true;
  }

  // donot output lost curbs
  if (lane_gate_keeper_param_.output_lost_lanes() && target->IsLost()) {
    return true;
  }

  HLOG_DEBUG << "target->IsLost()" << target->IsLost()
             << ", init: " << target->IsInit();

  return false;
}

bool PointLaneGatekeeper::AbleToOutput(
    const PointLaneGatekeeperOptions &options, const LaneTargetConstPtr &target,
    const std::vector<LaneTargetConstPtr> &targets) const {
  if (!inited_) {
    return false;
  }
  if (!AbleToOutput(target)) {
    HLOG_INFO << "NOT AbleToOutput return false ";
    return false;
  }

  const auto &point_set1 = target->GetConstTrackedLaneLine()->point_set;

  // nms
  for (int i = 0; i < targets.size(); ++i) {
    if (target == targets[i]) {
      continue;
    }
    if (AbleToOutput(targets[i])) {
      const auto &point_set2 = targets[i]->GetConstTrackedLaneLine()->point_set;
      float distance = GetDistBetweenTwoLane(point_set1, point_set2);
      HLOG_DEBUG << "distance: " << distance;
      bool is_lateral_overlap =
          distance < lane_gate_keeper_param_.output_nms_distance();
      float len1 = GetLength(point_set1);
      float len2 = GetLength(point_set2);
      // length small enough or short track
      bool is_short = false;
      if (len1 < len2) {
        if (len1 / (len2 + 1e-9) <
            lane_gate_keeper_param_.output_nms_length_ratio()) {
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
          target->GetConstTrackedLaneLine()->scene_type,
          targets[i]->GetConstTrackedLaneLine()->scene_type);
      float overlap_start = std::max(point_set1.front().vehicle_point.x,
                                     point_set2.front().vehicle_point.x);
      float overlap_end = std::min(point_set1.back().vehicle_point.x,
                                   point_set2.back().vehicle_point.x);
      float iou = (overlap_end < overlap_start)
                      ? 0
                      : (overlap_end - overlap_start) / (len1 + 1e-9);

      bool is_longitudinal_overlap =
          iou > lane_gate_keeper_param_.output_lane_longitudinal_overlap_iou();
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

bool PointLaneGatekeeper::AbleToOutput(
    const PointLaneGatekeeperOptions &options,
    const RoadEdgeTargetConstPtr &target,
    const std::vector<RoadEdgeTargetConstPtr> &targets) const {
  if (!inited_) {
    return false;
  }
  if (!AbleToOutput(target)) {
    HLOG_INFO << "NOT AbleToOutput return false ";
    return false;
  }
  const auto &point_set1 = target->GetConstTrackedLaneLine()->point_set;

  // nms
  for (int i = 0; i < targets.size(); ++i) {
    if (target == targets[i]) {
      continue;
    }
    if (AbleToOutput(targets[i])) {
      const auto &point_set2 = targets[i]->GetConstTrackedLaneLine()->point_set;
      float distance = GetDistBetweenTwoLane(point_set1, point_set2);
      HLOG_DEBUG << "RoadEdge distance: " << distance;
      bool is_lateral_overlap =
          distance < lane_gate_keeper_param_.output_nms_distance();
      float len1 = GetLength(point_set1);
      float len2 = GetLength(point_set2);
      // length small enough or short track
      bool is_short = false;
      if (len1 < len2) {
        if (len1 / (len2 + 1e-9) <
            lane_gate_keeper_param_.output_nms_length_ratio()) {
          is_short = true;

        } else {
          // short life_time track
          is_short = target->Count() <= targets[i]->Count();
        }
      } else {
        // short life_time track
        is_short = target->Count() < targets[i]->Count();
      }

      float overlap_start = std::max(point_set1.front().vehicle_point.x,
                                     point_set2.front().vehicle_point.x);
      float overlap_end = std::min(point_set1.back().vehicle_point.x,
                                   point_set2.back().vehicle_point.x);
      float iou = (overlap_end < overlap_start)
                      ? 0
                      : (overlap_end - overlap_start) / (len1 + 1e-9);

      bool is_longitudinal_overlap =
          iou > lane_gate_keeper_param_.output_lane_longitudinal_overlap_iou();
      if (is_lateral_overlap && is_short && is_longitudinal_overlap) {
        HLOG_DEBUG << "MinningTimestamp: "
                   << std::to_string(target->GetLastestTrackedTimestamp())
                   << " RoadEdge LaneTarget " << target->Id()
                   << " is big_overlap with LaneTarget " << targets[i]->Id();

        HLOG_DEBUG << " RoadEdge LaneTarget " << target->Id()
                   << " is big_overlap with LaneTarget " << targets[i]->Id()
                   << ", len1:" << len1 << ", len2:" << len2
                   << ", longitudinal_iou:" << iou;
        return false;
      }
    }
  }

  return true;
}

}  // namespace environment
}  // namespace mp
}  // namespace hozon
