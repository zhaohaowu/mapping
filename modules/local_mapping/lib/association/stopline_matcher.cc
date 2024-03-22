// Copyright 2023 Hozon Inc. All Rights Reserved.
// @author: hozon
// @file: lane_matcher.cc
// @brief: matcher for lane
#include "modules/local_mapping/lib/association/stopline_matcher.h"

#include <algorithm>
#include <vector>

#include "perception-base/base/utils/log.h"

namespace hozon {
namespace mp {
namespace lm {

bool StopLineMatcher::Init(const MatcherInitOptions& options) {
  // lane_match_param_ = options.lane_match_param;
  target_used_mask_.reserve(50);
  det_used_mask_.reserve(50);
  return true;
}

void StopLineMatcher::Clear() {}

bool StopLineMatcher::Associate(
    const MatcherOptions& options,
    const std::vector<StopLinePtr>& detect_measurements,
    const std::vector<StopLineTrackerPtr>& stopline_trackers,
    AssociationResult* association_result) {
  // 清理前一时刻的关联结果
  Clear();

  // 用于表示被占用的变量状态重置
  track_stoplines_size_ = stopline_trackers.size();
  detect_stoplines_size_ = detect_measurements.size();
  target_used_mask_.resize(track_stoplines_size_);
  target_used_mask_.assign(track_stoplines_size_, false);
  det_used_mask_.resize(detect_stoplines_size_);
  det_used_mask_.assign(detect_stoplines_size_, false);

  for (size_t i = 0; i < detect_stoplines_size_; i++) {
    double min_x_dis = FLT_MAX;
    size_t track_stop_line_index = 0;
    for (size_t j = 0; j < track_stoplines_size_; j++) {
      if (target_used_mask_[j]) {
        continue;
      }

      double center_point_x_distance = fabs(
          detect_measurements[i]->center_point.x() - stopline_trackers[j]
                                                      ->GetConstTarget()
                                                      ->GetConstTrackedObject()
                                                      ->center_point.x());

      if (center_point_x_distance < min_x_dis) {
        min_x_dis = center_point_x_distance;
        track_stop_line_index = j;
      }
    }
    if (min_x_dis < stopline_match_dis_thresh_) {
      // 配对上的数据
      // 0: track_index, 1: detect_index
      std::get<0>(match_score_tuple_) = track_stop_line_index;
      std::get<1>(match_score_tuple_) = i;
      std::get<2>(match_score_tuple_) = min_x_dis;
      association_result->assignments.push_back(match_score_tuple_);
      target_used_mask_[track_stop_line_index] = true;
      det_used_mask_[i] = true;
    } else {
      // 没有被匹配上的观测数据
      association_result->unsigned_objects.push_back(i);
    }
  }

  // 没有被匹配上的跟踪数据
  for (size_t m = 0; m < track_stoplines_size_; m++) {
    if (!target_used_mask_[m]) {
      association_result->unassigned_tracks.push_back(m);
    }
  }
  return true;
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
