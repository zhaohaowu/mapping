// Copyright 2023 Hozon Inc. All Rights Reserved.
// @author: hozon
// @file: lane_matcher.cc
// @brief: matcher for lane
#include "modules/local_mapping/lib/association/arrow_matcher.h"

#include <algorithm>
#include <vector>

#include "perception-base/base/utils/log.h"

namespace hozon {
namespace mp {
namespace lm {

bool ArrowMatcher::Init(const MatcherInitOptions& options) {
  // lane_match_param_ = options.lane_match_param;
  target_used_mask_.reserve(50);
  det_used_mask_.reserve(50);
  return true;
}

void ArrowMatcher::Clear() { match_score_list_.clear(); }

bool ArrowMatcher::Associate(
    const MatcherOptions& options,
    const std::vector<ArrowPtr>& detect_measurements,
    const std::vector<ArrowTrackerPtr>& arrows_trackers,
    AssociationResult* association_result) {
  Clear();

  track_arrows_size_ = arrows_trackers.size();
  detect_arrows_size_ = detect_measurements.size();
  target_used_mask_.resize(track_arrows_size_);
  target_used_mask_.assign(track_arrows_size_, false);
  det_used_mask_.resize(detect_arrows_size_);
  det_used_mask_.assign(detect_arrows_size_, false);

  for (size_t i = 0; i < detect_arrows_size_; i++) {
    double min_x_dis = FLT_MAX;
    size_t track_arrow_index = 0;
    for (size_t j = 0; j < track_arrows_size_; j++) {
      if (target_used_mask_[j]) {
        continue;
      }
      if (fabs(detect_measurements[i]->center_point.x() -
               arrows_trackers[j]
                   ->GetConstTarget()
                   ->GetConstTrackedObject()
                   ->center_point.x()) < min_x_dis &&
          fabs(detect_measurements[i]->center_point.y() -
               arrows_trackers[j]
                   ->GetConstTarget()
                   ->GetConstTrackedObject()
                   ->center_point.y()) < arrow_match_y_dis_thresh_) {
        min_x_dis = fabs(detect_measurements[i]->center_point.x() -
                         arrows_trackers[j]
                             ->GetConstTarget()
                             ->GetConstTrackedObject()
                             ->center_point.x());
        track_arrow_index = j;
      }
    }
    if (min_x_dis < arrow_match_x_dis_thresh_) {
      // 0: track_index, 1: detect_index
      std::get<0>(match_score_tuple_) = track_arrow_index;
      std::get<1>(match_score_tuple_) = i;
      association_result->assignments.push_back(match_score_tuple_);
      target_used_mask_[track_arrow_index] = true;
      det_used_mask_[i] = true;
    } else {
      association_result->unsigned_objects.push_back(i);
    }
  }
  for (size_t m = 0; m < track_arrows_size_; m++) {
    if (!target_used_mask_[m]) {
      association_result->unassigned_tracks.push_back(m);
    }
  }
  return true;
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
