// Copyright 2023 Hozon Inc. All Rights Reserved.
// @author: hozon
// @file: lane_matcher.cc
// @brief: matcher for lane
#include "modules/local_mapping/lib/association/zebracrossing_matcher.h"

#include <algorithm>
#include <vector>

#include "perception-base/base/utils/log.h"

namespace hozon {
namespace mp {
namespace lm {

bool ZebraCrossingMatcher::Init(const MatcherInitOptions& options) {
  // lane_match_param_ = options.lane_match_param;
  target_used_mask_.reserve(50);
  det_used_mask_.reserve(50);
  return true;
}

void ZebraCrossingMatcher::Clear() { match_score_list_.clear(); }

bool ZebraCrossingMatcher::Associate(
    const MatcherOptions& options,
    const std::vector<ZebraCrossingPtr>& detect_measurements,
    const std::vector<ZebraCrossingTrackerPtr>& zebracrossing_trackers,
    AssociationResult* association_result) {
  Clear();
  track_zebracrossing_size_ = zebracrossing_trackers.size();
  detect_zebracrossing_size_ = detect_measurements.size();
  target_used_mask_.resize(track_zebracrossing_size_);
  target_used_mask_.assign(track_zebracrossing_size_, false);
  det_used_mask_.resize(detect_zebracrossing_size_);
  det_used_mask_.assign(detect_zebracrossing_size_, false);

  for (size_t i = 0; i < detect_zebracrossing_size_; i++) {
    double min_x_dis = FLT_MAX;
    size_t track_zebracrossing_index = 0;
    for (size_t j = 0; j < track_zebracrossing_size_; j++) {
      if (target_used_mask_[j] ||
          (fabs(detect_measurements[i]->center_point.y() -
                zebracrossing_trackers[j]
                    ->GetConstTarget()
                    ->GetConstTrackedObject()
                    ->center_point.y()) > match_y_dis_thresh_)) {
        continue;
      }
      if (fabs(detect_measurements[i]->center_point.x() -
               zebracrossing_trackers[j]
                   ->GetConstTarget()
                   ->GetConstTrackedObject()
                   ->center_point.x()) < min_x_dis) {
        min_x_dis = fabs(detect_measurements[i]->center_point.x() -
                         zebracrossing_trackers[j]
                             ->GetConstTarget()
                             ->GetConstTrackedObject()
                             ->center_point.x());
        track_zebracrossing_index = j;
      }
    }
    if (min_x_dis < match_x_dis_thresh_) {
      // 0: track_index, 1: detect_index
      std::get<0>(match_score_tuple_) = track_zebracrossing_index;
      std::get<1>(match_score_tuple_) = i;
      association_result->assignments.push_back(match_score_tuple_);
      target_used_mask_[track_zebracrossing_index] = true;
      det_used_mask_[i] = true;
    } else {
      association_result->unsigned_objects.push_back(i);
    }
  }
  for (size_t m = 0; m < track_zebracrossing_size_; m++) {
    if (!target_used_mask_[m]) {
      association_result->unassigned_tracks.push_back(m);
    }
  }
  return true;
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
