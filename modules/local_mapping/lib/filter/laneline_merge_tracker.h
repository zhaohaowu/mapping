// Copyright 2020 Hozon Inc. All Rights Reserved.
// @file: laneline_merge_tracker.h
// @brief: merge tracker
#pragma once

#include <unordered_set>
#include <vector>

#include "modules/local_mapping/lib/tracker/laneline_tracker.h"

namespace hozon {
namespace mp {
namespace lm {

class LaneLineMergeTrack {
 public:
  void MergeTracks(std::vector<LaneTrackerPtr>* trackers);

 private:
  void MergeTrackPoints(const LaneTargetConstPtr& left_target,
                        const LaneTargetConstPtr& right_target);
  bool MergeOverlayStrategy(const LaneTargetConstPtr& left_target,
                            const LaneTargetConstPtr& right_target);
  bool MergeOverlayCrossStrategy(const LaneTargetConstPtr& left_line,
                                 const LaneTargetConstPtr& right_line);
  // 需要删除的tracker标记
  std::unordered_set<int> remove_index_;
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
