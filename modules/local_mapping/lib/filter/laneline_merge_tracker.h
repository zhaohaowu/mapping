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
  bool MergeOverlayStrategy(const LaneTargetPtr& left_target,
                            const LaneTargetPtr& right_target);
  bool MergeOverlayCrossStrategy(const LaneTargetPtr& left_line,
                                 const LaneTargetPtr& right_line);

  // 利用后处理策略判断两根线是否为分合流场景，
  // 以此来优化分合流场景时的merge处理。
  bool IsForkConvergelike(const LaneTargetConstPtr& left_line,
                          const LaneTargetConstPtr& right_line);
  // 选择merge后的车道线类型
  void SetLaneLineType(const LaneTargetPtr& curr_line,
                       const LaneTargetPtr& deleted_line);
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
