// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Li guo (liguo03@hozon.com)
// @file: simple_lane_tracker.h
// @brief: container for filter

#pragma once
#include <memory>

#include "modules/local_mapping/lib/filter/stopline_point_filter.h"
#include "modules/local_mapping/lib/target/base_target.h"
namespace hozon {
namespace mp {
namespace lm {

class StopLineTracker {
 public:
  bool Init(const ProcessOption& init_options,
            const StopLineTargetPtr& lane_target);

  void UpdateWithDetectedObject(const ProcessOption& options,
                                const StopLinePtr& detected_laneline);

  void UpdateWithoutDetectedObject(const ProcessOption& options);

  inline StopLineTargetConstPtr GetConstTarget() const {
    return stopline_target_;
  }

  inline StopLineTargetPtr GetTarget() { return stopline_target_; }

 private:
  std::unique_ptr<StopLinePointFilter> point_filter_ = nullptr;

  StopLineTargetPtr stopline_target_ = nullptr;
};

using StopLineTrackerPtr = std::shared_ptr<StopLineTracker>;
using StopLineTrackerConstPtr = std::shared_ptr<const StopLineTracker>;

}  // namespace lm
}  // namespace mp
}  // namespace hozon
