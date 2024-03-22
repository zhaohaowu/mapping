// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Li guo (liguo03@hozon.com)
// @file: simple_lane_tracker.h
// @brief: container for filter
#pragma once
#include <memory>

#include "modules/local_mapping/lib/filter/laneline_color_filter.h"
#include "modules/local_mapping/lib/filter/laneline_point_filter.h"
#include "modules/local_mapping/lib/filter/laneline_type_filter.h"
#include "modules/local_mapping/lib/interface/base_init_options.h"
#include "modules/local_mapping/lib/target/base_target.h"
namespace hozon {
namespace mp {
namespace lm {

class LaneTracker {
 public:
  bool Init(const ProcessOption& init_options, LaneTargetPtr lane_target);

  void UpdateWithDetectedObject(const ProcessOption& options,
                                const LaneLinePtr& detected_laneline);

  void UpdateWithoutDetectedObject(const ProcessOption& options);

  inline LaneTargetConstPtr GetConstTarget() const { return lane_target_; }

  inline LaneTargetPtr GetTarget() { return lane_target_; }

 private:
  std::unique_ptr<LaneLinePointFilter> lane_point_filter_ = nullptr;
  std::unique_ptr<LaneLineTypeFilter> lane_type_filter_ = nullptr;
  std::unique_ptr<LaneLineColorFilter> lane_color_filter_ = nullptr;

  LaneTargetPtr lane_target_ = nullptr;
};

using LaneTrackerPtr = std::shared_ptr<LaneTracker>;
using LaneTrackerConstPtr = std::shared_ptr<const LaneTracker>;

}  // namespace lm
}  // namespace mp
}  // namespace hozon
