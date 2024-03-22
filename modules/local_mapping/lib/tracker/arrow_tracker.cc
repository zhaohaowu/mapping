// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Li guo (liguo03@hozon.com)
// @file: simple_lane_tracker.cc
// @brief: container for filter

#include "modules/local_mapping/lib/tracker/arrow_tracker.h"

#include "perception-base/base/utils/log.h"
namespace hozon {
namespace mp {
namespace lm {

bool ArrowTracker::Init(const ProcessOption& init_options,
                        ArrowTargetPtr arrow_target) {
  if (arrow_target == nullptr) {
    return false;
  }
  arrow_target_ = arrow_target;

  // Init arrow_point filter

  point_filter_.reset(new ArrowPointFilter(arrow_target));
  point_filter_->Init();
  // Init arrow_type filter
  type_filter_.reset(new ArrowTypeFilter(arrow_target));
  type_filter_->Init();

  return true;
}

void ArrowTracker::UpdateWithDetectedObject(const ProcessOption& options,
                                            const ArrowPtr& detected_arrow) {
  // update lane_target
  arrow_target_->UpdateWithDetectedObject(options, detected_arrow);
  // ArrowFilterOptions arrow_filter_options;
  // lane_filter_options.timestamp = detected_laneline->latest_tracked_time;
  // arrow_filter_options.novatel2world_pose = options.novatel2world_pose;
  point_filter_->UpdateWithMeasurement(detected_arrow);
  type_filter_->UpdateWithMeasurement(detected_arrow);
}

void ArrowTracker::UpdateWithoutDetectedObject(const ProcessOption& options) {
  // ArrowFilterOptions arrow_filter_options;
  // arrow_filter_options.novatel2world_pose = options.novatel2world_pose;
  arrow_target_->UpdateWithoutDetectedObject(options);
  point_filter_->UpdateWithoutMeasurement();
  type_filter_->UpdateWithoutMeasurement();
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
