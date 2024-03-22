// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Li guo (liguo03@hozon.com)
// @file: simple_lane_tracker.cc
// @brief: container for filter

#include "modules/local_mapping/lib/tracker/laneline_tracker.h"

#include "modules/local_mapping/lib/interface/base_init_options.h"
#include "modules/local_mapping/lib/target/base_target.h"
#include "perception-base/base/utils/log.h"

namespace hozon {
namespace mp {
namespace lm {

bool LaneTracker::Init(const ProcessOption& init_options,
                       LaneTargetPtr lane_target) {
  if (lane_target == nullptr) {
    return false;
  }
  lane_target_ = lane_target;
  // Init lane_point filter
  FilterInitOption lane_point_filter_init_options;
  lane_point_filter_ = std::make_unique<LaneLinePointFilter>(lane_target);
  lane_point_filter_->Init(lane_point_filter_init_options);
  // Init lane_type filter
  FilterInitOption lane_type_filter_init_options;
  lane_type_filter_ = std::make_unique<LaneLineTypeFilter>(lane_target);
  lane_type_filter_->Init(lane_type_filter_init_options);
  // Init lane_color filter
  FilterInitOption lane_color_filter_init_options;
  lane_color_filter_ = std::make_unique<LaneLineColorFilter>(lane_target);
  lane_color_filter_->Init(lane_color_filter_init_options);

  return true;
}

void LaneTracker::UpdateWithDetectedObject(
    const ProcessOption& options, const LaneLinePtr& detected_laneline) {
  // update lane_target
  lane_target_->UpdateWithDetectedObject(options, detected_laneline);
  FilterOption lane_filter_options;
  lane_point_filter_->UpdateWithMeasurement(lane_filter_options,
                                            detected_laneline);
  lane_type_filter_->UpdateWithMeasurement(lane_filter_options,
                                           detected_laneline);
  lane_color_filter_->UpdateWithMeasurement(lane_filter_options,
                                            detected_laneline);
}

void LaneTracker::UpdateWithoutDetectedObject(const ProcessOption& options) {
  lane_target_->UpdateWithoutDetectedObject(options);
  FilterOption lane_filter_options;
  lane_point_filter_->UpdateWithoutMeasurement(lane_filter_options);
  lane_type_filter_->UpdateWithoutMeasurement(lane_filter_options);
  lane_color_filter_->UpdateWithoutMeasurement(lane_filter_options);
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
