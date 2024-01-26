// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Li guo (liguo03@hozon.com)
// @file: simple_lane_tracker.cc
// @brief: container for filter

#include "modules/laneline_postprocess/lib/laneline/tracker/bev_lane_tracker/data_fusion/simple_lane_tracker.h"

#include "perception-base/base/utils/log.h"
namespace hozon {
namespace mp {
namespace environment {

using base::LaneLine;
using base::LaneLineConstPtr;

SimpleLaneTracker::SimpleLaneTracker() {}
SimpleLaneTracker::~SimpleLaneTracker() {}

bool SimpleLaneTracker::Init(const SimpleLaneTrackerInitOptions& init_options,
                             LaneTargetPtr lane_target) {
  if (lane_target == nullptr) {
    return false;
  }
  lane_track_filter_param_ = init_options.lane_track_filter_param;
  lane_target_ = lane_target;

  // Init lane_point filter
  LanePointFilterInitOptions lane_point_filter_init_options;
  lane_point_filter_.reset(new LanePointFilter(lane_target));
  lane_point_filter_init_options.lane_point_filter_param =
      lane_track_filter_param_.lane_point_filter_param();
  lane_point_filter_->Init(lane_point_filter_init_options);

  // Init lane_type filter
  LaneTypeFilterInitOptions lane_type_filter_init_options;
  lane_type_filter_.reset(new LaneTypeFilter(lane_target));
  lane_type_filter_init_options.lane_type_filter_param =
      lane_track_filter_param_.lane_type_filter_param();
  lane_type_filter_->Init(lane_type_filter_init_options);

  // Init lane_color filter
  LaneColorFilterInitOptions lane_color_filter_init_options;
  lane_color_filter_.reset(new LaneColorFilter(lane_target));
  lane_color_filter_init_options.lane_color_filter_param =
      lane_track_filter_param_.lane_color_filter_param();
  lane_color_filter_->Init(lane_color_filter_init_options);

  return true;
}

void SimpleLaneTracker::UpdateWithDetectedLaneLine(
    const SimpleLaneTrackerOptions& options,
    const base::LaneLineMeasurementPtr& detected_laneline) {
  // update lane_target
  lane_target_->UpdateWithDetectedLaneLine(detected_laneline);
  LaneFilterOptions lane_filter_options;
  // lane_filter_options.timestamp = detected_laneline->latest_tracked_time;
  lane_filter_options.novatel2world_pose = options.novatel2world_pose;
  lane_point_filter_->UpdateWithMeasurement(lane_filter_options,
                                            detected_laneline);
  lane_type_filter_->UpdateWithMeasurement(lane_filter_options,
                                           detected_laneline);
  lane_color_filter_->UpdateWithMeasurement(lane_filter_options,
                                            detected_laneline);

  return;
}

void SimpleLaneTracker::UpdateWithoutDetectedLaneLine(
    const SimpleLaneTrackerOptions& options) {
  LaneFilterOptions lane_filter_options;
  lane_filter_options.novatel2world_pose = options.novatel2world_pose;
  lane_target_->UpdateWithoutDetectedLaneLine();
  lane_point_filter_->UpdateWithoutMeasurement(lane_filter_options);
  lane_type_filter_->UpdateWithoutMeasurement(lane_filter_options);
  lane_color_filter_->UpdateWithoutMeasurement(lane_filter_options);
  return;
}

}  // namespace environment
}  // namespace mp
}  // namespace hozon
