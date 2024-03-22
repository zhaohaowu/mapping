// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Li guo (liguo03@hozon.com)
// @file: simple_lane_tracker.cc
// @brief: container for filter

#include "modules/laneline_postprocess/lib/laneline/tracker/bev_lane_tracker/data_fusion/simple_roadedge_tracker.h"

#include "perception-base/base/utils/log.h"
namespace hozon {
namespace mp {
namespace environment {

SimpleRoadEdgeTracker::SimpleRoadEdgeTracker() {}
SimpleRoadEdgeTracker::~SimpleRoadEdgeTracker() {}

bool SimpleRoadEdgeTracker::Init(
    const SimpleRoadEdgeTrackerInitOptions& init_options,
    RoadEdgeTargetPtr lane_target) {
  if (lane_target == nullptr) {
    return false;
  }
  lane_track_filter_param_ = init_options.lane_track_filter_param;
  lane_target_ = lane_target;

  // Init roadedge_point filter
  LanePointFilterInitOptions lane_point_filter_init_options;
  point_filter_.reset(new RoadEdgePointFilter(lane_target));
  lane_point_filter_init_options.lane_point_filter_param =
      lane_track_filter_param_.lane_point_filter_param();
  point_filter_->Init(lane_point_filter_init_options);

  // Init roadedge_type filter
  LaneTypeFilterInitOptions lane_type_filter_init_options;
  type_filter_.reset(new RoadEdgeTypeFilter(lane_target));
  lane_type_filter_init_options.lane_type_filter_param =
      lane_track_filter_param_.lane_type_filter_param();
  type_filter_->Init(lane_type_filter_init_options);

  return true;
}

void SimpleRoadEdgeTracker::UpdateWithDetectedLaneLine(
    const SimpleRoadEdgeTrackerOptions& options,
    const perception_base::RoadEdgeMeasurementPtr& detected_laneline) {
  // update lane_target
  lane_target_->UpdateWithDetectedLaneLine(detected_laneline);
  LaneFilterOptions lane_filter_options;
  // lane_filter_options.timestamp = detected_laneline->latest_tracked_time;
  lane_filter_options.novatel2world_pose = options.novatel2world_pose;
  point_filter_->UpdateWithMeasurement(lane_filter_options, detected_laneline);
  type_filter_->UpdateWithMeasurement(lane_filter_options, detected_laneline);
  return;
}

void SimpleRoadEdgeTracker::UpdateWithoutDetectedLaneLine(
    const SimpleRoadEdgeTrackerOptions& options) {
  LaneFilterOptions lane_filter_options;
  lane_filter_options.novatel2world_pose = options.novatel2world_pose;
  lane_target_->UpdateWithoutDetectedLaneLine();
  point_filter_->UpdateWithoutMeasurement(lane_filter_options);
  type_filter_->UpdateWithoutMeasurement(lane_filter_options);
  return;
}

}  // namespace environment
}  // namespace mp
}  // namespace hozon
