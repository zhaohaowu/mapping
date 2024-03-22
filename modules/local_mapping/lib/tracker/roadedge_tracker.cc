// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Li guo (liguo03@hozon.com)
// @file: simple_lane_tracker.cc
// @brief: container for filter

#include "modules/local_mapping/lib/tracker/roadedge_tracker.h"

#include "perception-base/base/utils/log.h"
namespace hozon {
namespace mp {
namespace lm {

bool RoadEdgeTracker::Init(const ProcessOption& init_options,
                           RoadEdgeTargetPtr roadedge_target) {
  if (roadedge_target == nullptr) {
    return false;
  }
  roadedge_target_ = roadedge_target;

  // Init roadedge_point filter
  FilterInitOption lane_point_filter_init_options;
  point_filter_ = std::make_unique<RoadEdgePointFilter>(roadedge_target);
  // todo 配置化参数
  // lane_point_filter_init_options.lane_point_filter_param =
  //     lane_track_filter_param_.lane_point_filter_param();
  point_filter_->Init(lane_point_filter_init_options);

  // Init roadedge_type filter
  FilterInitOption lane_type_filter_init_options;
  type_filter_ = std::make_unique<RoadEdgeTypeFilter>(roadedge_target);
  // todo 配置化参数
  // lane_type_filter_init_options.lane_type_filter_param =
  //     lane_track_filter_param_.lane_type_filter_param();
  type_filter_->Init(lane_type_filter_init_options);

  return true;
}

void RoadEdgeTracker::UpdateWithDetectedObject(
    const ProcessOption& options, const RoadEdgePtr& detected_roadedge) {
  // update roadedge_target
  roadedge_target_->UpdateWithDetectedObject(options, detected_roadedge);
  FilterOption lane_filter_options;
  // todo 配置化参数
  // // lane_filter_options.timestamp = detected_laneline->latest_tracked_time;
  // lane_filter_options.novatel2world_pose = options.novatel2world_pose;
  point_filter_->UpdateWithMeasurement(lane_filter_options, detected_roadedge);
  type_filter_->UpdateWithMeasurement(lane_filter_options, detected_roadedge);
}

void RoadEdgeTracker::UpdateWithoutDetectedObject(
    const ProcessOption& options) {
  FilterOption lane_filter_options;
  // lane_filter_options.novatel2world_pose = options.novatel2world_pose;
  roadedge_target_->UpdateWithoutDetectedObject(options);
  point_filter_->UpdateWithoutMeasurement(lane_filter_options);
  type_filter_->UpdateWithoutMeasurement(lane_filter_options);
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
