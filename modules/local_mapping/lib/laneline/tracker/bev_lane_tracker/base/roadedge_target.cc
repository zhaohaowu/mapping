// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: LiGuo (liguo03@hozon.com)
// @file: lane_target.cc
// @brief: define the basic struct of lane target
#include "modules/local_mapping/lib/laneline/tracker/bev_lane_tracker/base/roadedge_target.h"

#include <algorithm>
#include <limits>

namespace hozon {
namespace mp {
namespace environment {

std::atomic<int> RoadEdgeTarget::s_global_track_id_ = {0};

RoadEdgeTarget::RoadEdgeTarget() {}

RoadEdgeTarget::~RoadEdgeTarget() {}

bool RoadEdgeTarget::Init(
    const LaneTargetInitOption& options,
    const perception_base::RoadEdgeMeasurementPtr& detected_lane_line) {
  lane_target_param_ = options.lane_target_param;

  tracked_laneline_ = std::make_shared<perception_base::RoadEdge>();
  // TODO(陈安猛)
  tracked_laneline_->point_set = detected_lane_line->point_set;

  id_ = RoadEdgeTarget::s_global_track_id_++;
  if (RoadEdgeTarget::s_global_track_id_ >= std::numeric_limits<int>::max()) {
    RoadEdgeTarget::s_global_track_id_ = {0};
    id_ = RoadEdgeTarget::s_global_track_id_;
  }

  lost_age_ = 0;
  tracked_count_++;
  UpdateTrackStatus(false);

  tracked_laneline_->id = id_;
  return true;
}

void RoadEdgeTarget::Reset() {
  tracked_laneline_ = nullptr;
  id_ = 0;
  lost_age_ = 0;
  start_tracked_timestamp_ = 0.0;
  tracked_count_ = 0;
}

void RoadEdgeTarget::UpdateWithDetectedLaneLine(
    const perception_base::RoadEdgeMeasurementPtr& detected_lane_line) {
  lost_age_ = 0;
  tracked_count_++;

  UpdateTrackStatus(false, tracked_laneline_);
  tracking_time_ = lastest_tracked_timestamp_ - start_tracked_timestamp_;

  return;
}

void RoadEdgeTarget::UpdateWithoutDetectedLaneLine() {
  ++lost_age_;
  UpdateTrackStatus(true, tracked_laneline_);
  return;
}

}  // namespace environment
}  // namespace mp
}  // namespace hozon
