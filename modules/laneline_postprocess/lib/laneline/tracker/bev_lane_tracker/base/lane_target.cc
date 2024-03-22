// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: LiGuo (liguo03@hozon.com)
// @file: lane_target.cc
// @brief: define the basic struct of lane target
#include "modules/laneline_postprocess/lib/laneline/tracker/bev_lane_tracker/base/lane_target.h"

#include <algorithm>
#include <limits>

namespace hozon {
namespace mp {
namespace environment {

std::atomic<int> LaneTarget::s_global_track_id_ = {0};

LaneTarget::LaneTarget() {}

LaneTarget::~LaneTarget() {}

bool LaneTarget::Init(const LaneTargetInitOption& options,
                      const LaneLineMeasurementPtr& detected_lane_line) {
  lane_target_param_ = options.lane_target_param;

  tracked_laneline_ = std::make_shared<perception_base::LaneLine>();
  // TODO(陈安猛)
  tracked_laneline_->point_set = detected_lane_line->point_set;

  id_ = LaneTarget::s_global_track_id_++;
  if (LaneTarget::s_global_track_id_ >= std::numeric_limits<int>::max()) {
    LaneTarget::s_global_track_id_ = {0};
    id_ = LaneTarget::s_global_track_id_;
  }

  lost_age_ = 0;
  tracked_count_++;
  UpdateTrackStatus(false);

  // TODO(张文海) start track time
  // latest_tracked_time = ts;

  tracked_laneline_->id = id_;
  return true;
}

void LaneTarget::Reset() {
  tracked_laneline_ = nullptr;
  id_ = 0;
  lost_age_ = 0;
  start_tracked_timestamp_ = 0.0;
  tracked_count_ = 0;
}

void LaneTarget::UpdateWithDetectedLaneLine(
    const LaneLineMeasurementPtr& detected_lane_line) {
  lost_age_ = 0;
  tracked_count_++;

  UpdateTrackStatus(false, tracked_laneline_);
  tracking_time_ = lastest_tracked_timestamp_ - start_tracked_timestamp_;

  return;
}

void LaneTarget::UpdateWithoutDetectedLaneLine() {
  ++lost_age_;
  UpdateTrackStatus(true, tracked_laneline_);
  return;
}

}  // namespace environment
}  // namespace mp
}  // namespace hozon
