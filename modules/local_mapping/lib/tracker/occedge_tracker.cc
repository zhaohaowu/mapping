// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Li guo (liguo03@hozon.com)
// @file: occedge_tracker.cc
// @brief: container for filter

#include "modules/local_mapping/lib/tracker/occedge_tracker.h"

#include "perception-base/base/utils/log.h"
namespace hozon {
namespace mp {
namespace lm {

bool OccEdgeTracker::Init(const ProcessOption& init_options,
                          OccEdgeTargetPtr occedge_target) {
  if (occedge_target == nullptr) {
    return false;
  }
  occedge_target_ = occedge_target;

  // Init roadedge_point filter
  FilterInitOption occedge_filter_init_options;
  point_filter_ = std::make_unique<OccEdgePointFilter>(occedge_target);
  // todo 配置化参数
  point_filter_->Init(occedge_filter_init_options);

  return true;
}

void OccEdgeTracker::UpdateWithDetectedObject(
    const ProcessOption& options, const OccEdgePtr& detected_occedge) {
  // update occedge_target
  occedge_target_->UpdateWithDetectedObject(options, detected_occedge);
  FilterOption occ_filter_options;
  // todo 配置化参数
  point_filter_->UpdateWithMeasurement(occ_filter_options, detected_occedge);
}

void OccEdgeTracker::UpdateWithoutDetectedObject(const ProcessOption& options) {
  FilterOption occ_filter_options;
  // occ_filter_options.novatel2world_pose = options.novatel2world_pose;
  occedge_target_->UpdateWithoutDetectedObject(options);
  point_filter_->UpdateWithoutMeasurement(occ_filter_options);
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
