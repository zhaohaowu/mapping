// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Li guo (liguo03@hozon.com)
// @file: simple_lane_tracker.cc
// @brief: container for filter

#include "modules/local_mapping/lib/tracker/stopline_tracker.h"

#include "modules/local_mapping/lib/filter/stopline_point_filter.h"
#include "perception-base/base/utils/log.h"
namespace hozon {
namespace mp {
namespace lm {

bool StopLineTracker::Init(const ProcessOption& init_options,
                           const StopLineTargetPtr& stopline_target) {
  if (stopline_target == nullptr) {
    return false;
  }
  stopline_target_ = stopline_target;

  // Init stopline_point filter
  point_filter_.reset(new StopLinePointFilter(stopline_target));
  point_filter_->Init();

  return true;
}

void StopLineTracker::UpdateWithDetectedObject(
    const ProcessOption& options, const StopLinePtr& detected_stopline) {
  // update stopline target state
  stopline_target_->UpdateWithDetectedObject(options, detected_stopline);
  // update stopline target point
  HLOG_DEBUG << "start do stopline point filter update with measurement...";
  point_filter_->UpdateWithMeasurement(detected_stopline);
  HLOG_DEBUG << "finish do stopline point filter update with measurement...";
}

void StopLineTracker::UpdateWithoutDetectedObject(
    const ProcessOption& options) {
  // update stopline target state
  stopline_target_->UpdateWithoutDetectedObject(options);
  // update stopline target point
  HLOG_DEBUG << "start do stopline point filter update without measurement...";
  point_filter_->UpdateWithoutMeasurement();
  HLOG_DEBUG << "finish do stopline point filter update without measurement...";
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
