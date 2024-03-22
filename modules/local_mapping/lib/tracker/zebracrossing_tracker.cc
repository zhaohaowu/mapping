// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Li guo (liguo03@hozon.com)
// @file: simple_lane_tracker.cc
// @brief: container for filter

#include "modules/local_mapping/lib/tracker/zebracrossing_tracker.h"

#include "perception-base/base/utils/log.h"
namespace hozon {
namespace mp {
namespace lm {

bool ZebraCrossingTracker::Init(const ProcessOption& init_options,
                                ZebraCrossingTargetPtr zebracrossing_target) {
  if (zebracrossing_target == nullptr) {
    return false;
  }
  zebracrossing_target_ = zebracrossing_target;
  // Init zebracrossing_point filter
  point_filter_.reset(new ZebraCrossingPointFilter(zebracrossing_target));
  point_filter_->Init();

  return true;
}

void ZebraCrossingTracker::UpdateWithDetectedObject(
    const ProcessOption& options,
    const ZebraCrossingPtr& detected_zebracrossing) {
  // update lane_target
  zebracrossing_target_->UpdateWithDetectedObject(options,
                                                  detected_zebracrossing);
  point_filter_->UpdateWithMeasurement(detected_zebracrossing);
}

void ZebraCrossingTracker::UpdateWithoutDetectedObject(
    const ProcessOption& options) {
  zebracrossing_target_->UpdateWithoutDetectedObject(options);
  point_filter_->UpdateWithoutMeasurement();
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
