// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Li guo (liguo03@hozon.com)
// @file: simple_lane_tracker.h
// @brief: container for filter

#pragma once
#include <memory>

#include "modules/local_mapping/lib/filter/zebracrossing_point_filter.h"
#include "modules/local_mapping/lib/interface/base_init_options.h"
#include "modules/local_mapping/lib/target/base_target.h"
namespace hozon {
namespace mp {
namespace lm {

class ZebraCrossingTracker {
 public:
  bool Init(const ProcessOption& init_options,
            ZebraCrossingTargetPtr zebracrossing_target);

  void UpdateWithDetectedObject(const ProcessOption& options,
                                const ZebraCrossingPtr& detected_zebracrossing);

  void UpdateWithoutDetectedObject(const ProcessOption& options);

  inline ZebraCrossingTargetConstPtr GetConstTarget() const {
    return zebracrossing_target_;
  }

  inline ZebraCrossingTargetPtr GetTarget() { return zebracrossing_target_; }

 private:
  std::unique_ptr<ZebraCrossingPointFilter> point_filter_ = nullptr;

  ZebraCrossingTargetPtr zebracrossing_target_ = nullptr;
};

using ZebraCrossingTrackerPtr = std::shared_ptr<ZebraCrossingTracker>;
using ZebraCrossingTrackerConstPtr =
    std::shared_ptr<const ZebraCrossingTracker>;

}  // namespace lm
}  // namespace mp
}  // namespace hozon
