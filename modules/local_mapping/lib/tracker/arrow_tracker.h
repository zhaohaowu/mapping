// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Li guo (liguo03@hozon.com)
// @file: simple_lane_tracker.h
// @brief: container for filter

#pragma once
#include <memory>

#include "modules/local_mapping/lib/filter/arrow_point_filter.h"
#include "modules/local_mapping/lib/filter/arrow_type_filter.h"
#include "modules/local_mapping/lib/interface/base_init_options.h"
#include "modules/local_mapping/lib/target/base_target.h"
namespace hozon {
namespace mp {
namespace lm {

class ArrowTracker {
 public:
  bool Init(const ProcessOption& init_options, ArrowTargetPtr lane_target);

  void UpdateWithDetectedObject(const ProcessOption& options,
                                const ArrowPtr& detected_laneline);

  void UpdateWithoutDetectedObject(const ProcessOption& options);

  inline ArrowTargetConstPtr GetConstTarget() const { return arrow_target_; }

  inline ArrowTargetPtr GetTarget() { return arrow_target_; }

 private:
  std::unique_ptr<ArrowPointFilter> point_filter_ = nullptr;
  std::unique_ptr<ArrowTypeFilter> type_filter_ = nullptr;

  ArrowTargetPtr arrow_target_ = nullptr;
};

using ArrowTrackerPtr = std::shared_ptr<ArrowTracker>;
using ArrowTrackerConstPtr = std::shared_ptr<const ArrowTracker>;

}  // namespace lm
}  // namespace mp
}  // namespace hozon
