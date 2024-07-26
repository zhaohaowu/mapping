// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Li guo (liguo03@hozon.com)
// @file: occedge_tracker.h
// @brief: container for filter

#pragma once
#include <memory>

#include "modules/local_mapping/lib/filter/occedge_point_filter.h"
#include "modules/local_mapping/lib/interface/base_init_options.h"
#include "modules/local_mapping/lib/target/base_target.h"

namespace hozon {
namespace mp {
namespace lm {

class OccEdgeTracker {
 public:
  bool Init(const ProcessOption& init_options, OccEdgeTargetPtr occedge_target);

  void UpdateWithDetectedObject(const ProcessOption& options,
                                const OccEdgePtr& detected_occedge);

  void UpdateWithoutDetectedObject(const ProcessOption& options);

  inline OccEdgeTargetConstPtr GetConstTarget() const {
    return occedge_target_;
  }

  inline OccEdgeTargetPtr GetTarget() { return occedge_target_; }

 private:
  std::unique_ptr<OccEdgePointFilter> point_filter_ = nullptr;

  OccEdgeTargetPtr occedge_target_ = nullptr;
};

using OccEdgeTrackerPtr = std::shared_ptr<OccEdgeTracker>;
using OccEdgeTrackerConstPtr = std::shared_ptr<const OccEdgeTracker>;

}  // namespace lm
}  // namespace mp
}  // namespace hozon
