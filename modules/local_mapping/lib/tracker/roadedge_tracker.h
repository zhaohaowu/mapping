// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Li guo (liguo03@hozon.com)
// @file: simple_lane_tracker.h
// @brief: container for filter

#pragma once
#include <memory>

#include "modules/local_mapping/lib/filter/roadedge_point_filter.h"
#include "modules/local_mapping/lib/filter/roadedge_type_filter.h"
#include "modules/local_mapping/lib/interface/base_init_options.h"
#include "modules/local_mapping/lib/target/base_target.h"

namespace hozon {
namespace mp {
namespace lm {

class RoadEdgeTracker {
 public:
  // RoadEdgeTracker();
  // virtual ~RoadEdgeTracker();
  // RoadEdgeTracker(const RoadEdgeTracker&) = delete;
  // RoadEdgeTracker& operator=(const RoadEdgeTracker&) = delete;

  bool Init(const ProcessOption& init_options, RoadEdgeTargetPtr lane_target);

  void UpdateWithDetectedObject(const ProcessOption& options,
                                const RoadEdgePtr& detected_laneline);

  void UpdateWithoutDetectedObject(const ProcessOption& options);

  inline RoadEdgeTargetConstPtr GetConstTarget() const {
    return roadedge_target_;
  }

  inline RoadEdgeTargetPtr GetTarget() { return roadedge_target_; }

 private:
  std::unique_ptr<RoadEdgePointFilter> point_filter_ = nullptr;
  std::unique_ptr<RoadEdgeTypeFilter> type_filter_ = nullptr;

  RoadEdgeTargetPtr roadedge_target_ = nullptr;
};

using RoadEdgeTrackerPtr = std::shared_ptr<RoadEdgeTracker>;
using RoadEdgeTrackerConstPtr = std::shared_ptr<const RoadEdgeTracker>;

}  // namespace lm
}  // namespace mp
}  // namespace hozon
