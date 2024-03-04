// Copyright 2020 Hozon Inc. All Rights Reserved.
// @file: base_lane_filter.h
// @brief: interfence for lane filter

#pragma once

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "modules/local_mapping/lib/laneline/tracker/bev_lane_tracker/base/lane_target.h"
#include "modules/local_mapping/lib/laneline/tracker/bev_lane_tracker/base/roadedge_target.h"

namespace hozon {
namespace mp {
namespace environment {

namespace perception_base = hozon::perception::base;

struct LaneFilterOptions {
  int frame_id = 0;
  double timestamp = 0.0;
  bool is_abnormal_pose = false;
  Eigen::Affine3d novatel2world_pose = Eigen::Affine3d::Identity();
};

class BaseLaneFilter {
 public:
  explicit BaseLaneFilter(LaneTargetPtr lane_target) {
    lane_target_ref_ = lane_target;
  }
  virtual ~BaseLaneFilter() {}

  BaseLaneFilter(const BaseLaneFilter&) = delete;
  BaseLaneFilter& operator=(const BaseLaneFilter&) = delete;

  void UpdateWithMeasurement(
      const LaneFilterOptions& filter_options,
      const perception_base::LaneLineConstPtr& measurement) {}

  void UpdateWithoutMeasurement(const LaneFilterOptions& filter_options) {}

 protected:
  LaneTargetPtr lane_target_ref_;
};

class BaseRoadEdgeFilter {
 public:
  explicit BaseRoadEdgeFilter(RoadEdgeTargetPtr lane_target) {
    lane_target_ref_ = lane_target;
  }
  virtual ~BaseRoadEdgeFilter() {}

  BaseRoadEdgeFilter(const BaseRoadEdgeFilter&) = delete;
  BaseRoadEdgeFilter& operator=(const BaseRoadEdgeFilter&) = delete;

  void UpdateWithMeasurement(
      const LaneFilterOptions& filter_options,
      const perception_base::RoadEdgeConstPtr& measurement) {}

  void UpdateWithoutMeasurement(const LaneFilterOptions& filter_options) {}

 protected:
  RoadEdgeTargetPtr lane_target_ref_;
};

}  // namespace environment
}  // namespace mp
}  // namespace hozon
