// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Li guo (liguo03@hozon.com)
// @file: simple_lane_tracker.h
// @brief: container for filter

#pragma once
#include <memory>

#include "modules/local_mapping/lib/laneline/proto/lane_postprocess.pb.h"
#include "modules/local_mapping/lib/laneline/tracker/bev_lane_tracker/base/roadedge_target.h"
#include "modules/local_mapping/lib/laneline/tracker/bev_lane_tracker/data_fusion/laneline_type_filter.h"
#include "modules/local_mapping/lib/laneline/tracker/bev_lane_tracker/data_fusion/roadedge_point_filter.h"
namespace hozon {
namespace mp {
namespace environment {

struct SimpleRoadEdgeTrackerInitOptions {
  Eigen::Affine3d novatel2world_pose = Eigen::Affine3d::Identity();
  LaneTrackFilterParam lane_track_filter_param;
};

struct SimpleRoadEdgeTrackerOptions {
  Eigen::Affine3d novatel2world_pose = Eigen::Affine3d::Identity();
};

class SimpleRoadEdgeTracker {
 public:
  SimpleRoadEdgeTracker();
  virtual ~SimpleRoadEdgeTracker();
  SimpleRoadEdgeTracker(const SimpleRoadEdgeTracker&) = delete;
  SimpleRoadEdgeTracker& operator=(const SimpleRoadEdgeTracker&) = delete;

  bool Init(const SimpleRoadEdgeTrackerInitOptions& init_options,
            RoadEdgeTargetPtr lane_target);

  void UpdateWithDetectedLaneLine(
      const SimpleRoadEdgeTrackerOptions& options,
      const perception_base::RoadEdgeMeasurementPtr& detected_laneline);

  void UpdateWithoutDetectedLaneLine(
      const SimpleRoadEdgeTrackerOptions& options);

  inline const RoadEdgeTargetConstPtr GetConstLaneTarget() const {
    return lane_target_;
  }

  inline RoadEdgeTargetPtr GetLaneTarget() { return lane_target_; }

 private:
  std::unique_ptr<RoadEdgePointFilter> point_filter_ = nullptr;
  std::unique_ptr<RoadEdgeTypeFilter> type_filter_ = nullptr;

  RoadEdgeTargetPtr lane_target_ = nullptr;
  LaneTrackFilterParam lane_track_filter_param_;
};

typedef std::shared_ptr<SimpleRoadEdgeTracker> SimpleRoadEdgeTrackerPtr;
typedef std::shared_ptr<const SimpleRoadEdgeTracker>
    SimpleRoadEdgeTrackerConstPtr;

}  // namespace environment
}  // namespace mp
}  // namespace hozon
