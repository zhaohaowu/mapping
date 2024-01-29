// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Li guo (liguo03@hozon.com)
// @file: simple_lane_tracker.h
// @brief: container for filter

#pragma once
#include <memory>

#include "modules/laneline_postprocess/lib/laneline/proto/lane_postprocess.pb.h"
#include "modules/laneline_postprocess/lib/laneline/tracker/bev_lane_tracker/base/lane_target.h"
#include "modules/laneline_postprocess/lib/laneline/tracker/bev_lane_tracker/data_fusion/laneline_color_filter.h"
#include "modules/laneline_postprocess/lib/laneline/tracker/bev_lane_tracker/data_fusion/laneline_point_filter.h"
#include "modules/laneline_postprocess/lib/laneline/tracker/bev_lane_tracker/data_fusion/laneline_type_filter.h"
namespace hozon {
namespace mp {
namespace environment {

namespace perception_base = hozon::perception::base;

struct SimpleLaneTrackerInitOptions {
  Eigen::Affine3d novatel2world_pose = Eigen::Affine3d::Identity();
  LaneTrackFilterParam lane_track_filter_param;
};

struct SimpleLaneTrackerOptions {
  Eigen::Affine3d novatel2world_pose = Eigen::Affine3d::Identity();
};

class SimpleLaneTracker {
 public:
  SimpleLaneTracker();
  virtual ~SimpleLaneTracker();
  SimpleLaneTracker(const SimpleLaneTracker&) = delete;
  SimpleLaneTracker& operator=(const SimpleLaneTracker&) = delete;

  bool Init(const SimpleLaneTrackerInitOptions& init_options,
            LaneTargetPtr lane_target);

  void UpdateWithDetectedLaneLine(
      const SimpleLaneTrackerOptions& options,
      const perception_base::LaneLineMeasurementPtr& detected_laneline);

  void UpdateWithoutDetectedLaneLine(const SimpleLaneTrackerOptions& options);

  inline const LaneTargetConstPtr GetConstLaneTarget() const {
    return lane_target_;
  }

  inline LaneTargetPtr GetLaneTarget() { return lane_target_; }

 private:
  std::unique_ptr<LanePointFilter> lane_point_filter_ = nullptr;
  std::unique_ptr<LaneTypeFilter> lane_type_filter_ = nullptr;
  std::unique_ptr<LaneColorFilter> lane_color_filter_ = nullptr;

  LaneTargetPtr lane_target_ = nullptr;
  LaneTrackFilterParam lane_track_filter_param_;
};

typedef std::shared_ptr<SimpleLaneTracker> SimpleLaneTrackerPtr;
typedef std::shared_ptr<const SimpleLaneTracker> SimpleLaneTrackerConstPtr;

}  // namespace environment
}  // namespace mp
}  // namespace hozon
