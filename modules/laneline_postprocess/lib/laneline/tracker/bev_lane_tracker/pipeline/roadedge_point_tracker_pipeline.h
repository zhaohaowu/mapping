// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Li guo (liguo03@hozon.com)
// @file: simple_lane_tracker_pipeline.h
// @brief: associate history lane_track to current detected lane　object

#pragma once

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "Eigen/Core"
#include "modules/laneline_postprocess/lib/laneline/interface/base_tracker_pipeline.h"
#include "modules/laneline_postprocess/lib/laneline/proto/roadedge_postprocess.pb.h"
#include "modules/laneline_postprocess/lib/laneline/tracker/bev_lane_tracker/association/lane_matcher.h"
#include "modules/laneline_postprocess/lib/laneline/tracker/bev_lane_tracker/association/point_matcher.h"
#include "modules/laneline_postprocess/lib/laneline/tracker/bev_lane_tracker/data_fusion/simple_roadedge_tracker.h"
#include "modules/laneline_postprocess/lib/laneline/tracker/bev_lane_tracker/gatekeeper/point_lane_gatekeeper.h"
#include "modules/laneline_postprocess/lib/laneline/utils/lane_utils.h"

namespace hozon {
namespace mp {
namespace environment {

namespace perception_base = hozon::perception::base;
namespace perception_lib = hozon::perception::lib;

class RoadEdgePointFilterTrackerPipeline : public BaseTrackerPipeline {
 public:
  RoadEdgePointFilterTrackerPipeline() : BaseTrackerPipeline() {}
  virtual ~RoadEdgePointFilterTrackerPipeline() {}

  bool Init(const ProcessInitOption& options = ProcessInitOption()) override;
  bool Track(const ProcessOption& options,
             perception_base::RoadEdgesMeasurementConstPtr detect_measurements,
             const perception_base::RoadEdgesPtr& track_outputs);

  std::string Name() const override;

 private:
  void TransMeasurementVehicle2Local(
      std::vector<perception_base::RoadEdgeMeasurementPtr>*
          detect_measurements);
  void TransTrackerLocal2Vehicle(
      std::vector<perception_base::RoadEdgePtr>* tracked_outputs);

  void UpdateAssignedTracks(
      std::vector<perception_base::RoadEdgeMeasurementPtr>* detect_measurements,
      const PointAssociationResult& association_result);

  void UpdateUnassignedTracks(
      const std::vector<perception_base::RoadEdgeMeasurementPtr>*
          detect_measurements,
      const PointAssociationResult& association_result);

  void CreateNewTracks(
      const std::vector<perception_base::RoadEdgeMeasurementPtr>*
          detect_measurements,
      const PointAssociationResult& association_result);

  void PostProcess();

  void CollectOutputObjects(
      std::vector<perception_base::RoadEdgePtr>* tracked_outputs);

  void RemoveLostTracks();

  std::vector<RoadEdgeTargetConstPtr> GetAllLaneTarget();

 private:
  LanePostProcessParam lane_post_process_param_;

  LaneTargetInitOption target_init_option_;
  SimpleRoadEdgeTrackerInitOptions tracker_init_option_;

  std::vector<SimpleRoadEdgeTrackerPtr> lane_trackers_;
  std::vector<RoadEdgeTargetConstPtr> lane_targets_;
  std::unique_ptr<PointMatcher> point_matcher_ = nullptr;
  std::unique_ptr<PointLaneGatekeeper> point_lane_gate_keeper_ = nullptr;
};

PERCEPTION_ENVIRONMENT_REGISTER_BASE_TRACKER_PIPELINE(
    RoadEdgePointFilterTrackerPipeline);

}  // namespace environment
}  // namespace mp
}  // namespace hozon
