// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Li guo (liguo03@hozon.com)
// @file: simple_lane_tracker_pipeline.h
// @brief: associate history lane_track to current detected lane　object

#pragma once

#include <algorithm>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "Eigen/Core"
#include "modules/local_mapping/lib/laneline/interface/base_tracker_pipeline.h"
#include "modules/local_mapping/lib/laneline/proto/lane_postprocess.pb.h"
#include "modules/local_mapping/lib/laneline/tracker/bev_lane_tracker/association/lane_matcher.h"
#include "modules/local_mapping/lib/laneline/tracker/bev_lane_tracker/association/point_matcher.h"
#include "modules/local_mapping/lib/laneline/tracker/bev_lane_tracker/data_fusion/simple_lane_tracker.h"
#include "modules/local_mapping/lib/laneline/tracker/bev_lane_tracker/gatekeeper/point_lane_gatekeeper.h"
#include "modules/local_mapping/lib/laneline/utils/lane_utils.h"

namespace hozon {
namespace mp {
namespace environment {

namespace perception_base = hozon::perception::base;
namespace perception_lib = hozon::perception::lib;

class LanePointFilterTrackerPipeline : public BaseTrackerPipeline {
 public:
  LanePointFilterTrackerPipeline() : BaseTrackerPipeline() {}
  virtual ~LanePointFilterTrackerPipeline() {}

  bool Init(const ProcessInitOption& options = ProcessInitOption()) override;
  bool Track(const ProcessOption& options,
             perception_base::LaneLinesMeasurementConstPtr detect_measurements,
             const perception_base::LaneLinesPtr& track_lanelines);

  std::string Name() const override;

 private:
  void TransMeasurementVehicle2Local(
      std::vector<perception_base::LaneLineMeasurementPtr>*
          detect_measurements);
  void TransTrackerLocal2Vehicle(
      std::vector<perception_base::LaneLinePtr>* tracked_lanelines);

  void UpdateAssignedTracks(
      std::vector<perception_base::LaneLineMeasurementPtr>* detected_lanelines,
      const PointAssociationResult& association_result);

  void UpdateUnassignedTracks(
      const std::vector<perception_base::LaneLineMeasurementPtr>*
          detected_lanelines,
      const PointAssociationResult& association_result);

  void CreateNewTracks(
      const std::vector<perception_base::LaneLineMeasurementPtr>*
          detected_lanelines,
      const PointAssociationResult& association_result);

  void PostProcess(
      std::vector<perception_base::LaneLinePtr>* tracked_lanelines);

  void CollectOutputObjects(
      std::vector<perception_base::LaneLinePtr>* tracked_lanelines);

  void RemoveLostTracks();

  void SetPoseAttribute(
      std::vector<perception_base::LaneLinePtr>* tracked_lanelines);

  void SetMainRoadQuality(
      std::vector<perception_base::LaneLinePtr>* tracked_lanelines) {}

  void GetMainRoadLaneLine(perception_base::LaneLinePtr left_laneline,
                           perception_base::LaneLinePtr right_laneline);

  std::vector<bool> CheckLanesDistance(
      const std::vector<perception_base::LaneLinePtr>* tracked_lanelines);

  bool IsMainRoadAboutDisappear(
      const std::vector<perception_base::LaneLinePtr>* tracked_lanelines,
      const std::vector<bool>& near_lanelines);
  void revise_lanes_flag(
      const std::vector<perception_base::LaneLinePtr>* tracked_lanelines,
      std::vector<bool>&
          far_lanes_flag);  // NOLINT revise far_lanes_flag in function

  bool CheckStableFlag(
      const std::vector<perception_base::LaneLinePtr>* tracked_lanelines,
      bool disappear_flag);
  bool CheckStablePosFlag(
      const std::vector<perception_base::LaneLinePtr>* tracked_lanelines);
  void RevisePose(
      const std::vector<perception_base::LaneLinePtr>* tracked_lanelines,
      bool stable_flag, bool stable_pos_flag);
  bool CheckEgoPose(perception_base::LaneLinePosition pose);
  void print_debug_info();

  std::vector<LaneTargetConstPtr> GetAllLaneTarget();

 private:
  LanePostProcessParam lane_post_process_param_;

  LaneTargetInitOption target_init_option_;
  SimpleLaneTrackerInitOptions tracker_init_option_;

  std::vector<SimpleLaneTrackerPtr> lane_trackers_;
  std::vector<LaneTargetConstPtr> lane_targets_;
  std::unique_ptr<PointMatcher> point_matcher_ = nullptr;
  std::unique_ptr<PointLaneGatekeeper> point_lane_gate_keeper_ = nullptr;

  bool is_last_main_road_left_exist_ = false;
  bool is_last_main_road_right_exist_ = false;
  perception_base::LaneLinePtr main_road_left_line_ = nullptr;
  perception_base::LaneLinePtr main_road_right_line_ = nullptr;

  float dist_separation_point_ = 5.0;

  // pos 维持不变阈值设置
  int pos_change_count_threshold_ = 5;
  int pos_stable_count_ = 0;
  float d_change_threshold_ = 0.4;
  // <track_id, <lane_pose, count(lane_pose != current_pose)>>
  std::unordered_map<int, perception_base::LaneLinePosition> lane_pos_map;
  std::unordered_map<int, perception_base::LaneLinePosition>
      lane_result_pos_map;
  // <track_id, <last_d, d_error>> 横向位置变化
  std::unordered_map<int, std::tuple<float, float>> lane_d_map;
};
PERCEPTION_ENVIRONMENT_REGISTER_BASE_TRACKER_PIPELINE(
    LanePointFilterTrackerPipeline);

}  // namespace environment
}  // namespace mp
}  // namespace hozon
