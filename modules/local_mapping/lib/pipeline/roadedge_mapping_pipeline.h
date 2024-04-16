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
#include "Eigen/src/Core/util/Constants.h"
#include "Eigen/src/Geometry/Transform.h"
#include "modules/local_mapping/base/scene/localmap.h"
#include "modules/local_mapping/lib/association/roadedge_matcher.h"
#include "modules/local_mapping/lib/filter/post_roadedge_position_manager.h"
#include "modules/local_mapping/lib/filter/roadedge_position_manager.h"
#include "modules/local_mapping/lib/gatekeeper/roadedge_gatekeeper.h"
#include "modules/local_mapping/lib/interface/base_mapping_pipeline.h"
#include "modules/local_mapping/utils/lane_utils.h"

namespace hozon {
namespace mp {
namespace lm {
class RoadEdgeMappingPipeline : public BaseMappingPipeline {
 public:
  bool Init() override;
  bool Process(const ProcessOption& option,
               MeasurementFrameConstPtr measurement_ptr,
               LocalMapFramePtr localmap_frame_ptr) override;
  std::string Name() const override;

 private:
  // 对观测数据和跟踪数据统一到统一坐标系。
  void UpdateTracks();
  void UpdateAssignedTracks(const ProcessOption& tracker_option,
                            const std::vector<RoadEdgePtr>& detect_measurements,
                            const AssociationResult& association_result);
  void UpdateUnassignedTracks(
      const ProcessOption& tracker_option,
      const std::vector<RoadEdgePtr>& detect_measurements,
      const AssociationResult& association_result);
  void CreateNewTracks(const ProcessOption& tracker_option,
                       const std::vector<RoadEdgePtr>& detect_measurements,
                       const AssociationResult& association_result);
  void CollectOutputObjects(RoadEdgesPtr tracked_roadedges);
  void CatmullRomFit(RoadEdgesPtr tracked_roadedges);
  void RemoveLostTracks();
  void MergeTracks(std::vector<RoadEdgeTrackerPtr>* trackers);
  void LimitTracksNum();
  static bool CompareTrackTime(const RoadEdgeTrackerPtr& d1,
                               const RoadEdgeTrackerPtr& d2);
  void PostProcess(RoadEdgesPtr tracked_roadedges);
  std::vector<RoadEdgePtr> CleanMeasureData(
      const std::vector<RoadEdgePtr>* measurement_data);

  std::vector<RoadEdgeTargetConstPtr> GetAllConstTarget();

  std::vector<RoadEdgeTargetPtr> GetAllTarget();

  bool CheckBadTrack(const RoadEdgeTrackerPtr& roadedge_track);

  bool roadedge_nanvalue_trigger();

 private:
  std::unique_ptr<RoadEdgeMatcher> roadedge_matcher_ = nullptr;
  std::unique_ptr<RoadEdgeGatekeeper> roadedge_gate_keeper_ = nullptr;
  std::vector<RoadEdgeTrackerPtr> roadedge_trackers_;
  std::vector<RoadEdgeTargetConstPtr> roadedge_targets_;
  std::unique_ptr<RoadEdgePositionManager> roadedge_position_manager_ =
      nullptr;  // roadedge position管理器
  std::unique_ptr<PostRoadEdgePositionManager> post_roadedge_position_manager_ =
      nullptr;
  int limit_max_tracker_nums_ = 50;
  //   ProcessInitOption tracker_init_option_;
  //   ProcessInitOption target_init_option_;
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
