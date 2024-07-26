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
#include "modules/local_mapping/base/scene/localmap.h"
#include "modules/local_mapping/base/scene/occedge.h"
#include "modules/local_mapping/lib/association/occedge_matcher.h"
#include "modules/local_mapping/lib/gatekeeper/occedge_gatekeeper.h"
#include "modules/local_mapping/lib/interface/base_mapping_pipeline.h"
#include "modules/local_mapping/utils/lane_utils.h"

namespace hozon {
namespace mp {
namespace lm {
class OccEdgeMappingPipeline : public BaseMappingPipeline {
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
                            const std::vector<OccEdgePtr>& detect_measurements,
                            const AssociationResult& association_result);
  void UpdateUnassignedTracks(
      const ProcessOption& tracker_option,
      const std::vector<OccEdgePtr>& detect_measurements,
      const AssociationResult& association_result);
  void CreateNewTracks(const ProcessOption& tracker_option,
                       const std::vector<OccEdgePtr>& detect_measurements,
                       const AssociationResult& association_result);
  void CreateNewTracks(const ProcessOption& tracker_option,
                       const std::vector<OccEdgePtr>& detect_measurements);
  void CollectOutputObjects(OccEdgesPtr tracked_occedges);
  void CatmullRomFit(OccEdgesPtr tracked_occedges);
  void InterpolatePoint(std::vector<OccEdgePtr> detect_measurements);
  void RemoveLostTracks();
  void MergeTracks(std::vector<OccEdgeTrackerPtr>* trackers);
  void LimitTracksNum();
  static bool CompareTrackTime(const OccEdgeTrackerPtr& d1,
                               const OccEdgeTrackerPtr& d2);
  std::vector<OccEdgePtr> CleanMeasureData(
      const std::vector<OccEdgePtr>* measurement_data);
  std::vector<OccEdgePtr> CurveFitMeasureData(
      const std::vector<OccEdgePtr>* measurement_data);
  std::vector<OccEdgeTargetConstPtr> GetAllConstTarget();

  std::vector<OccEdgeTargetPtr> GetAllTarget();

  bool CheckBadTrack(const OccEdgeTrackerPtr& occedge_track);

 private:
  std::unique_ptr<OccEdgeMatcher> occedge_matcher_ = nullptr;
  std::unique_ptr<OccEdgeGatekeeper> occedge_gate_keeper_ = nullptr;
  std::vector<OccEdgeTrackerPtr> occedge_trackers_;
  std::vector<OccEdgeTargetConstPtr> occedge_targets_;
  int limit_max_tracker_nums_ = 50;
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
