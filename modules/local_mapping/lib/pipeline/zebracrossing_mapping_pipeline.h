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
#include "modules/local_mapping/base/scene/localmap.h"
#include "modules/local_mapping/base/scene/zebracrossing.h"
#include "modules/local_mapping/lib/association/base_struct.h"
#include "modules/local_mapping/lib/association/zebracrossing_matcher.h"
#include "modules/local_mapping/lib/gatekeeper/zebracrossing_gatekeeper.h"
#include "modules/local_mapping/lib/interface/base_mapping_pipeline.h"
#include "modules/local_mapping/lib/target/base_target.h"
#include "modules/local_mapping/lib/tracker/zebracrossing_tracker.h"
namespace hozon {
namespace mp {
namespace lm {

class ZebraCrossingMappingPipeline : public BaseMappingPipeline {
 public:
  bool Init() override;

  bool Process(const ProcessOption& option,
               MeasurementFrameConstPtr measurement_ptr,
               LocalMapFramePtr fusion_ptr) override;

  std::string Name() const override;

 private:
  std::vector<ZebraCrossingPtr> CleanMeasureData(
      const std::vector<ZebraCrossingPtr>* measurement_data);

  // 对观测数据和跟踪数据统一到统一坐标系。
  void UpdateTracks();

  void CollectOutputObjects(
      std::vector<ZebraCrossingPtr>* tracked_zebracrossings);

  void RemoveLostTracks();

  void UpdateAssignedTracks(
      const ProcessOption& option,
      const std::vector<ZebraCrossingPtr>& detected_stoplines,
      const AssociationResult& association_result);

  void UpdateUnassignedTracks(
      const ProcessOption& option,
      const std::vector<ZebraCrossingPtr>& detected_stoplines,
      const AssociationResult& association_result);

  void CreateNewTracks(const ProcessOption& option,
                       const std::vector<ZebraCrossingPtr>& detected_stoplines,
                       const AssociationResult& association_result);

  void MergeTracks();

  void LimitTracksNum();
  static bool CompareTrackTime(const ZebraCrossingTrackerPtr& d1,
                               const ZebraCrossingTrackerPtr& d2);

  std::vector<ZebraCrossingTargetConstPtr> GetAllConstTarget();

  std::vector<ZebraCrossingTargetPtr> GetAllTarget();

 private:
  std::vector<ZebraCrossingTrackerPtr>
      zebracrossing_trackers_;  // 停止线跟踪管理器
  std::vector<ZebraCrossingTargetConstPtr>
      zebracrossing_targets_;  // 停止线跟踪数据
  std::unique_ptr<ZebraCrossingMatcher> zebracrossing_matcher_ =
      nullptr;  // 停止线匹配器
  std::unique_ptr<ZebraCrossingGatekeeper> zebracrossing_gate_keeper_ =
      nullptr;  // 停止线准出器

  int limit_max_tracker_nums_ = 50;
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
