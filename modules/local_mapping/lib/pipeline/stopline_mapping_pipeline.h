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
#include "base/scene/stopline.h"
#include "modules/local_mapping/base/scene/localmap.h"
#include "modules/local_mapping/base/scene/stopline.h"
#include "modules/local_mapping/lib/association/stopline_matcher.h"
#include "modules/local_mapping/lib/gatekeeper/stopline_gatekeeper.h"
#include "modules/local_mapping/lib/interface/base_mapping_pipeline.h"
#include "modules/local_mapping/lib/tracker/stopline_tracker.h"
namespace hozon {
namespace mp {
namespace lm {

class StopLineMappingPipeline : public BaseMappingPipeline {
 public:
  bool Init() override;

  bool Process(const ProcessOption& option,
               MeasurementFrameConstPtr measurement_ptr,
               LocalMapFramePtr fusion_ptr) override;

  std::string Name() const override;

 private:
  // 对观测数据和跟踪数据统一到统一坐标系。
  void UpdateTracks();

  std::vector<StopLinePtr> CleanMeasureData(
      const std::vector<StopLinePtr>* measurement_data);

  void UpdateAssignedTracks(const ProcessOption& option,
                            const std::vector<StopLinePtr>& detected_stoplines,
                            const AssociationResult& association_result);

  void UpdateUnassignedTracks(
      const ProcessOption& option,
      const std::vector<StopLinePtr>& detected_stoplines,
      const AssociationResult& association_result);

  void CreateNewTracks(const ProcessOption& option,
                       const std::vector<StopLinePtr>& detected_stoplines,
                       const AssociationResult& association_result);

  void MergeTracks();

  void CollectOutputObjects(std::vector<StopLinePtr>* tracked_stoplines);

  void RemoveLostTracks();

  void LimitTracksNum();
  static bool CompareTrackTime(const StopLineTrackerPtr& d1,
                               const StopLineTrackerPtr& d2);

  std::vector<StopLineTargetConstPtr> GetAllConstTarget();

  std::vector<StopLineTargetPtr> GetAllTarget();

 private:
  std::vector<StopLineTrackerPtr> stopline_trackers_;  // 停止线跟踪管理器
  std::vector<StopLineTargetConstPtr> stopline_targets_;  // 停止线跟踪数据
  std::unique_ptr<StopLineMatcher> stopline_matcher_ = nullptr;  // 停止线匹配器
  std::unique_ptr<StopLineGatekeeper> stopline_gate_keeper_ =
      nullptr;  // 停止线准出器

  int limit_max_tracker_nums_ = 50;
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
