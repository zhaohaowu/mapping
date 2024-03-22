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
#include "modules/local_mapping/lib/association/arrow_matcher.h"
#include "modules/local_mapping/lib/association/base_struct.h"
#include "modules/local_mapping/lib/gatekeeper/arrow_gatekeeper.h"
#include "modules/local_mapping/lib/interface/base_mapping_pipeline.h"
#include "modules/local_mapping/lib/tracker/arrow_tracker.h"
namespace hozon {
namespace mp {
namespace lm {

class ArrowMappingPipeline : public BaseMappingPipeline {
 public:
  bool Init() override;

  bool Process(const ProcessOption& option,
               MeasurementFrameConstPtr measurement_ptr,
               LocalMapFramePtr fusion_ptr) override;

  std::string Name() const override;

 private:
  std::vector<ArrowPtr> CleanMeasureData(
      const std::vector<ArrowPtr>* measurement_data);

  // 对观测数据和跟踪数据统一到统一坐标系。
  void UpdateTracks();

  void CollectOutputObjects(std::vector<ArrowPtr>* tracked_arrows);

  void RemoveLostTracks();

  void UpdateAssignedTracks(const ProcessOption& option,
                            const std::vector<ArrowPtr>& detected_arrows,
                            const AssociationResult& association_result);

  void UpdateUnassignedTracks(const ProcessOption& option,
                              const std::vector<ArrowPtr>& detected_arrows,
                              const AssociationResult& association_result);

  void CreateNewTracks(const ProcessOption& option,
                       const std::vector<ArrowPtr>& detected_arrows,
                       const AssociationResult& association_result);

  void MergeTracks();

  void LimitTracksNum();
  static bool CompareTrackTime(const ArrowTrackerPtr& d1,
                               const ArrowTrackerPtr& d2);

  std::vector<ArrowTargetConstPtr> GetAllConstTarget();

  std::vector<ArrowTargetPtr> GetAllTarget();

 private:
  std::vector<ArrowTrackerPtr> arrow_trackers_;     // 箭头跟踪管理器
  std::vector<ArrowTargetConstPtr> arrow_targets_;  // 箭头跟踪数据
  std::unique_ptr<ArrowMatcher> arrow_matcher_ = nullptr;  // 箭头匹配器
  std::unique_ptr<ArrowGatekeeper> arrow_gate_keeper_ = nullptr;  // 箭头准出器
  int limit_max_tracker_nums_ = 50;
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
