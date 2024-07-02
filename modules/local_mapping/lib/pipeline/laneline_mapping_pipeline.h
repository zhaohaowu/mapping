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
#include "modules/local_mapping/lib/association/laneline_matcher.h"
#include "modules/local_mapping/lib/filter/lane_position_manager.h"
#include "modules/local_mapping/lib/filter/laneline_measure_filter.h"
#include "modules/local_mapping/lib/filter/laneline_merge_tracker.h"
#include "modules/local_mapping/lib/filter/mapping_position_manager.h"
#include "modules/local_mapping/lib/filter/mapping_remove_manager.h"
#include "modules/local_mapping/lib/gatekeeper/laneline_gatekeeper.h"
#include "modules/local_mapping/lib/interface/base_mapping_pipeline.h"
#include "modules/local_mapping/utils/lane_utils.h"
namespace hozon {
namespace mp {
namespace lm {

class LaneLineMappingPipeline : public BaseMappingPipeline {
 public:
  bool Init() override;

  bool Process(const ProcessOption& option,
               MeasurementFrameConstPtr measurement_ptr,
               LocalMapFramePtr fusion_ptr) override;
  std::string Name() const override;

 private:
  // 对观测数据和跟踪数据统一到统一坐标系。
  void UpdateTracks();
  void UpdateAssignedTracks(const ProcessOption& tracker_option,
                            const std::vector<LaneLinePtr>& detect_measurements,
                            const AssociationResult& association_result);
  void UpdateUnassignedTracks(
      const ProcessOption& tracker_option,
      const std::vector<LaneLinePtr>& detect_measurements,
      const AssociationResult& association_result);
  void CreateNewTracks(const ProcessOption& tracker_option,
                       const std::vector<LaneLinePtr>& detect_measurements,
                       const AssociationResult& association_result);
  void CollectOutputObjects(LaneLinesPtr tracked_lanelines);
  void CatmullRomFit(LaneLinesPtr tracked_lanelines);
  void AdjustIntersectionLines(LaneLinesPtr tracked_lanelines);
  bool DeleteLaneLineShortPart(size_t laneline_delete_index,
                               std::vector<Eigen::Vector3d>* vehicle_points);
  void RemoveLostTracks();
  void SetIntersectionThreshold();
  void AssginPosition(LaneLinesPtr localmap_frame_ptr);
  void SetHistoryLaneLinePos(LaneLinesPtr tracked_lanelines);
  void DeleteOutlierLaneLines(std::vector<LaneTrackerPtr>* trackers);
  void LimitTracksNum();
  void SmoothEndPt();
  static bool CompareTrackTime(const LaneTrackerPtr& d1,
                               const LaneTrackerPtr& d2);
  std::vector<LaneTargetConstPtr> GetAllConstTarget();

  std::vector<LaneTargetPtr> GetAllTarget();

  bool CheckBadTrack(const LaneTrackerPtr& laneline_track);

  bool laneline_nanvalue_trigger();
  void CheckTriggerLanelineNan();

 private:
  std::unique_ptr<LaneLineMatcher> laneline_matcher_ = nullptr;  // 车道线匹配器
  std::unique_ptr<LaneLineMergeTrack> laneline_merge_tracker_ = nullptr;
  std::vector<LaneTrackerPtr> lane_trackers_;     // 车道线跟踪器
  std::vector<LaneTargetConstPtr> lane_targets_;  // 车道线数据管理器
  std::unique_ptr<LaneGatekeeper> lane_gate_keeper_ = nullptr;
  std::unique_ptr<LaneMeasurementFilter> lane_meas_filter_ =
      nullptr;  // 车道线观测过滤模块
  std::unique_ptr<MappingRemoveManager> mapping_remove_manager_ = nullptr;
  //   ProcessInitOption tracker_init_option_;
  //   ProcessInitOption target_init_option_;
  // 用于判断路口前后
  std::unique_ptr<LanePositionManager> lane_position_manager_ =
      nullptr;  // 车道线position管理器
  std::unique_ptr<MappingPositionManager> map_lane_position_manager_ =
      nullptr;  // local map position管理器

  int limit_max_tracker_nums_ = 100;
  double max_delet_dis = 20.0;
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
