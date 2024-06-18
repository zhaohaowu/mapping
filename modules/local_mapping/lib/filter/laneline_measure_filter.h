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
#include "modules/local_mapping/base/scene/laneline.h"
#include "modules/local_mapping/lib/association/base_struct.h"
#include "modules/local_mapping/lib/tracker/laneline_tracker.h"

namespace hozon {
namespace mp {
namespace lm {

class LaneMeasurementFilter {
 public:
  bool Init();
  bool Process(const std::vector<LaneLinePtr>& input_measurements,
               std::vector<LaneLinePtr>* out_measurements);
  bool SetLostTrackerTruncation(std::vector<LaneLinePtr>* measurements,
                                std::vector<LaneTrackerPtr>* lane_trackers);

  bool AssginForkOrConvergeType(std::vector<LaneLinePtr>* out_measurements,
                                std::vector<LaneTrackerPtr>* lane_trackers);

  std::string Name() const { return "LaneMeasurementFilter"; }

 private:
  bool DeleteBehindVehicleLaneLine(std::vector<LaneLinePtr>* measure_lanelines);

  bool DelFarShortLaneLine(std::vector<LaneLinePtr>* measure_lanelines);

  bool DelHighlyOverlayShortLaneLine(
      std::vector<LaneLinePtr>* measure_lanelines);

  bool DelHighlyOverlayLowConfLaneLine(
      std::vector<LaneLinePtr>* measure_lanelines);

  bool DelMiddleOverlayFarLaneline(std::vector<LaneLinePtr>* measure_lanelines);

  bool DelAnomalyIntervalLaneLine(std::vector<LaneLinePtr>* measure_lanelines);

  std::vector<LaneLinePtr> GetForkOrMergeLaneLine(
      const std::vector<LaneLinePtr>& measure_lanelines);

  std::vector<LaneLinePtr> GetUnForkOrMergeLaneLine(
      const std::vector<LaneLinePtr>& measure_lanelines);

  bool IsSamePosLaneLine(const LaneLineConstPtr& ori_lanelane,
                         const LaneLineConstPtr& compare_laneline);

 private:
  float same_pos_distance_thresh_;
  float hight_overlay_length_ratio_thresh_;
  float length_ratio_thresh_;
  float low_overlay_length_ratio_thresh_;
  float anomaly_inter_ratio_thresh_;
  float min_interval_thresh_;
  float short_lane_length_thresh_;
  float far_distance_thresh_;
  float interval_diff_ratio_thresh_;
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
