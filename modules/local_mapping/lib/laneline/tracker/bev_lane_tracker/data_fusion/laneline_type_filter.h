// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: hozon
// @file: lane_type_filter.h
// @brief: filter 3d points

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "boost/circular_buffer.hpp"
#include "modules/local_mapping/lib/laneline/curve_fitter/base_curve_fitter.h"
#include "modules/local_mapping/lib/laneline/proto/lane_postprocess.pb.h"
#include "modules/local_mapping/lib/laneline/tracker/bev_lane_tracker/base/lane_filter.h"
#include "modules/local_mapping/lib/laneline/utils/laneline_polynomial.h"
#include "perception-base/base/laneline/base_laneline.h"

namespace hozon {
namespace mp {
namespace environment {

namespace perception_base = hozon::perception::base;
struct LaneTypeFilterInitOptions {
  LaneTypeFilterParam lane_type_filter_param;
};

class LaneTypeFilter : public BaseLaneFilter {
 public:
  explicit LaneTypeFilter(LaneTargetPtr lane_target)
      : BaseLaneFilter(lane_target) {}
  virtual ~LaneTypeFilter() {}
  LaneTypeFilter(const LaneTypeFilter&) = delete;
  LaneTypeFilter& operator=(const LaneTypeFilter&) = delete;

  bool Init(const LaneTypeFilterInitOptions& init_options);
  bool countTypeProbability(const LaneLineMeasurementPtr& measurement);
  int countContinueTypeNum(const LaneLineMeasurementPtr& measurement);
  perception_base::LaneLineType getTrackType(
      const LaneLineMeasurementPtr& measurement);
  void UpdateWithMeasurement(const LaneFilterOptions& filter_options,
                             const LaneLineMeasurementPtr& measurement);

  void UpdateWithoutMeasurement(const LaneFilterOptions& filter_options);

  void Reset();

 private:
  void UpdateResult();

 private:
  std::unordered_map<perception_base::LaneLineType, double> type_probability_;
  LaneTypeFilterParam lane_type_filter_param_;
  perception_base::LaneLineType max_count_type_;
  boost::circular_buffer<perception_base::LaneLineType>
      lastest_n_measures_type_;
  perception_base::LaneLineType final_track_type_;
  int max_history_window_ = 10;
  float type_keep_weight_ = 1.0;
  int type_count_threshold_ = 3;
};

class RoadEdgeTypeFilter : public BaseRoadEdgeFilter {
 public:
  explicit RoadEdgeTypeFilter(RoadEdgeTargetPtr lane_target)
      : BaseRoadEdgeFilter(lane_target) {}
  virtual ~RoadEdgeTypeFilter() {}
  RoadEdgeTypeFilter(const RoadEdgeTypeFilter&) = delete;
  RoadEdgeTypeFilter& operator=(const RoadEdgeTypeFilter&) = delete;

  bool Init(const LaneTypeFilterInitOptions& init_options);
  bool countTypeProbability(const RoadEdgeMeasurementPtr& measurement);
  int countContinueTypeNum(const RoadEdgeMeasurementPtr& measurement);
  perception_base::RoadEdgeType getTrackType(
      const RoadEdgeMeasurementPtr& measurement);
  void UpdateWithMeasurement(const LaneFilterOptions& filter_options,
                             const RoadEdgeMeasurementPtr& measurement);

  void UpdateWithoutMeasurement(const LaneFilterOptions& filter_options);

  void Reset();

 private:
  void UpdateResult();

 private:
  std::unordered_map<perception_base::RoadEdgeType, double> type_probability_;
  LaneTypeFilterParam lane_type_filter_param_;
  perception_base::RoadEdgeType max_count_type_;
  boost::circular_buffer<perception_base::RoadEdgeType>
      lastest_n_measures_type_;
  perception_base::RoadEdgeType final_track_type_;
  int max_history_window_ = 10;
  float type_keep_weight_ = 1.0;
  int type_count_threshold_ = 3;
};

}  // namespace environment
}  // namespace mp
}  // namespace hozon
