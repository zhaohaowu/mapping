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

struct LaneColorFilterInitOptions {
  LaneColorFilterParam lane_color_filter_param;
};

class LaneColorFilter : public BaseLaneFilter {
 public:
  explicit LaneColorFilter(LaneTargetPtr lane_target)
      : BaseLaneFilter(lane_target) {}
  virtual ~LaneColorFilter() {}
  LaneColorFilter(const LaneColorFilter&) = delete;
  LaneColorFilter& operator=(const LaneColorFilter&) = delete;

  bool Init(const LaneColorFilterInitOptions& init_options);
  bool countColorProbability(
      const perception_base::LaneLineMeasurementPtr& measurement);
  int countContinueColorNum(
      const perception_base::LaneLineMeasurementPtr& measurement);
  perception_base::LaneLineColor getTrackColor(
      const perception_base::LaneLineMeasurementPtr& measurement);
  void UpdateWithMeasurement(
      const LaneFilterOptions& filter_options,
      const perception_base::LaneLineMeasurementPtr& measurement);

  void UpdateWithoutMeasurement(const LaneFilterOptions& filter_options);

  void Reset();

 private:
  void UpdateResult();

 private:
  std::unordered_map<perception_base::LaneLineColor, double> color_probability_;
  LaneColorFilterParam lane_color_filter_param_;
  perception_base::LaneLineColor max_count_color_;
  boost::circular_buffer<perception_base::LaneLineColor>
      lastest_n_measures_color_;
  perception_base::LaneLineColor final_track_color_;
  int max_history_window_ = 10;
  float color_keep_weight_ = 1.0;
  int color_count_threshold_ = 3;
};

}  // namespace environment
}  // namespace mp
}  // namespace hozon
