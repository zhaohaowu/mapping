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
#include "modules/laneline_postprocess/lib/laneline/curve_fitter/base_curve_fitter.h"
#include "modules/laneline_postprocess/lib/laneline/proto/lane_postprocess.pb.h"
#include "modules/laneline_postprocess/lib/laneline/tracker/bev_lane_tracker/base/lane_filter.h"
#include "modules/laneline_postprocess/lib/laneline/utils/laneline_polynomial.h"
#include "perception-base/base/laneline/base_laneline.h"

namespace hozon {
namespace mp {
namespace environment {

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
  bool countColorProbability(const base::LaneLineMeasurementPtr& measurement);
  int countContinueColorNum(const base::LaneLineMeasurementPtr& measurement);
  base::LaneLineColor getTrackColor(
      const base::LaneLineMeasurementPtr& measurement);
  void UpdateWithMeasurement(const LaneFilterOptions& filter_options,
                             const base::LaneLineMeasurementPtr& measurement);

  void UpdateWithoutMeasurement(const LaneFilterOptions& filter_options);

  void Reset();

 private:
  void UpdateResult();

 private:
  std::unordered_map<base::LaneLineColor, double> color_probability_;
  LaneColorFilterParam lane_color_filter_param_;
  base::LaneLineColor max_count_color_;
  boost::circular_buffer<base::LaneLineColor> lastest_n_measures_color_;
  base::LaneLineColor final_track_color_;
  int max_history_window_ = 10;
  float color_keep_weight_ = 1.0;
  int color_count_threshold_ = 3;
};

}  // namespace environment
}  // namespace mp
}  // namespace hozon
