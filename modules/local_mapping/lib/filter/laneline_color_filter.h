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
#include "modules/local_mapping/lib/interface/base_init_options.h"
#include "modules/local_mapping/lib/target/base_target.h"

namespace hozon {
namespace mp {
namespace lm {

class LaneLineColorFilter{
 public:
  explicit LaneLineColorFilter(LaneTargetPtr lane_target)
      : target_ref_(std::move(lane_target)) {}
  ~LaneLineColorFilter() = default;
  LaneLineColorFilter(const LaneLineColorFilter&) = delete;
  LaneLineColorFilter& operator=(const LaneLineColorFilter&) = delete;

  bool Init(const FilterInitOption& init_options);
  bool countColorProbability(const LaneLinePtr& measurement);
  int countContinueColorNum(const LaneLinePtr& measurement);
  LaneLineColor getTrackColor(
      const LaneLinePtr& measurement);
  void UpdateWithMeasurement(const FilterOption& filter_options,
                             const LaneLinePtr& measurement);

  void UpdateWithoutMeasurement(const FilterOption& filter_options);

  void Reset();

 private:
  void UpdateResult();

 private:
  std::unordered_map<LaneLineColor, double> color_probability_;
  // todo: 配置参数
  // LaneLineColorFilterParam lane_color_filter_param_;
  LaneLineColor max_count_color_;
  boost::circular_buffer<LaneLineColor> lastest_n_measures_color_;
  LaneLineColor final_track_color_;
  int max_history_window_ = 10;
  float color_keep_weight_ = 1.0;
  int color_count_threshold_ = 3;
  LaneTargetPtr target_ref_;
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
