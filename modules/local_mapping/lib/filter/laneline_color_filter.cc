// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Li guo (liguo03@hozon.com)
// @file: lane_point_filter.cc
// @brief: filter 3d points

#include "modules/local_mapping/lib/filter/laneline_color_filter.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <numeric>
#include <random>
#include <sstream>
#include <utility>

#include "depend/common/util/perf_util.h"
#include "modules/util/include/util/mapping_log.h"

namespace hozon {
namespace mp {
namespace lm {

bool LaneLineColorFilter::Init(const FilterInitOption& init_options) {
  // todo 配置参数
  // lane_color_filter_param_ = init_options.lane_color_filter_param;

  // max_history_window_ = lane_color_filter_param_.max_history_window();
  // lastest_n_measures_color_.set_capacity(max_history_window_);
  // // 维持状态的时间长短
  // color_keep_weight_ = lane_color_filter_param_.color_keep_weight();
  // // 类型连续N帧阈值
  // color_count_threshold_ = lane_color_filter_param_.color_count_threshold();
  return true;
}

// 统计不同颜色出现的概率和最大概率类型
// 返回不同颜色的统计概率值
bool LaneLineColorFilter::countColorProbability(const LaneLinePtr& measurement) {
  const LaneLineColor& detect_color = measurement->color;
  max_count_color_ = target_ref_->GetConstTrackedObject()->color;
  if (!(color_probability_.find(detect_color) == color_probability_.end())) {
    color_probability_[detect_color] += measurement->color_confidence;
  } else {
    color_probability_[detect_color] = measurement->color_confidence;
  }
  double color_score = -1.0;
  for (const auto& pair : color_probability_) {
    if (pair.second > color_score) {
      color_score = pair.second;
      max_count_color_ = pair.first;
    }
  }
  return true;
}
// 统计连续多帧的检测类型count数
int LaneLineColorFilter::countContinueColorNum(
    const LaneLinePtr& measurement) {
  int count = 1, size_color = lastest_n_measures_color_.size();
  const LaneLineColor& detect_color = measurement->color;
  // 判断连续多帧的检测结果
  if (detect_color != target_ref_->GetConstTrackedObject()->color) {
    for (int index = size_color - 1; index >= 1; index--) {
      if (detect_color == lastest_n_measures_color_[index - 1]) {
        count++;
      } else {
        break;
      }
    }
  }
  return count;
}

LaneLineColor LaneLineColorFilter::getTrackColor(
    const LaneLinePtr& measurement) {
  LaneLineColor track_color =
      target_ref_->GetConstTrackedObject()->color;
  const LaneLineColor& detect_color = measurement->color;

  int color_count_threshold = color_count_threshold_;
  // 1. 根据统计类型概率的最大值选取颜色
  countColorProbability(measurement);
  track_color = max_count_color_;
  // 2. 根据连续多帧检测出同一种颜色切换
  int count = countContinueColorNum(measurement);
  if (count >= color_count_threshold &&
      detect_color != LaneLineColor::UNKNOWN) {
    color_probability_.clear();
    // 为了防止下一次检测秒切颜色,默认是1，可配置
    color_probability_[detect_color] = color_keep_weight_;
    track_color = detect_color;
  }

  HLOG_DEBUG << " [LaneColorCount]: "
             << ", trackID: " << target_ref_->GetConstTrackedObject()->id
             << ", original_color: "
             << static_cast<int>(
                    target_ref_->GetConstTrackedObject()->color)
             << ", detect_color: " << static_cast<int>(detect_color)
             << ", track_color: " << static_cast<int>(track_color)
             << ", continue_count: " << count;

  return track_color;
}

void LaneLineColorFilter::UpdateWithMeasurement(
    const FilterOption& filter_options,
    const LaneLinePtr& measurement) {
  PERF_BLOCK_START();

  lastest_n_measures_color_.push_back(measurement->color);
  final_track_color_ = getTrackColor(measurement);
  UpdateResult();
  PERF_BLOCK_END("LaneLineColorFilter UpdateWithMeasurement Used Time");
}

void LaneLineColorFilter::UpdateWithoutMeasurement(
    const FilterOption& filter_options) {
  // 没有检测时， 保持上一帧结果进行输出。
  // final_track_color_ = lane_target_ref_->GetTrackedLaneLine()->color;
  // UpdateResult();

  return;
}

void LaneLineColorFilter::UpdateResult() {
  target_ref_->GetTrackedObject()->color = final_track_color_;
}

void LaneLineColorFilter::Reset() { return; }

}  // namespace lm
}  // namespace mp
}  // namespace hozon
