// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Li guo (liguo03@hozon.com)
// @file: lane_point_filter.cc
// @brief: filter 3d points

#include "modules/laneline_postprocess/lib/laneline/tracker/bev_lane_tracker/data_fusion/laneline_type_filter.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <numeric>
#include <random>
#include <sstream>
#include <utility>

#include "modules/laneline_postprocess/lib/laneline/tracker/bev_lane_tracker/base/curve_fitter.h"
#include "modules/laneline_postprocess/lib/laneline/tracker/bev_lane_tracker/datalogger/load_data_singleton.h"
// #include "perception-common/common/performance/perf_util.h"
namespace hozon {
namespace mp {
namespace environment {

bool LaneTypeFilter::Init(const LaneTypeFilterInitOptions& init_options) {
  lane_type_filter_param_ = init_options.lane_type_filter_param;

  max_history_window_ = lane_type_filter_param_.max_history_window();
  lastest_n_measures_type_.set_capacity(max_history_window_);
  // 维持状态的时间长短
  type_keep_weight_ = lane_type_filter_param_.type_keep_weight();
  // 类型连续N帧阈值
  type_count_threshold_ = lane_type_filter_param_.type_count_threshold();
  return true;
}

// 统计不同类型出现的概率和最大概率类型
// 返回不同类型的统计概率值
bool LaneTypeFilter::countTypeProbability(
    const perception_base::LaneLineMeasurementPtr& measurement) {
  const perception_base::LaneLineType& detect_type = measurement->type;
  max_count_type_ = lane_target_ref_->GetConstTrackedLaneLine()->type;
  if (!(type_probability_.find(detect_type) == type_probability_.end())) {
    type_probability_[detect_type] += measurement->type_confidence;
  } else {
    type_probability_[detect_type] = measurement->type_confidence;
  }
  double type_score = -1.0;
  for (const auto& pair : type_probability_) {
    if (pair.second > type_score) {
      type_score = pair.second;
      max_count_type_ = pair.first;
    }
  }
  return true;
}
// 统计连续多帧的检测类型count数
int LaneTypeFilter::countContinueTypeNum(
    const perception_base::LaneLineMeasurementPtr& measurement) {
  int count = 1, size_type = lastest_n_measures_type_.size();
  const perception_base::LaneLineType& detect_type = measurement->type;
  // 判断连续多帧的检测结果
  if (detect_type != lane_target_ref_->GetConstTrackedLaneLine()->type) {
    for (int index = size_type - 1; index >= 1; index--) {
      if (detect_type == lastest_n_measures_type_[index - 1]) {
        count++;
      } else {
        break;
      }
    }
  }
  return count;
}

perception_base::LaneLineType LaneTypeFilter::getTrackType(
    const perception_base::LaneLineMeasurementPtr& measurement) {
  perception_base::LaneLineType track_type =
      lane_target_ref_->GetConstTrackedLaneLine()->type;
  const perception_base::LaneLineType& detect_type = measurement->type;

  int type_count_threshold = type_count_threshold_;
  // 1. 根据统计类型概率的最大值选取类型
  countTypeProbability(measurement);
  track_type = max_count_type_;
  // 2. 根据连续多帧检测出同一种类型切换
  int count = countContinueTypeNum(measurement);
  if (count >= type_count_threshold &&
      detect_type != perception_base::LaneLineType::Unknown) {
    type_probability_.clear();
    // 为了防止下一次检测秒切类型,默认是1，可配置
    type_probability_[detect_type] = type_keep_weight_;
    track_type = detect_type;
  }

  HLOG_DEBUG << " [LaneTypeCount]: "
             << ", trackID: " << lane_target_ref_->GetConstTrackedLaneLine()->id
             << ", original_type: "
             << static_cast<int>(
                    lane_target_ref_->GetConstTrackedLaneLine()->type)
             << ", detect_type: " << static_cast<int>(detect_type)
             << ", track_type: " << static_cast<int>(track_type)
             << ", continue_count: " << count;

  return track_type;
}

void LaneTypeFilter::UpdateWithMeasurement(
    const LaneFilterOptions& filter_options,
    const perception_base::LaneLineMeasurementPtr& measurement) {
  // PERF_FUNCTION();
  // PERF_BLOCK_START();

  lastest_n_measures_type_.push_back(measurement->type);
  final_track_type_ = getTrackType(measurement);
  UpdateResult();
  // PERF_BLOCK_END("LaneTyperFilter UpdateWithMeasurement Used Time");
  return;
}

void LaneTypeFilter::UpdateWithoutMeasurement(
    const LaneFilterOptions& filter_options) {
  // 没有检测时， 保持上一帧结果进行输出。
  // final_track_type_ = lane_target_ref_->GetTrackedLaneLine()->type;
  // UpdateResult();

  return;
}

void LaneTypeFilter::UpdateResult() {
  lane_target_ref_->GetTrackedLaneLine()->type = final_track_type_;
}

void LaneTypeFilter::Reset() { return; }

bool RoadEdgeTypeFilter::Init(const LaneTypeFilterInitOptions& init_options) {
  lane_type_filter_param_ = init_options.lane_type_filter_param;

  max_history_window_ = lane_type_filter_param_.max_history_window();
  lastest_n_measures_type_.set_capacity(max_history_window_);
  // 维持状态的时间长短
  type_keep_weight_ = lane_type_filter_param_.type_keep_weight();
  // 类型连续N帧阈值
  type_count_threshold_ = lane_type_filter_param_.type_count_threshold();
  return true;
}

// 统计不同类型出现的概率和最大概率类型
// 返回不同类型的统计概率值
bool RoadEdgeTypeFilter::countTypeProbability(
    const perception_base::RoadEdgeMeasurementPtr& measurement) {
  const perception_base::RoadEdgeType& detect_type = measurement->type;
  max_count_type_ = lane_target_ref_->GetConstTrackedLaneLine()->type;
  if (!(type_probability_.find(detect_type) == type_probability_.end())) {
    type_probability_[detect_type] += measurement->confidence;
  } else {
    type_probability_[detect_type] = measurement->confidence;
  }
  double type_score = -1.0;
  for (const auto& pair : type_probability_) {
    if (pair.second > type_score) {
      type_score = pair.second;
      max_count_type_ = pair.first;
    }
  }
  return true;
}
// 统计连续多帧的检测类型count数
int RoadEdgeTypeFilter::countContinueTypeNum(
    const perception_base::RoadEdgeMeasurementPtr& measurement) {
  int count = 1, size_type = lastest_n_measures_type_.size();
  const perception_base::RoadEdgeType& detect_type = measurement->type;
  // 判断连续多帧的检测结果
  if (detect_type != lane_target_ref_->GetConstTrackedLaneLine()->type) {
    for (int index = size_type - 1; index >= 1; index--) {
      if (detect_type == lastest_n_measures_type_[index - 1]) {
        count++;
      } else {
        break;
      }
    }
  }
  return count;
}

perception_base::RoadEdgeType RoadEdgeTypeFilter::getTrackType(
    const perception_base::RoadEdgeMeasurementPtr& measurement) {
  perception_base::RoadEdgeType track_type =
      lane_target_ref_->GetConstTrackedLaneLine()->type;
  const perception_base::RoadEdgeType& detect_type = measurement->type;

  int type_count_threshold = type_count_threshold_;
  // 1. 根据统计类型概率的最大值选取类型
  countTypeProbability(measurement);
  track_type = max_count_type_;
  // 2. 根据连续多帧检测出同一种类型切换
  int count = countContinueTypeNum(measurement);
  if (count >= type_count_threshold &&
      detect_type != perception_base::RoadEdgeType::UNKNOWN) {
    type_probability_.clear();
    // 为了防止下一次检测秒切类型,默认是1，可配置
    type_probability_[detect_type] = type_keep_weight_;
    track_type = detect_type;
  }

  HLOG_DEBUG << " [LaneTypeCount]: "
             << ", trackID: " << lane_target_ref_->GetConstTrackedLaneLine()->id
             << ", original_type: "
             << static_cast<int>(
                    lane_target_ref_->GetConstTrackedLaneLine()->type)
             << ", detect_type: " << static_cast<int>(detect_type)
             << ", track_type: " << static_cast<int>(track_type)
             << ", continue_count: " << count;

  return track_type;
}

void RoadEdgeTypeFilter::UpdateWithMeasurement(
    const LaneFilterOptions& filter_options,
    const perception_base::RoadEdgeMeasurementPtr& measurement) {
  // PERF_FUNCTION();
  // PERF_BLOCK_START();

  lastest_n_measures_type_.push_back(measurement->type);
  final_track_type_ = getTrackType(measurement);
  UpdateResult();
  // PERF_BLOCK_END("LaneTyperFilter UpdateWithMeasurement Used Time");
  return;
}

void RoadEdgeTypeFilter::UpdateWithoutMeasurement(
    const LaneFilterOptions& filter_options) {
  // 没有检测时， 保持上一帧结果进行输出。
  // final_track_type_ = lane_target_ref_->GetTrackedLaneLine()->type;
  // UpdateResult();

  return;
}

void RoadEdgeTypeFilter::UpdateResult() {
  lane_target_ref_->GetTrackedLaneLine()->type = final_track_type_;
}

void RoadEdgeTypeFilter::Reset() { return; }

}  // namespace environment
}  // namespace mp
}  // namespace hozon
