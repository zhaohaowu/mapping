// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: hozon
// @file: lane_type_filter.h
// @brief: filter 3d points

#pragma once

#include <memory>
#include <unordered_map>
#include <utility>

#include "boost/circular_buffer.hpp"
#include "depend/common/util/perf_util.h"
#include "modules/local_mapping/lib/interface/base_init_options.h"
#include "modules/local_mapping/lib/target/base_target.h"
#include "modules/util/include/util/mapping_log.h"

namespace hozon {
namespace mp {
namespace lm {

template <typename LineType, typename MeasureType, typename TargetType>
class BaseLaneLineTypeFilter {
 public:
  using MeasurementPtr = std::shared_ptr<MeasureType>;
  using TargetPtr = std::shared_ptr<TargetType>;

 public:
  explicit BaseLaneLineTypeFilter(TargetPtr lane_target)
      : target_ref_(std::move(lane_target)) {}
  virtual ~BaseLaneLineTypeFilter() = default;
  BaseLaneLineTypeFilter(const BaseLaneLineTypeFilter&) = delete;
  BaseLaneLineTypeFilter& operator=(const BaseLaneLineTypeFilter&) = delete;

  bool Init(const FilterInitOption& init_options);
  bool countTypeProbability(const MeasurementPtr& measurement);
  int countContinueTypeNum(const MeasurementPtr& measurement);
  LineType getTrackType(const MeasurementPtr& measurement);
  void UpdateWithMeasurement(const FilterOption& filter_options,
                             const MeasurementPtr& measurement);

  void UpdateWithoutMeasurement(const FilterOption& filter_options);

  void Reset();

 protected:
  void UpdateResult();

 protected:
  TargetPtr target_ref_;
  std::unordered_map<LineType, double> type_probability_;
  // todo 加上配置化参数
  // LaneLineTypeFilterParam lane_type_filter_param_;
  LineType max_count_type_;
  boost::circular_buffer<LineType> lastest_n_measures_type_;
  LineType final_track_type_;
  int max_history_window_ = 10;
  float type_keep_weight_ = 1.0;
  int type_count_threshold_ = 3;
};

template <typename LineType, typename MeasureType, typename TargetType>
bool BaseLaneLineTypeFilter<LineType, MeasureType, TargetType>::Init(
    const FilterInitOption& init_options) {
  // todo 配置参数
  // lane_color_filter_param_ = init_options.lane_color_filter_param;

  // max_history_window_ = lane_color_filter_param_.max_history_window();
  lastest_n_measures_type_.set_capacity(max_history_window_);
  //   // 维持状态的时间长短
  //   type_keep_weight_ = lane_color_filter_param_.color_keep_weight();
  //   // 类型连续N帧阈值
  //   color_count_threshold_ =
  //   lane_color_filter_param_.color_count_threshold();
  return true;
}

// 统计不同类型出现的概率和最大概率类型
// 返回不同类型的统计概率值
template <typename LineType, typename MeasureType, typename TargetType>
bool BaseLaneLineTypeFilter<LineType, MeasureType, TargetType>::
    countTypeProbability(const MeasurementPtr& measurement) {
  const LineType& detect_type = measurement->type;
  max_count_type_ = target_ref_->GetConstTrackedObject()->type;
  if (type_probability_.find(detect_type) != type_probability_.end()) {
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
template <typename LineType, typename MeasureType, typename TargetType>
int BaseLaneLineTypeFilter<LineType, MeasureType, TargetType>::
    countContinueTypeNum(const MeasurementPtr& measurement) {
  int count = 1;
  int size_type = lastest_n_measures_type_.size();
  const LineType& detect_type = measurement->type;
  // 判断连续多帧的检测结果
  if (detect_type != target_ref_->GetConstTrackedObject()->type) {
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

template <typename LineType, typename MeasureType, typename TargetType>
LineType
BaseLaneLineTypeFilter<LineType, MeasureType, TargetType>::getTrackType(
    const MeasurementPtr& measurement) {
  LineType track_type = target_ref_->GetConstTrackedObject()->type;
  const LineType& detect_type = measurement->type;

  int type_count_threshold = type_count_threshold_;
  // 1. 根据统计类型概率的最大值选取类型
  countTypeProbability(measurement);
  track_type = max_count_type_;
  // 2. 根据连续多帧检测出同一种类型切换
  int count = countContinueTypeNum(measurement);
  if (count >= type_count_threshold && detect_type != LineType::UNKNOWN) {
    type_probability_.clear();
    // 为了防止下一次检测秒切类型,默认是1，可配置
    type_probability_[detect_type] = type_keep_weight_;
    track_type = detect_type;
  }

  HLOG_DEBUG << " [LaneTypeCount]: "
             << ", trackID: " << target_ref_->GetConstTrackedObject()->id
             << ", original_type: "
             << static_cast<int>(target_ref_->GetConstTrackedObject()->type)
             << ", detect_type: " << static_cast<int>(detect_type)
             << ", track_type: " << static_cast<int>(track_type)
             << ", continue_count: " << count;

  return track_type;
}

template <typename LineType, typename MeasureType, typename TargetType>
void BaseLaneLineTypeFilter<LineType, MeasureType, TargetType>::
    UpdateWithMeasurement(const FilterOption& filter_options,
                          const MeasurementPtr& measurement) {
  // PERF_FUNCTION();
  // PERF_BLOCK_START();

  lastest_n_measures_type_.push_back(measurement->type);
  final_track_type_ = getTrackType(measurement);
  UpdateResult();
  // PERF_BLOCK_END("LaneTyperFilter UpdateWithMeasurement Used Time");
  return;
}

template <typename LineType, typename MeasureType, typename TargetType>
void BaseLaneLineTypeFilter<LineType, MeasureType, TargetType>::
    UpdateWithoutMeasurement(const FilterOption& filter_options) {
  // 没有检测时， 保持上一帧结果进行输出。
  // final_track_type_ = target_ref_->GetConstTrackedObject()->type;
  // UpdateResult();

  return;
}

template <typename LineType, typename MeasureType, typename TargetType>
void BaseLaneLineTypeFilter<LineType, MeasureType, TargetType>::UpdateResult() {
  target_ref_->GetTrackedObject()->type = final_track_type_;
}

template <typename LineType, typename MeasureType, typename TargetType>
void BaseLaneLineTypeFilter<LineType, MeasureType, TargetType>::Reset() {
  return;
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
