// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Li guo (liguo03@hozon.com)
// @file: lane_point_filter.cc
// @brief: filter 3d points

#include "modules/local_mapping/lib/filter/zebracrossing_point_filter.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <iostream>
#include <memory>
#include <numeric>
#include <random>
#include <sstream>
#include <utility>

#include "modules/local_mapping/lib/datalogger/map_manager.h"
#include "modules/local_mapping/lib/datalogger/pose_manager.h"
#include "modules/local_mapping/utils/common.h"

namespace hozon {
namespace mp {
namespace lm {

bool ZebraCrossingPointFilter::Init() {
  // history_measure_stoplines_.resize(history_measure_size_);
  history_measure_zebracrossings_.set_capacity(history_measure_size_);
  history_measure_zebracrossings_.clear();
  return true;
}

bool ZebraCrossingPointFilter::IsAbnormalPose(
    const Eigen::Affine3d& novatel2world_pose) {
  return true;
}

bool ZebraCrossingPointFilter::CheckCenterPointStableMeasureState() {
  // 观测数据量小于10帧时，直接认为状态不稳定
  if (history_measure_zebracrossings_.size() < history_measure_size_) {
    return false;
  }

  // 如果状态稳定，跟踪结果等于10帧观测结果的平均值，且"拍死"不再更新。
  if (center_point_is_stable_state_) {
    return true;
  }

  // 默认状态不稳定
  // return false;

  std::vector<float> measurement_xs;
  std::vector<float> measurement_ys;
  measurement_xs.clear();
  measurement_ys.clear();

  for (const auto& measure_zebracrossing_ptr :
       history_measure_zebracrossings_) {
    measurement_xs.push_back(measure_zebracrossing_ptr->center_point.x());
    measurement_ys.push_back(measure_zebracrossing_ptr->center_point.y());
  }

  float max_x = *max_element(measurement_xs.begin(), measurement_xs.end());
  float min_x = *min_element(measurement_xs.begin(), measurement_xs.end());
  float max_y = *max_element(measurement_ys.begin(), measurement_ys.end());
  float min_y = *min_element(measurement_ys.begin(), measurement_ys.end());

  if (max_x - min_x < 0.3 && max_y - min_y < 0.1) {
    auto& track_zebracrossing = target_ref_->GetTrackedObject();
    float sum_x = 0.0;
    float sum_y = 0.0;
    for (const auto& measurement_x : measurement_xs) {
      sum_x += measurement_x;
    }
    for (const auto& measurement_y : measurement_ys) {
      sum_y += measurement_y;
    }
    track_zebracrossing->center_point =
        Eigen::Vector3d{sum_x / static_cast<float>(history_measure_size_),
                        sum_y / static_cast<float>(history_measure_size_), 0};
    center_point_is_stable_state_ = true;
    return true;
  }

  return false;
}

bool ZebraCrossingPointFilter::CheckLengthStableMeasureState() {
  // 观测数据量小于10帧时，直接认为状态不稳定
  if (history_measure_zebracrossings_.size() < history_measure_size_) {
    return false;
  }

  // 如果状态稳定，跟踪结果等于10帧观测结果的平均值，且"拍死"不再更新。
  if (length_is_stable_state_) {
    return true;
  }

  // 默认状态不稳定
  // return false;

  std::vector<float> measurement_lengths;
  measurement_lengths.clear();

  for (const auto& measure_zebracrossing_ptr :
       history_measure_zebracrossings_) {
    measurement_lengths.push_back(measure_zebracrossing_ptr->length);
  }

  float max_length =
      *max_element(measurement_lengths.begin(), measurement_lengths.end());
  float min_length =
      *min_element(measurement_lengths.begin(), measurement_lengths.end());
  // HLOG_ERROR << "max_length - min_length: " << max_length - min_length;
  if (max_length - min_length < 0.3) {
    float sum_length = 0.0;
    for (const auto& measurement_length : measurement_lengths) {
      sum_length += measurement_length;
    }
    auto& track_zebracrossing = target_ref_->GetTrackedObject();
    track_zebracrossing->length =
        sum_length / static_cast<float>(history_measure_size_);
    length_is_stable_state_ = true;
    return true;
  }

  return false;
}

bool ZebraCrossingPointFilter::CheckWidthStableMeasureState() {
  // 观测数据量小于10帧时，直接认为状态不稳定
  if (history_measure_zebracrossings_.size() < history_measure_size_) {
    return false;
  }

  // 如果状态稳定，跟踪结果等于10帧观测结果的平均值，且"拍死"不再更新。
  if (width_is_stable_state_) {
    return true;
  }

  // 默认状态不稳定
  // return false;

  std::vector<float> measurement_widths;
  measurement_widths.clear();

  for (const auto& measure_zebracrossing_ptr :
       history_measure_zebracrossings_) {
    measurement_widths.push_back(measure_zebracrossing_ptr->width);
  }

  float max_width =
      *max_element(measurement_widths.begin(), measurement_widths.end());
  float min_width =
      *min_element(measurement_widths.begin(), measurement_widths.end());
  // HLOG_ERROR << "max_width - min_width: " << max_width - min_width;
  if (max_width - min_width < 0.3) {
    float sum_width = 0.0;
    for (const auto& measurement_width : measurement_widths) {
      sum_width += measurement_width;
    }
    auto& track_zebracrossing = target_ref_->GetTrackedObject();
    track_zebracrossing->width =
        sum_width / static_cast<float>(history_measure_size_);
    width_is_stable_state_ = true;
    return true;
  }

  return false;
}

bool ZebraCrossingPointFilter::CheckHeadingStableMeasureState() {
  // 观测数据量小于10帧时，直接认为状态不稳定
  if (history_measure_zebracrossings_.size() < history_measure_size_) {
    return false;
  }

  // 如果状态稳定，跟踪结果等于10帧观测结果的平均值，且"拍死"不再更新。
  if (heading_is_stable_state_) {
    return true;
  }

  // 默认状态不稳定
  // return false;

  std::vector<float> measurement_headings;
  measurement_headings.clear();

  for (const auto& measure_zebracrossing_ptr :
       history_measure_zebracrossings_) {
    measurement_headings.push_back(measure_zebracrossing_ptr->heading);
  }

  float max_heading =
      *max_element(measurement_headings.begin(), measurement_headings.end());
  float min_heading =
      *min_element(measurement_headings.begin(), measurement_headings.end());
  // // HLOG_ERROR << "斑马线max_heading - min_heading: "
  // << max_heading - min_heading;
  if (max_heading - min_heading < 0.17) {
    float sum_heading = 0.0;
    for (const auto& measurement_heading : measurement_headings) {
      sum_heading += measurement_heading;
    }
    auto& track_zebracrossing = target_ref_->GetTrackedObject();
    track_zebracrossing->heading =
        sum_heading / static_cast<float>(history_measure_size_);
    heading_is_stable_state_ = true;
    return true;
  }

  return false;
}

void ZebraCrossingPointFilter::UpdateWithMeasurement(
    const ZebraCrossingPtr& measurement) {
  history_measure_zebracrossings_.push_back(measurement);

  if (!CheckCenterPointStableMeasureState()) {
    UpdateCenterPoint(measurement);
  }
  if (!CheckLengthStableMeasureState()) {
    UpdateLength(measurement);
  }
  if (!CheckWidthStableMeasureState()) {
    UpdateWidth(measurement);
  }
  if (!CheckHeadingStableMeasureState()) {
    UpdateHeading(measurement);
  }
  if (!length_is_stable_state_ || !width_is_stable_state_ ||
      !heading_is_stable_state_ || !center_point_is_stable_state_) {
    UpdateVehiclePoints();
  }
}

void ZebraCrossingPointFilter::UpdateWithoutMeasurement() {
  // 未匹配时， 保持上一帧跟踪结果。
}

void ZebraCrossingPointFilter::Reset() {
  history_measure_zebracrossings_.clear();
}

void ZebraCrossingPointFilter::UpdateCenterPoint(
    const ZebraCrossingPtr& measurement) {
  auto& track_zebracrossing_ptr = target_ref_->GetTrackedObject();
  track_zebracrossing_ptr->center_point =
      0.5 * (measurement->center_point + track_zebracrossing_ptr->center_point);
}

void ZebraCrossingPointFilter::UpdateLength(
    const ZebraCrossingPtr& measurement) {
  auto& track_zebracrossing_ptr = target_ref_->GetTrackedObject();
  track_zebracrossing_ptr->length =
      0.5 * (measurement->length + track_zebracrossing_ptr->length);
}

void ZebraCrossingPointFilter::UpdateWidth(
    const ZebraCrossingPtr& measurement) {
  auto& track_zebracrossing_ptr = target_ref_->GetTrackedObject();
  track_zebracrossing_ptr->width =
      0.5 * (measurement->width + track_zebracrossing_ptr->width);
}

void ZebraCrossingPointFilter::UpdateHeading(
    const ZebraCrossingPtr& measurement) {
  auto& track_zebracrossing_ptr = target_ref_->GetTrackedObject();
  LaneLinePtr before_left_lane_line = std::make_shared<LaneLine>();
  LaneLinePtr before_right_lane_line = std::make_shared<LaneLine>();
  LaneLinePtr after_left_lane_line = std::make_shared<LaneLine>();
  LaneLinePtr after_right_lane_line = std::make_shared<LaneLine>();

  LocalMapFramePtr localmap_frame = MAP_MANAGER->GetLocalMap();
  for (const auto& laneline_ptr : localmap_frame->lane_lines_ptr->lanelines) {
    if (!laneline_ptr->after_intersection) {
      if (laneline_ptr->position == LaneLinePosition::EGO_LEFT) {
        before_left_lane_line = laneline_ptr;
      } else if (laneline_ptr->position == LaneLinePosition::EGO_RIGHT) {
        before_right_lane_line = laneline_ptr;
      }
    } else if (laneline_ptr->after_intersection) {
      if (laneline_ptr->position == LaneLinePosition::EGO_LEFT) {
        after_left_lane_line = laneline_ptr;
      } else if (laneline_ptr->position == LaneLinePosition::EGO_RIGHT) {
        after_right_lane_line = laneline_ptr;
      }
    }
  }
  double before_dis = FLT_MAX;
  double after_dis = FLT_MAX;
  if (before_left_lane_line->vehicle_points.empty() &&
      !before_right_lane_line->vehicle_points.empty()) {
    before_dis = fabs(track_zebracrossing_ptr->center_point.x() -
                      (before_right_lane_line->vehicle_points.back().x() +
                       before_right_lane_line->vehicle_points.front().x()) /
                          2);
  } else if (!before_left_lane_line->vehicle_points.empty() &&
             before_right_lane_line->vehicle_points.empty()) {
    before_dis = fabs(track_zebracrossing_ptr->center_point.x() -
                      (before_left_lane_line->vehicle_points.back().x() +
                       before_left_lane_line->vehicle_points.front().x()) /
                          2);
  } else if (!before_left_lane_line->vehicle_points.empty() &&
             !before_right_lane_line->vehicle_points.empty()) {
    before_dis = fabs(track_zebracrossing_ptr->center_point.x() -
                      (before_left_lane_line->vehicle_points.back().x() +
                       before_right_lane_line->vehicle_points.back().x() +
                       before_left_lane_line->vehicle_points.front().x() +
                       before_right_lane_line->vehicle_points.front().x()) /
                          4);
  }
  if (after_left_lane_line->vehicle_points.empty() &&
      !after_right_lane_line->vehicle_points.empty()) {
    after_dis = fabs(track_zebracrossing_ptr->center_point.x() -
                     (after_right_lane_line->vehicle_points.back().x() +
                      after_right_lane_line->vehicle_points.front().x()) /
                         2);
  } else if (!after_left_lane_line->vehicle_points.empty() &&
             after_right_lane_line->vehicle_points.empty()) {
    after_dis = fabs(track_zebracrossing_ptr->center_point.x() -
                     (after_left_lane_line->vehicle_points.back().x() +
                      after_left_lane_line->vehicle_points.front().x()) /
                         2);
  } else if (!after_left_lane_line->vehicle_points.empty() &&
             !after_right_lane_line->vehicle_points.empty()) {
    after_dis = fabs(track_zebracrossing_ptr->center_point.x() -
                     (after_left_lane_line->vehicle_points.front().x() +
                      after_right_lane_line->vehicle_points.front().x() +
                      after_left_lane_line->vehicle_points.back().x() +
                      after_right_lane_line->vehicle_points.back().x()) /
                         4);
  }
  LaneLinePtr left_lane_line = std::make_shared<LaneLine>();
  LaneLinePtr right_lane_line = std::make_shared<LaneLine>();
  if (before_dis <= after_dis) {
    left_lane_line = before_left_lane_line;
    right_lane_line = before_right_lane_line;
  } else if (before_dis > after_dis) {
    left_lane_line = after_left_lane_line;
    right_lane_line = after_right_lane_line;
  }
  int left_size = static_cast<int>(left_lane_line->vehicle_points.size());
  int right_size = static_cast<int>(right_lane_line->vehicle_points.size());
  if (left_size < 2 || right_size < 2) {
    track_zebracrossing_ptr->heading =
        0.5 * (measurement->heading + track_zebracrossing_ptr->heading);
  } else {
    std::vector<Eigen::Vector3d> left_points;
    std::vector<Eigen::Vector3d> right_points;
    // 给主车道左右点数组赋值，用来计算heading
    if (left_lane_line->vehicle_points.front().x() >=
        track_zebracrossing_ptr->center_point.x()) {
      left_points.emplace_back(left_lane_line->vehicle_points[0]);
      left_points.emplace_back(left_lane_line->vehicle_points[1]);
    } else if (left_lane_line->vehicle_points.back().x() <
               track_zebracrossing_ptr->center_point.x()) {
      left_points.emplace_back(left_lane_line->vehicle_points[left_size - 2]);
      left_points.emplace_back(left_lane_line->vehicle_points[left_size - 1]);
    } else {
      int left_idnex = 0;
      for (; left_idnex < left_size; left_idnex++) {
        if (left_lane_line->vehicle_points[left_idnex].x() >
            track_zebracrossing_ptr->center_point.x()) {
          break;
        }
      }
      left_points.emplace_back(left_lane_line->vehicle_points[left_idnex - 1]);
      left_points.emplace_back(left_lane_line->vehicle_points[left_idnex]);
    }

    if (right_lane_line->vehicle_points.front().x() >=
        track_zebracrossing_ptr->center_point.x()) {
      right_points.emplace_back(right_lane_line->vehicle_points[0]);
      right_points.emplace_back(right_lane_line->vehicle_points[1]);
    } else if (right_lane_line->vehicle_points.back().x() <
               track_zebracrossing_ptr->center_point.x()) {
      right_points.emplace_back(
          right_lane_line->vehicle_points[right_size - 2]);
      right_points.emplace_back(
          right_lane_line->vehicle_points[right_size - 1]);
    } else {
      int right_idnex = 0;
      for (; right_idnex < right_size; right_idnex++) {
        if (right_lane_line->vehicle_points[right_idnex].x() >
            track_zebracrossing_ptr->center_point.x()) {
          break;
        }
      }
      right_points.emplace_back(
          right_lane_line->vehicle_points[right_idnex - 1]);
      right_points.emplace_back(right_lane_line->vehicle_points[right_idnex]);
    }
    auto main_lane_heading =
        CommonUtil::CalMainLaneHeading(left_points, right_points);
    track_zebracrossing_ptr->heading =
        fabs(main_lane_heading - track_zebracrossing_ptr->heading) < 1.047
            ? main_lane_heading
            : 0.5 * (measurement->heading + track_zebracrossing_ptr->heading);
    // HLOG_ERROR << "更新斑马线heading" << track_zebracrossing_ptr->heading;
  }
}

void ZebraCrossingPointFilter::UpdateVehiclePoints() {
  auto& track_zebracrossing_ptr = target_ref_->GetTrackedObject();
  Eigen::Vector3d l = {-track_zebracrossing_ptr->length / 2 *
                           sin(track_zebracrossing_ptr->heading),
                       track_zebracrossing_ptr->length / 2 *
                           cos(track_zebracrossing_ptr->heading),
                       0};
  Eigen::Vector3d w = {track_zebracrossing_ptr->width / 2 *
                           cos(track_zebracrossing_ptr->heading),
                       track_zebracrossing_ptr->width / 2 *
                           sin(track_zebracrossing_ptr->heading),
                       0};
  Eigen::Vector3d point_0 = track_zebracrossing_ptr->center_point + l + w;
  Eigen::Vector3d point_1 = track_zebracrossing_ptr->center_point + l - w;
  Eigen::Vector3d point_2 = track_zebracrossing_ptr->center_point - l - w;
  Eigen::Vector3d point_3 = track_zebracrossing_ptr->center_point - l + w;
  std::vector<Eigen::Vector3d> new_vehicle_points = {point_0, point_1, point_2,
                                                     point_3};
  track_zebracrossing_ptr->vehicle_points = new_vehicle_points;
}
}  // namespace lm
}  // namespace mp
}  // namespace hozon
