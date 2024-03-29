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

bool ZebraCrossingPointFilter::CheckStableMeasureState() {
  if (is_stable_state_) {
    return true;
  }

  if (history_measure_zebracrossings_.size() < history_measure_size_) {
    return false;
  }
  return false;
}

void ZebraCrossingPointFilter::UpdateWithMeasurement(
    const ZebraCrossingPtr& measurement) {
  history_measure_zebracrossings_.push_back(measurement);
  if (!CheckStableMeasureState()) {
    UpdateCenterPoint(measurement);
    UpdateLength(measurement);
    UpdateWidth(measurement);
    UpdateHeading(measurement);
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
      right_points.emplace_back(right_lane_line->vehicle_points[left_size - 2]);
      right_points.emplace_back(right_lane_line->vehicle_points[left_size - 1]);
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
