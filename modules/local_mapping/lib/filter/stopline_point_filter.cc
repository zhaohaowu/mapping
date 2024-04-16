// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Li guo (liguo03@hozon.com)
// @file: lane_point_filter.cc
// @brief: filter 3d points

#include "modules/local_mapping/lib/filter/stopline_point_filter.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <iostream>
#include <map>
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

bool StopLinePointFilter::Init() {
  history_measure_stoplines_.set_capacity(history_measure_size_);
  history_measure_stoplines_.clear();
  return true;
}

bool StopLinePointFilter::IsAbnormalPose(
    const Eigen::Affine3d& novatel2world_pose) {
  return true;
}

bool StopLinePointFilter::CheckCenterPointStableMeasureState() {
  // 观测数据量小于10帧时，直接认为状态不稳定
  if (history_measure_stoplines_.size() < history_measure_size_) {
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

  for (const auto& measure_stopline_ptr : history_measure_stoplines_) {
    measurement_xs.push_back(measure_stopline_ptr->center_point.x());
    measurement_ys.push_back(measure_stopline_ptr->center_point.y());
  }

  float max_x = *max_element(measurement_xs.begin(), measurement_xs.end());
  float min_x = *min_element(measurement_xs.begin(), measurement_xs.end());
  float max_y = *max_element(measurement_ys.begin(), measurement_ys.end());
  float min_y = *min_element(measurement_ys.begin(), measurement_ys.end());

  if (max_x - min_x < 0.3 && max_y - min_y < 0.1) {
    auto& track_stopline = target_ref_->GetTrackedObject();
    float sum_x = 0.0;
    float sum_y = 0.0;
    for (const auto& measurement_x : measurement_xs) {
      sum_x += measurement_x;
    }
    for (const auto& measurement_y : measurement_ys) {
      sum_y += measurement_y;
    }
    track_stopline->center_point =
        Eigen::Vector3d{sum_x / static_cast<float>(history_measure_size_),
                        sum_y / static_cast<float>(history_measure_size_), 0};
    center_point_is_stable_state_ = true;
    return true;
  }

  return false;
}

bool StopLinePointFilter::CheckLengthStableMeasureState() {
  // 观测数据量小于10帧时，直接认为状态不稳定
  if (history_measure_stoplines_.size() < history_measure_size_) {
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

  for (const auto& measure_stopline_ptr : history_measure_stoplines_) {
    measurement_lengths.push_back(measure_stopline_ptr->length);
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
    auto& track_stopline = target_ref_->GetTrackedObject();
    track_stopline->length =
        sum_length / static_cast<float>(history_measure_size_);
    length_is_stable_state_ = true;
    return true;
  }

  return false;
}

bool StopLinePointFilter::CheckHeadingStableMeasureState() {
  // 观测数据量小于10帧时，直接认为状态不稳定
  if (history_measure_stoplines_.size() < history_measure_size_) {
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

  for (const auto& measure_stopline_ptr : history_measure_stoplines_) {
    measurement_headings.push_back(measure_stopline_ptr->heading);
  }

  float max_heading =
      *max_element(measurement_headings.begin(), measurement_headings.end());
  float min_heading =
      *min_element(measurement_headings.begin(), measurement_headings.end());
  // HLOG_ERROR << "停止线max_heading - min_heading: "
  //            << max_heading - min_heading;
  if (max_heading - min_heading < 0.17) {
    float sum_heading = 0.0;
    for (const auto& measurement_heading : measurement_headings) {
      sum_heading += measurement_heading;
    }
    auto& track_stopline = target_ref_->GetTrackedObject();
    track_stopline->heading =
        sum_heading / static_cast<float>(history_measure_size_);
    heading_is_stable_state_ = true;
    return true;
  }

  return false;
}

std::map<std::string, LaneLinePtr> StopLinePointFilter::GetAllEgolines() {
  std::map<std::string, LaneLinePtr> all_egolines;
  LocalMapFramePtr localmap_frame = MAP_MANAGER->GetLocalMap();
  for (const auto& laneline_ptr : localmap_frame->lane_lines_ptr->lanelines) {
    if (!laneline_ptr->after_intersection) {
      if (laneline_ptr->position == LaneLinePosition::EGO_LEFT) {
        all_egolines["before_left"] = laneline_ptr;
      } else if (laneline_ptr->position == LaneLinePosition::EGO_RIGHT) {
        all_egolines["before_right"] = laneline_ptr;
      }
    } else if (laneline_ptr->after_intersection) {
      if (laneline_ptr->position == LaneLinePosition::EGO_LEFT) {
        all_egolines["after_left"] = laneline_ptr;
      } else if (laneline_ptr->position == LaneLinePosition::EGO_RIGHT) {
        all_egolines["after_right"] = laneline_ptr;
      }
    }
  }

  return all_egolines;
}

std::map<std::string, LaneLinePtr> StopLinePointFilter::SelectEgolines(
    const std::map<std::string, LaneLinePtr>& all_ego_lines) {
  auto& track_stopline = target_ref_->GetTrackedObject();
  double before_dis = FLT_MAX;
  if ((all_ego_lines.count("before_left") == 0U) &&
      (all_ego_lines.count("before_right") != 0U)) {
    before_dis =
        fabs(track_stopline->center_point.x() -
             (all_ego_lines.at("before_right")->vehicle_points.back().x() +
              all_ego_lines.at("before_right")->vehicle_points.front().x()) /
                 2);
  } else if ((all_ego_lines.count("before_left") != 0U) &&
             (all_ego_lines.count("before_right") == 0U)) {
    before_dis =
        fabs(track_stopline->center_point.x() -
             (all_ego_lines.at("before_left")->vehicle_points.back().x() +
              all_ego_lines.at("before_left")->vehicle_points.front().x()) /
                 2);
  } else if ((all_ego_lines.count("before_left") != 0U) &&
             (all_ego_lines.count("before_right") != 0U)) {
    before_dis =
        fabs(track_stopline->center_point.x() -
             (all_ego_lines.at("before_left")->vehicle_points.back().x() +
              all_ego_lines.at("before_left")->vehicle_points.front().x() +
              all_ego_lines.at("before_right")->vehicle_points.back().x() +
              all_ego_lines.at("before_left")->vehicle_points.front().x()) /
                 4);
  } else {
    before_dis = FLT_MAX;
  }

  double after_dis = FLT_MAX;
  if ((all_ego_lines.count("after_left") == 0U) &&
      (all_ego_lines.count("after_right") != 0U)) {
    after_dis =
        fabs(track_stopline->center_point.x() -
             (all_ego_lines.at("after_right")->vehicle_points.front().x() +
              all_ego_lines.at("after_right")->vehicle_points.back().x()) /
                 2);
  } else if ((all_ego_lines.count("after_left") != 0U) &&
             (all_ego_lines.count("after_right") == 0U)) {
    after_dis =
        fabs(track_stopline->center_point.x() -
             (all_ego_lines.at("after_left")->vehicle_points.front().x() +
              all_ego_lines.at("after_left")->vehicle_points.back().x()) /
                 2);
  } else if ((all_ego_lines.count("after_left") != 0U) &&
             (all_ego_lines.count("after_right") != 0U)) {
    after_dis =
        fabs(track_stopline->center_point.x() -
             (all_ego_lines.at("after_left")->vehicle_points.front().x() +
              all_ego_lines.at("after_right")->vehicle_points.front().x() +
              all_ego_lines.at("after_left")->vehicle_points.back().x() +
              all_ego_lines.at("after_right")->vehicle_points.back().x()) /
                 4);
  } else {
    after_dis = FLT_MAX;
  }
  std::map<std::string, LaneLinePtr> select_egolines;
  select_egolines.clear();
  if (before_dis <= after_dis && all_ego_lines.count("before_left") != 0U &&
      all_ego_lines.count("before_right") != 0U) {
    select_egolines["ego_left"] = all_ego_lines.at("before_left");
    select_egolines["ego_right"] = all_ego_lines.at("before_right");
  } else if (before_dis > after_dis &&
             all_ego_lines.count("after_left") != 0U &&
             all_ego_lines.count("after_right") != 0U) {
    select_egolines["ego_left"] = all_ego_lines.at("after_left");
    select_egolines["ego_right"] = all_ego_lines.at("after_right");
  }

  return select_egolines;
}

void StopLinePointFilter::UpdateWithMeasurement(
    const StopLinePtr& measurement) {
  history_measure_stoplines_.push_back(measurement);
  // 中心点滤波
  if (!CheckCenterPointStableMeasureState()) {
    UpdateCenterPoint(measurement);
  }
  // length滤波
  if (!CheckLengthStableMeasureState()) {
    UpdateLength(measurement);
  }
  // 朝向滤波
  if (!CheckHeadingStableMeasureState()) {
    UpdateHeading(measurement);
  }
  // 优化中心点的位置（避免停止线出现在斑马线内）
  // OptiCenterPoint();
  // 跟新最终车系下的左右端点
  if (!length_is_stable_state_ || !heading_is_stable_state_ ||
      !center_point_is_stable_state_) {
    UpdateVehiclePoints();
  }
}

void StopLinePointFilter::UpdateWithoutMeasurement() {
  // 未匹配时， 保持上一帧跟踪结果。
}

void StopLinePointFilter::Reset() { history_measure_stoplines_.clear(); }

void StopLinePointFilter::UpdateCenterPoint(const StopLinePtr& measurement) {
  auto& track_stopline = target_ref_->GetTrackedObject();
  track_stopline->center_point =
      0.5 * (measurement->center_point + track_stopline->center_point);
}

bool StopLinePointFilter::IsPointInsideBox(const Point& point,
                                           const ZebraCrossing& box) {
  // 将点坐标旋转回原始坐标系
  auto xMin = box.center_point.x() - box.length / 2;
  auto xMax = box.center_point.x() + box.length / 2;
  auto yMin = box.center_point.y() - box.width / 2;
  auto yMax = box.center_point.y() + box.width / 2;
  double rotatedX = (point.x - xMin) * cos(-box.heading) -
                    (point.y - xMin) * sin(-box.heading);
  double rotatedY = (point.x - yMin) * sin(-box.heading) +
                    (point.y - yMin) * cos(-box.heading);

  // 判断点是否在旋转后的框范围内
  return rotatedX >= 0 && rotatedX <= (xMax - xMin) && rotatedY >= 0 &&
         rotatedY <= (yMax - yMin);
}

// 将点移动到旋转框外
void StopLinePointFilter::MoveOutsideBox(Point* point,
                                         const ZebraCrossing& box) {
  // 将点坐标转换为相对于旋转框中心的坐标
  double relativeX = point->x - box.center_point.x();
  double relativeY = point->y - box.center_point.y();

  // 将点坐标旋转回原始坐标系
  double rotatedX = relativeX * cos(box.heading) - relativeY * sin(box.heading);
  double rotatedY = relativeX * sin(box.heading) + relativeY * cos(box.heading);

  // 将点坐标移动到旋转框外
  if (rotatedX < -box.length / 2.0) {
    rotatedX = -box.length / 2.0;
  } else if (rotatedX > box.length / 2.0) {
    rotatedX = box.length / 2.0;
  }
  if (rotatedY < -box.width / 2.0) {
    rotatedY = -box.width / 2.0;
  } else if (rotatedY > box.width / 2.0) {
    rotatedY = box.width / 2.0;
  }

  // 将点坐标旋转到旋转框坐标系
  double newX = rotatedX * cos(-box.heading) - rotatedY * sin(-box.heading);
  double newY = rotatedX * sin(-box.heading) + rotatedY * cos(-box.heading);

  // 将点坐标转换为绝对坐标
  point->x = newX + box.center_point.x();
  point->y = newY + box.center_point.y();
}

void StopLinePointFilter::OptiCenterPoint() {
  auto& stopline_cp = target_ref_->GetTrackedObject()->center_point;
  Point cp = {stopline_cp.x(), stopline_cp.y()};
  LocalMapFramePtr localmap_frame = MAP_MANAGER->GetLocalMap();
  std::vector<ZebraCrossingPtr> all_zebracrossings =
      localmap_frame->zebra_crossings_ptr->zebra_crossings;

  Eigen::Affine3d T_cur_last_ = PoseManager::Instance()->GetDeltaPose();
  for (auto& zebracrossing_ptr : all_zebracrossings) {
    ZebraCrossing zebracross_data = *zebracrossing_ptr;
    auto& vehicle_points = zebracross_data.vehicle_points;
    for (auto& point : vehicle_points) {
      point = T_cur_last_ * point;
    }

    auto& center_point = zebracross_data.center_point;
    center_point = T_cur_last_ * center_point;

    Eigen::Matrix3d R_C_L = T_cur_last_.rotation();
    auto& heading = zebracross_data.heading;
    Eigen::Matrix3d R_L_S;
    R_L_S << cos(heading), -sin(heading), 0, sin(heading), cos(heading), 0, 0,
        0, 1;
    Eigen::Matrix3d R_C_S = R_C_L * R_L_S;
    heading = atan2(R_C_S(1, 0), R_C_S(0, 0));

    if (IsPointInsideBox(cp, zebracross_data)) {
      MoveOutsideBox(&cp, zebracross_data);
      HLOG_INFO << "find stopline in zebracrossing...";
      break;
    }
  }

  stopline_cp.x() = cp.x;
  stopline_cp.y() = cp.y;
}

void StopLinePointFilter::UpdateLength(const StopLinePtr& measurement) {
  auto& track_stopline = target_ref_->GetTrackedObject();
  track_stopline->length = 0.5 * (measurement->length + track_stopline->length);
}

void StopLinePointFilter::UpdateHeading(const StopLinePtr& measurement) {
  auto& track_stopline = target_ref_->GetTrackedObject();
  auto all_ego_lines = GetAllEgolines();
  auto sel_ego_lines = SelectEgolines(all_ego_lines);
  if (!sel_ego_lines.empty()) {
    int left_size = 0;
    int right_size = 0;
    if (sel_ego_lines.count("ego_left") != 0) {
      left_size = sel_ego_lines.at("ego_left")->vehicle_points.size();
    }
    if (sel_ego_lines.count("ego_right") != 0) {
      right_size = sel_ego_lines.at("ego_right")->vehicle_points.size();
    }
    if (left_size < 2 || right_size < 2) {
      track_stopline->heading =
          0.5 * measurement->heading + 0.5 * track_stopline->heading;
    } else {
      std::vector<Eigen::Vector3d> left_points;
      std::vector<Eigen::Vector3d> right_points;
      if (sel_ego_lines.at("ego_left")->vehicle_points.front().x() >=
          track_stopline->center_point.x()) {
        left_points.emplace_back(
            sel_ego_lines.at("ego_left")->vehicle_points[0]);
        left_points.emplace_back(
            sel_ego_lines.at("ego_left")->vehicle_points[1]);
      } else if (sel_ego_lines.at("ego_left")->vehicle_points.back().x() <
                 track_stopline->center_point.x()) {
        left_points.emplace_back(
            sel_ego_lines.at("ego_left")->vehicle_points[left_size - 2]);
        left_points.emplace_back(
            sel_ego_lines.at("ego_left")->vehicle_points[left_size - 1]);
      } else {
        int left_idnex = 0;
        for (; left_idnex < left_size; left_idnex++) {
          if (sel_ego_lines.at("ego_left")->vehicle_points[left_idnex].x() >
              track_stopline->center_point.x()) {
            break;
          }
        }
        left_points.emplace_back(
            sel_ego_lines.at("ego_left")->vehicle_points[left_idnex - 1]);
        left_points.emplace_back(
            sel_ego_lines.at("ego_left")->vehicle_points[left_idnex]);
      }

      if (sel_ego_lines.at("ego_right")->vehicle_points.front().x() >=
          track_stopline->center_point.x()) {
        right_points.emplace_back(
            sel_ego_lines.at("ego_right")->vehicle_points[0]);
        right_points.emplace_back(
            sel_ego_lines.at("ego_right")->vehicle_points[1]);
      } else if (sel_ego_lines.at("ego_right")->vehicle_points.back().x() <
                 track_stopline->center_point.x()) {
        right_points.emplace_back(
            sel_ego_lines.at("ego_right")->vehicle_points[right_size - 2]);
        right_points.emplace_back(
            sel_ego_lines.at("ego_right")->vehicle_points[right_size - 1]);
      } else {
        int right_idnex = 0;
        for (; right_idnex < right_size; right_idnex++) {
          if (sel_ego_lines.at("ego_right")->vehicle_points[right_idnex].x() >
              track_stopline->center_point.x()) {
            break;
          }
        }
        right_points.emplace_back(
            sel_ego_lines.at("ego_right")->vehicle_points[right_idnex - 1]);
        right_points.emplace_back(
            sel_ego_lines.at("ego_right")->vehicle_points[right_idnex]);
      }
      auto main_lane_heading =
          CommonUtil::CalMainLaneHeading(left_points, right_points) + 1.57;
      track_stopline->heading =
          fabs(main_lane_heading - track_stopline->heading) < 1.047
              ? main_lane_heading
              : 0.5 * (measurement->heading + track_stopline->heading);
    }
  } else {
    track_stopline->heading =
        0.5 * measurement->heading + 0.5 * track_stopline->heading;
  }
}

void StopLinePointFilter::UpdateVehiclePoints() {
  auto& track_stopline = target_ref_->GetTrackedObject();
  Eigen::Vector3d temp_left_point = {
      track_stopline->center_point.x() +
          track_stopline->length / 2.0 * cos(track_stopline->heading),
      track_stopline->center_point.y() +
          track_stopline->length / 2.0 * sin(track_stopline->heading),
      0};
  Eigen::Vector3d temp_right_point_ = {
      track_stopline->center_point.x() -
          track_stopline->length / 2.0 * cos(track_stopline->heading),
      track_stopline->center_point.y() -
          track_stopline->length / 2.0 * sin(track_stopline->heading),
      0};
  track_stopline->left_point = temp_left_point;
  track_stopline->right_point = temp_right_point_;
}
}  // namespace lm
}  // namespace mp
}  // namespace hozon
