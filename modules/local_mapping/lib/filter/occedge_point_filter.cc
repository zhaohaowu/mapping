// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Li guo (liguo03@hozon.com)
// @file: occedge_point_filter.cc
// @brief: kalman update points

#include "modules/local_mapping/lib/filter/occedge_point_filter.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <numeric>
#include <random>
#include <sstream>
#include <utility>

#include "modules/local_mapping/lib/datalogger/pose_manager.h"
#include "modules/util/include/util/mapping_log.h"

namespace hozon {
namespace mp {
namespace lm {
bool OccEdgePointFilter::Init(const FilterInitOption& init_options) {
  measures_.set_capacity(4);
  total_observe_points_.reserve(200);
  // set range for each bin
  ref_bin_range_table_.reserve(150);
  count_for_each_bin_.reserve(150);
  std::pair<float, float> range_for_bin;

  // for near blind spot sample range
  float bin_start = min_near_distance_;
  float bin_end = 0.0F;
  while (bin_start < middle_near_distance_) {
    bin_end = bin_start + near_sample_bin_width_;
    range_for_bin.first = bin_start;
    range_for_bin.second = bin_end;
    ref_bin_range_table_.push_back(range_for_bin);
    bin_start = bin_end;
  }

  // for far sample range
  bin_start = middle_near_distance_;
  while (bin_start < max_far_distance_) {
    bin_end = bin_start + far_sample_bin_width_;
    range_for_bin.first = bin_start;
    range_for_bin.second = bin_end;
    ref_bin_range_table_.push_back(range_for_bin);
    bin_start = bin_end;
  }
  return true;
}

bool OccEdgePointFilter::IsAbnormalPose(
    const Eigen::Affine3d& novatel2world_pose) {
  Eigen::Affine3d delta_pose = PoseManager::Instance()->GetDeltaPose();
  // yaw(z) picth(y) roll(x)
  Eigen::Vector3d euler_angles = delta_pose.linear().eulerAngles(2, 1, 0);
  float yaw =
      std::min(std::min(fabs(euler_angles[0]), fabs(euler_angles[0] - M_PI)),
               fabs(euler_angles[0] + M_PI));
  float pitch =
      std::min(std::min(fabs(euler_angles[1]), fabs(euler_angles[1] - M_PI)),
               fabs(euler_angles[1] + M_PI));
  float roll =
      std::min(std::min(fabs(euler_angles[2]), fabs(euler_angles[2] - M_PI)),
               fabs(euler_angles[2] + M_PI));

  // todo 配置化参数
  // bool is_angle_big_change =
  //     (yaw * 180 / M_PI >
  //      lane_point_filter_param_.max_delta_change_for_pose_angle()) ||
  //     (pitch * 180 / M_PI >
  //      lane_point_filter_param_.max_delta_change_for_pose_angle()) ||
  //     (roll * 180 / M_PI >
  //      lane_point_filter_param_.max_delta_change_for_pose_angle());
  bool is_angle_big_change = false;
  float lateral_postion_error = delta_pose.translation()[1];
  // bool is_position_big_change =
  //     fabs(lateral_postion_error) >
  //     lane_point_filter_param_.max_delta_change_for_pose_position();
  bool is_position_big_change = false;

  bool is_pose_error = is_angle_big_change || is_position_big_change;
  if (is_pose_error) {
    HLOG_ERROR << " [OccEdgePointFilter]: " << " IsAbnormalPose: "
               << " is_angle_big_change, " << is_angle_big_change
               << " is_position_big_change, " << is_position_big_change
               << " Current pose error, yaw:" << yaw * 180 / M_PI
               << ", pitch: " << pitch * 180 / M_PI
               << ", roll: " << roll * 180 / M_PI
               << ", lateral_position_error: " << lateral_postion_error;
  }
  return is_pose_error;
}

int OccEdgePointFilter::LocateBinIndexByLinearTime(
    const float& locate_value) const {
  int find_index = -1;
  int ref_bin_size = ref_bin_range_table_.size();
  if (ref_bin_size <= 0) {
    return -1;
  }

  if (locate_value < min_near_distance_ || locate_value > max_far_distance_) {
    return -1;
  }

  if (locate_value > middle_near_distance_) {
    find_index = std::floor((locate_value - middle_near_distance_) /
                            far_sample_bin_width_) +
                 std::floor((middle_near_distance_ - min_near_distance_) /
                            near_sample_bin_width_);
  } else {
    find_index = std::floor((locate_value - min_near_distance_) /
                            near_sample_bin_width_);
  }

  return find_index;
}

void OccEdgePointFilter::UpdateHistoryPoints() {
  for (auto& measurement : measures_) {
    for (auto& lane_point : measurement->vehicle_points) {
      auto new_pt = POSE_MANAGER->GetDeltaPose() * lane_point;
      lane_point = new_pt;
    }
  }
}

void OccEdgePointFilter::UpdateObservePoints() {
  total_observe_points_.clear();
  count_for_each_bin_.clear();
  int ref_bin_size = ref_bin_range_table_.size();
  count_for_each_bin_.resize(ref_bin_size);
  count_for_each_bin_.assign(ref_bin_size, 0);

  const LaneLineCurve& last_track_curve =
      target_ref_->GetConstTrackedObject()->vehicle_curve;
  CurveFitter curve_fitter(last_track_curve);
  for (int i = measures_.size() - 1; i >= 0; i--) {
    auto observe_points = measures_[i]->vehicle_points;
    if (i != measures_.size() - 1) {
      std::random_device rd;
      std::mt19937 g(rd());
      std::shuffle(observe_points.begin(), observe_points.end(), g);
    }
    for (const auto& lane_point : observe_points) {
      int index = LocateBinIndexByLinearTime(lane_point.x());
      if (index < 0 || index >= count_for_each_bin_.size()) {
        continue;
      }
      // for not newest measurement, just compensate points according to ref_bin
      // max_count
      if (count_for_each_bin_[index] >= min_point_num_each_bin_) {
        continue;
      }
      // filter big abormal history point acccording to current laneline
      // measurement
      double eval_value = curve_fitter.evalueValue(lane_point.x());
      float threshold_value = 1.5F;
      if (fabs(eval_value - lane_point.y()) > threshold_value) {
        continue;
      }

      total_observe_points_.push_back(lane_point);
      ++count_for_each_bin_[index];
    }
    // sort total_observe_points_ before curvefitting
    sort(total_observe_points_.begin(), total_observe_points_.end(),
         [](const auto& a, const auto& b) -> bool { return a.x() < b.x(); });
  }
}

void OccEdgePointFilter::UpdateWithMeasurement(
    const FilterOption& filter_options, const OccEdgePtr& measurement) {
  // PERF_FUNCTION();
  // PERF_BLOCK_START();
  // 如果前后帧姿态异常，则不做任何处理。
  if (IsAbnormalPose(PoseManager::Instance()->GetCurrentPose())) {
    return;
  }
  OccEdgePtr point_measure = std::make_shared<OccEdge>(*measurement);
  measures_.push_back(point_measure);
  UpdateHistoryPoints();
  UpdateObservePoints();
  UpdateResult(true);
  // PERF_BLOCK_END("UpdateResult Used Time");
  return;
}

void OccEdgePointFilter::UpdateWithoutMeasurement(
    const FilterOption& filter_options) {
  // 没有检测时， 保持上一帧结果进行输出。
  // PredictStage();
  // LANE_PREDICT_POINT_DEBUG_INFO(X_);
  // UpdateResult(false);
  return;
}

void OccEdgePointFilter::UpdateResult(bool match_flag) {
  auto tracked_lane = target_ref_->GetTrackedObject();
  // 对total_observe_points_进行拟合
  if (!curve_fit_.RandomPointsPolyFitProcess(total_observe_points_)) {
    HLOG_DEBUG << "track id : " << target_ref_->Id()
               << ", points_size: " << total_observe_points_.size()
               << " curve_fit error !!!";
    return;
  }
  // 拿车身坐标系下的点进行三次方程拟合
  auto& track_polynomial = tracked_lane->vehicle_curve;

  track_polynomial.min = curve_fit_.x_min;
  track_polynomial.max = curve_fit_.x_max;
  track_polynomial.coeffs.resize(4);
  track_polynomial.coeffs[0] = curve_fit_.params[0];
  track_polynomial.coeffs[1] = curve_fit_.params[1];
  track_polynomial.coeffs[2] = curve_fit_.params[2];
  track_polynomial.coeffs[3] = curve_fit_.params[3];
}

void OccEdgePointFilter::Reset() { return; }

}  // namespace lm
}  // namespace mp
}  // namespace hozon
