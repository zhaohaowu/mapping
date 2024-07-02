// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Hozon Inc.
// @file: lane_utils.h
// @brief: lane utils

#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <algorithm>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "Eigen/Eigen"
#include "base/utils/log.h"
#include "modules/local_mapping/base/scene/laneline.h"
#include "modules/local_mapping/base/scene/localmap.h"
#include "modules/local_mapping/lib/datalogger/pose_manager.h"
#include "modules/local_mapping/utils/common.h"

namespace hozon {
namespace mp {
namespace lm {

const std::map<int, LaneLinePosition> kIndex2LanePosMap = {
    {-99, LaneLinePosition::OTHER},        {1, LaneLinePosition::EGO_RIGHT},
    {2, LaneLinePosition::ADJACENT_RIGHT}, {3, LaneLinePosition::THIRD_RIGHT},
    {4, LaneLinePosition::FOURTH_RIGHT},   {-1, LaneLinePosition::EGO_LEFT},
    {-2, LaneLinePosition::ADJACENT_LEFT}, {-3, LaneLinePosition::THIRD_LEFT},
    {-4, LaneLinePosition::FOURTH_LEFT}};

static const int max_poly_order = 3;

template <typename T1, typename T2, typename T3>
void LaneLinePolynomialEval(const std::vector<T1>& coeffs, const T2 input,
                            T3* result) {
  *result = 0.F;
  // curve coefficient ordered from low to high
  T2 tmp_input = 1.0;
  for (auto& coeff : coeffs) {
    *result += (coeff * tmp_input);
    tmp_input *= input;
  }
}

template <typename T1, typename T2, typename T3>
void LaneLinePolynomialFirstOrderEval(const std::vector<T1>& coeffs,
                                      const T2 input, T3* result) {
  // first order differential
  *result = 0.0f;
  T2 value = 1.0;
  for (int i = 1; i < coeffs.size(); ++i) {
    *result += i * coeffs[i] * value;
    value *= input;
  }
}

bool TransformLaneLineCurveInNovatelPolyfit(
    const LaneLineCurve& curve, const Eigen::Affine3d& transform_matrix,
    LaneLineCurve* transform_line_curve);

bool TransformLaneLineCurveInNovatel(const LaneLineCurve& curve,
                                     const Eigen::Affine3d& transform_matrix,
                                     LaneLineCurve* transform_line_curve);

// fit polynomial function with QR decomposition (using Eigen 3)
template <typename Dtype>
bool PolyFit(const std::vector<Eigen::Matrix<Dtype, 2, 1>>& pos_vec,
             const int& order,
             Eigen::Matrix<Dtype, max_poly_order + 1, 1>* coeff,
             const bool& is_x_axis = true) {
  if (coeff == nullptr) {
    HLOG_ERROR << "The coefficient pointer is NULL.";
    return false;
  }

  if (order > max_poly_order) {
    HLOG_ERROR << "The order of polynomial must be smaller than "
               << max_poly_order;
    return false;
  }

  int n = static_cast<int>(pos_vec.size());
  if (n <= order) {
    HLOG_ERROR
        << "The number of points should be larger than the order. #points = "
        << pos_vec.size();
    return false;
  }

  // create data matrix
  Eigen::Matrix<Dtype, Eigen::Dynamic, Eigen::Dynamic> A(n, order + 1);
  Eigen::Matrix<Dtype, Eigen::Dynamic, 1> y(n);
  Eigen::Matrix<Dtype, Eigen::Dynamic, 1> result;
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j <= order; ++j) {
      A(i, j) = static_cast<Dtype>(
          std::pow(is_x_axis ? pos_vec[i].x() : pos_vec[i].y(), j));
    }
    y(i) = is_x_axis ? pos_vec[i].y() : pos_vec[i].x();
  }

  // solve linear least squares
  result = A.householderQr().solve(y);
  assert(result.size() == order + 1);

  for (int j = 0; j <= max_poly_order; ++j) {
    (*coeff)(j) = (j <= order) ? result(j) : static_cast<Dtype>(0);
  }

  return true;
}

// bool IsCurb(const base::LaneLinePosition &line_position);

bool IsForkOrConverge(const LaneLineSceneType& scene_type);

bool IsForkOrConvergePair(const LaneLineSceneType& scene_type1,
                          const LaneLineSceneType& scene_type2);

bool IsForkPair(const LaneLineSceneType& scene_type1,
                const LaneLineSceneType& scene_type2);

bool IsConvergePair(const LaneLineSceneType& scene_type1,
                    const LaneLineSceneType& scene_type2);

std::string GetLaneLineCurveInfo(const LaneLineCurve& laneline_curve);

std::string GetLaneLineInfo(const LaneLineConstPtr& lane_line);
std::string GetLaneLineInfo(const LaneLine& lane_line);

void GetRefValueForLineCurve(const LaneLineCurve& curve, float* d,
                             float ref_min = 10.0, float ref_length = 20.0,
                             int sample_num = 10);

void SetLanePosition(
    const float& ref_min, const float& ref_length, const int& sample_num,
    const std::vector<LaneLinePtr>& lane_lines,
    const std::vector<bool>& far_lanes_flag,
    std::unordered_map<int, std::tuple<float, float>>* lane_d_map,
    std::unordered_map<int, LaneLinePosition>* lane_result_pos_map,
    bool use_far);

void SetLanePosition(const float& ref_min, const float& ref_length,
                     const int& sample_num,
                     const std::vector<LaneLinePtr>& lane_lines);
void SetLanePosition(const float& ref_min, const float& ref_length,
                     const int& sample_num,
                     const std::vector<RoadEdgePtr>& roadedges);
void GetCurvartureRadius(const std::vector<float>& coeffs, float input,
                         float* result);

void GetTrangentialAngle(const std::vector<float>& coeffs, float input,
                         float* result);

bool CalLaneLineDistance(const LaneLineCurve& curve1,
                         const LaneLineCurve& curve2, float* distance,
                         float* var, float sample_interval = 0.5,
                         float care_start = -10, float care_end = 100,
                         bool hard_interval = false);

bool CalLaneLineDistance(const LaneLineCurve& curve1,
                         const LaneLineCurve& curve2, float* distance,
                         float sample_interval = 0.5, float care_start = -10,
                         float care_end = 100, bool hard_interval = false);

bool GetSamplePoint(const LaneLineCurve& curve1, const LaneLineCurve& curve2,
                    std::vector<float>* sample_values,
                    float sample_interval = 0.5, float care_start = -10,
                    float care_end = 100, bool hard_interval = false);

bool GetMiddleLineCurve(const LaneLineCurve& curve1,
                        const LaneLineCurve& curve2,
                        LaneLineCurve* middle_curve);

float GetLength(const LaneLineCurve& curve);

float GetLength(const std::vector<Eigen::Vector3d>& point_sets);

float GetDistPointLane(const Eigen::Vector3d& x, const Eigen::Vector3d& x1,
                       const Eigen::Vector3d& x2);

float GetDistBetweenTwoLane(const std::vector<Eigen::Vector3d>& point_set1,
                            const std::vector<Eigen::Vector3d>& point_set2);
std::pair<double, double> GetOverLayLengthBetweenTwoLane(
    const std::vector<Eigen::Vector3d>& point_set1,
    const std::vector<Eigen::Vector3d>& point_set2, float thresh_length);
float GetLengthRatioBetweenTwoLane(const LaneLineConstPtr& curve1,
                                   const LaneLineConstPtr& curve2);
void TransMeasurementVehicle2Local(
    std::vector<LaneLinePtr>* measurement_lanelines_ptr);
void TransMeasurementVehicle2Local(
    std::vector<RoadEdgePtr>* measurement_roadedges_ptr);

void TransTrackerLocal2Vehicle(const LaneLinesPtr& tracked_lanes);
void TransTrackerLocal2Vehicle(const RoadEdgesPtr& tracked_roadedges);

bool LaneLineIntersection(const std::vector<Eigen::Vector3d>& point_set1,
                          const std::vector<Eigen::Vector3d>& point_set2,
                          Eigen::Vector3d* intersect_pt);

void DeepCopy(const LocalMapFramePtr& src_data,
              LocalMapFramePtr& dst_data);  // NOLINT

std::pair<float, float> GetEndY(const std::vector<Eigen::Vector3d>& point_sets);
// 函数模版
template <typename lineType>
float GetOverLayRatioBetweenTwoLane(const std::shared_ptr<lineType>& curve1,
                                    const std::shared_ptr<lineType>& curve2) {
  auto length1 = GetLength(curve1->vehicle_points);
  auto length2 = GetLength(curve2->vehicle_points);

  auto overlay_start = std::max(curve1->vehicle_points.front().x(),
                                curve2->vehicle_points.front().x());
  auto overlay_end = std::min(curve1->vehicle_points.back().x(),
                              curve2->vehicle_points.back().x());
  auto overlay_length =
      overlay_end - overlay_start > 0 ? overlay_end - overlay_start : 0;

  return overlay_length / std::min(length1, length2);
}

// 函数模版
template <typename lineType>
float GetOverLayYRatioBetweenTwoLane(const std::shared_ptr<lineType>& curve1,
                                     const std::shared_ptr<lineType>& curve2) {
  auto lane_y_1 = GetEndY(curve1->vehicle_points);
  auto lane_y_2 = GetEndY(curve2->vehicle_points);
  auto length1 = lane_y_1.second - lane_y_1.first;
  auto length2 = lane_y_2.second - lane_y_2.first;

  auto overlay_start = std::max(lane_y_1.first, lane_y_2.first);
  auto overlay_end = std::min(lane_y_1.second, lane_y_2.second);
  auto overlay_length =
      overlay_end - overlay_start > 0 ? overlay_end - overlay_start : 0;

  return overlay_length / std::min(length1, length2);
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
