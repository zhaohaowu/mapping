// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Hozon Inc.
// @file: lane_utils.h
// @brief: lane utils

#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "Eigen/Eigen"
#include "base/utils/log.h"
#include "perception-base/base/measurement/laneline_measurement.h"
#include "perception-base/base/scene/laneline.h"

namespace hozon {
namespace mp {
namespace environment {

using hozon::perception::base::LaneLine;
using hozon::perception::base::LaneLineConstPtr;
using hozon::perception::base::LaneLineCurve;
using hozon::perception::base::LaneLineMeasurementConstPtr;
using hozon::perception::base::LaneLinePoint;
using hozon::perception::base::LaneLinePosition;
using hozon::perception::base::LaneLinePtr;
using hozon::perception::base::LaneLineSceneType;
using hozon::perception::base::Point2DF;
using hozon::perception::base::Point3DF;

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
  *result = 0.f;
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

bool TransformLaneLineCurveInNovatelPolyfit(
    const LaneLineCurve& curve, const Eigen::Affine3d& transform_matrix,
    LaneLineCurve* transform_line_curve);

bool TransformLaneLineCurveInNovatel(const LaneLineCurve& curve,
                                     const Eigen::Affine3d& transform_matrix,
                                     LaneLineCurve* transform_line_curve);

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

float GetLength(const std::vector<LaneLinePoint>& point_sets);

float GetDistPointLane(const Point3DF& x, const Point3DF& x1,
                       const Point3DF& x2);

float GetDistBetweenTwoLane(const std::vector<LaneLinePoint>& point_set1,
                            const std::vector<LaneLinePoint>& point_set2);

float GetLengthRatioBetweenTwoLane(const LaneLineMeasurementConstPtr& curve1,
                                   const LaneLineMeasurementConstPtr& curve2);

float GetOverLayRatioBetweenTwoLane(const LaneLineMeasurementConstPtr& curve1,
                                    const LaneLineMeasurementConstPtr& curve2);

}  // namespace environment
}  // namespace mp
}  // namespace hozon
