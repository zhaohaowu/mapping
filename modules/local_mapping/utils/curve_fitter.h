// Copyright 2020 Hozon Inc. All Rights Reserved.
// @file: curve_fitter.h
// @brief: vehicle curve_fitter method

#pragma once

#include <cfloat>
#include <cmath>
#include <string>
#include <unordered_set>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/LU"
#include "Eigen/QR"
#include "modules/local_mapping/base/scene/laneline.h"
#include "perception-base/base/utils/log.h"

namespace hozon {
namespace mp {
namespace lm {

class CurveFitter {
 public:
  template <typename DataType, int rows,
            typename std::enable_if<((rows >= 2) && (rows <= 3)), bool>::type =
                0>
  using Vector = Eigen::Matrix<DataType, rows, 1>;
  CurveFitter();
  explicit CurveFitter(int order);
  explicit CurveFitter(int order, int pt_num);
  explicit CurveFitter(const LaneLineCurve& curve);
  template <typename DataType, int rows>
  bool PolyFitProcess(const std::vector<Vector<DataType, rows>>& points);
  template <typename DataType, int rows>
  bool WeightPolyFitProcess(const std::vector<Vector<DataType, rows>>& points,
                            const std::vector<float>& weights);
  template <typename DataType, int rows>
  bool RandomPointsPolyFitProcess(
      const std::vector<Vector<DataType, rows>>& points);
  template <typename T>
  double evalueValue(const T& x) {
    double sum = 0.0;
    double val = 1.0;
    for (auto& param : params) {
      sum += param * val;
      val *= x;
    }
    return sum;
  }

  template <typename T>
  double evalueHeading(const T& x) {
    double sum = 0.0;
    double val = 1.0;
    // 一阶导
    for (int i = 1; i < params.size(); ++i) {
      sum += params[i] * val * i;
      val *= x;
    }
    return sum;
  }

 public:
  std::vector<float> params;
  float x_min = FLT_MAX;
  float x_max = -FLT_MAX;

 private:
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> A_;
  Eigen::Matrix<float, Eigen::Dynamic, 1> y_;
  Eigen::Matrix<float, Eigen::Dynamic, 1> result_;
  int order_ = 3;  // 默认3次多项式拟合
};

template <typename DataType, int rows>
bool CurveFitter::PolyFitProcess(
    const std::vector<Vector<DataType, rows>>& points) {
  int point_num = static_cast<int>(points.size());
  if (point_num < order_ + 1) {
    HLOG_ERROR << "PolyFitProcess The number of points should be larger than "
                  "the order.";
    return false;
  }
  if (point_num > A_.rows()) {
    HLOG_INFO << "PolyFitProcess The number of points should be smaller than "
                 "the A_ size.";
    point_num = static_cast<int>(A_.rows());
  }
  // create data matrix
  auto A = A_.block(0, 0, point_num, order_ + 1);
  auto y = y_.block(0, 0, point_num, 1);
  auto result = result_.block(0, 0, order_ + 1, 1);
  A.setZero();
  y.setZero();
  result.setZero();
  params.clear();

  x_min = FLT_MAX;
  x_max = -FLT_MAX;
  for (int i = 0; i < point_num; ++i) {
    float x_pow = 1.F;
    x_min = x_min < points[i].x() ? x_min : points[i].x();
    x_max = x_max > points[i].x() ? x_max : points[i].x();
    for (int j = 0; j < order_ + 1; ++j) {
      A(i, j) = x_pow;
      x_pow *= points[i].x();
    }
    y(i, 0) = points[i].y();
  }
  result = A.householderQr().solve(y);
  for (int j = 0; j < order_ + 1; ++j) {
    if (std::isnan(result(j, 0))) {
      HLOG_ERROR << "PolyFitProcess result nan.";
      return false;
    }
    params.emplace_back(result(j, 0));
  }
  return true;
}

template <typename DataType, int rows>
bool CurveFitter::WeightPolyFitProcess(
    const std::vector<Vector<DataType, rows>>& points,
    const std::vector<float>& weights) {
  int point_num = static_cast<int>(points.size());
  if (point_num < order_ + 1) {
    HLOG_ERROR << "PolyFitProcess The number of points should be larger than "
                  "the order.";
    return false;
  }
  if (point_num > A_.rows()) {
    HLOG_INFO << "PolyFitProcess The number of points should be smaller than "
                 "the A_ size.";
    point_num = static_cast<int>(A_.rows());
  }
  // create data matrix
  auto A = A_.block(0, 0, point_num, order_ + 1);
  auto y = y_.block(0, 0, point_num, 1);
  auto result = result_.block(0, 0, order_ + 1, 1);
  A.setZero();
  y.setZero();
  result.setZero();
  params.clear();
  x_min = FLT_MAX;
  x_max = -FLT_MAX;
  for (int i = 0; i < point_num; ++i) {
    for (int j = 0; j < order_ + 1; ++j) {
      A(i, j) = std::pow(points[i].x(), j) * weights[i];
    }
    y(i, 0) = points[i].y() * weights[i];
  }
  result = A.householderQr().solve(y);
  for (int j = 0; j < order_ + 1; ++j) {
    if (std::isnan(result(j, 0))) {
      HLOG_ERROR << "PolyFitProcess result nan.";
      return false;
    }
    params.emplace_back(result(j, 0));
  }
  return true;
}

// 不确定点数的曲线拟合函数,优先用确定点数的函数
template <typename DataType, int rows>
bool CurveFitter::RandomPointsPolyFitProcess(
    const std::vector<Vector<DataType, rows>>& points) {
  int point_num = static_cast<int>(points.size());
  A_.resize(point_num, order_ + 1);
  y_.resize(point_num);
  if (point_num < order_ + 1) {
    HLOG_ERROR << "PolyFitProcess The number of points should be larger than "
                  "the order.";
    return false;
  }
  // create data matrix
  auto A = A_.block(0, 0, point_num, order_ + 1);
  auto y = y_.block(0, 0, point_num, 1);
  auto result = result_.block(0, 0, order_ + 1, 1);
  A.setZero();
  y.setZero();
  result.setZero();
  params.clear();

  x_min = FLT_MAX;
  x_max = -FLT_MAX;
  for (int i = 0; i < point_num; ++i) {
    float x_pow = 1.F;
    x_min = x_min < points[i].x() ? x_min : points[i].x();
    x_max = x_max > points[i].x() ? x_max : points[i].x();
    for (int j = 0; j < order_ + 1; ++j) {
      A(i, j) = x_pow;
      x_pow *= points[i].x();
    }
    y(i, 0) = points[i].y();
  }
  result = A.householderQr().solve(y);
  for (int j = 0; j < order_ + 1; ++j) {
    if (std::isnan(result(j, 0))) {
      HLOG_ERROR << "PolyFitProcess result nan.";
      return false;
    }
    params.emplace_back(result(j, 0));
  }
  return true;
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
