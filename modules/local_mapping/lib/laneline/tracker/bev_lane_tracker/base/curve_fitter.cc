// Copyright 2020 Hozon Inc. All Rights Reserved.
// @file: curve_fitter.cc
// @brief: vehicle curve_fitter method

#include "modules/local_mapping/lib/laneline/tracker/bev_lane_tracker/base/curve_fitter.h"

#include <memory>
#include <random>
#include <vector>

namespace hozon {
namespace mp {
namespace environment {

bool CurveFitter::PolyFitProcess(const std::vector<Eigen::Vector2d>& points) {
  int point_num = static_cast<int>(points.size());
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
  params_.clear();
  for (int i = 0; i < point_num; ++i) {
    float x_pow = 1.f;
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
    params_.emplace_back(result(j, 0));
  }
  return true;
}

bool CurveFitter::WeightPolyFitProcess(
    const std::vector<Eigen::Vector2d>& points,
    const std::vector<float>& weights) {
  int point_num = static_cast<int>(points.size());
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
  params_.clear();
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
    params_.emplace_back(result(j, 0));
  }
  return true;
}

}  // namespace environment
}  // namespace mp
}  // namespace hozon
