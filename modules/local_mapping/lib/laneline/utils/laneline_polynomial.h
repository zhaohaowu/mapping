// Copyright 2020 Hozon Inc. All Rights Reserved.
// File Name: laneline_polynomial.h
// Author: YangYuhao (yangyuhao@hozon.com)
// Descriptions: polynomial struct

#pragma once

#include <float.h>

#include <memory>
#include <string>
#include <vector>

#include "Eigen/Core"

namespace hozon {
namespace mp {
namespace environment {

struct LaneLinePolynomial {
  typedef Eigen::Matrix<float, Eigen::Dynamic, 1> Vector;
  // Two initialization methods are provided
  LaneLinePolynomial() {
    params.reserve(10);
    inliers_indices.reserve(500);
  }
  explicit LaneLinePolynomial(const int &_order) {
    order = _order;
    params.reserve(_order + 1);
    inliers_indices.reserve(500);
  }
  int order{3};
  float min{FLT_MAX};
  float max{0.f};
  float eval(const float &x) const;
  //  Vector eval(const Vector &xs) const;
  // Coefficients are stored in _params in low-to-high degree order
  // which means 1 + 2x + 3x^2 + 4x^3 correspond to _params = [1 2 3 4]
  std::vector<float> params;
  std::vector<int> inliers_indices;
};
typedef std::shared_ptr<LaneLinePolynomial> LaneLinePolynomialPtr;
typedef std::shared_ptr<const LaneLinePolynomial> LaneLinePolynomialConstPtr;
}  // namespace environment
}  // namespace mp
}  // namespace hozon
