// Copyright 2020 Hozon Inc. All Rights Reserved.
// @file: curve_fitter.h
// @brief: vehicle curve_fitter method

#pragma once

#include <cmath>
#include <string>
#include <unordered_set>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/LU"
#include "Eigen/QR"
#include "perception-base/base/utils/log.h"

namespace hozon {
namespace mp {
namespace environment {

class CurveFitter {
 public:
  CurveFitter() {
    order_ = 3;
    params_.reserve(order_ + 1);
    A_.resize(40, order_ + 1);
    y_.resize(40);
    result_.resize(order_ + 1);
  }
  explicit CurveFitter(int order) : order_(order) {
    params_.reserve(order_ + 1);
    A_.resize(40, order_ + 1);
    y_.resize(40);
    result_.resize(order_ + 1);
  }
  bool PolyFitProcess(const std::vector<Eigen::Vector2d> &points);
  bool WeightPolyFitProcess(const std::vector<Eigen::Vector2d> &points,
                            const std::vector<float> &weights);
  template <typename T>
  double evalueValue(const T &x) {
    double sum = 0.0, val = 1.0;
    for (int i = 0; i < params_.size(); ++i) {
      sum += params_[i] * val;
      val *= x;
    }
    return sum;
  }

 private:
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> A_;
  Eigen::Matrix<float, Eigen::Dynamic, 1> y_;
  Eigen::Matrix<float, Eigen::Dynamic, 1> result_;
  std::vector<float> params_;

  int order_ = 3;  // 默认3次多项式拟合
};

}  // namespace environment
}  // namespace mp
}  // namespace hozon
