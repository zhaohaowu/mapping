// Copyright 2020 Hozon Inc. All Rights Reserved.
// @file: curve_fitter.cc
// @brief: vehicle curve_fitter method

#include "modules/local_mapping/utils/curve_fitter.h"

#include <memory>
#include <random>
#include <vector>

namespace hozon {
namespace mp {
namespace lm {
CurveFitter::CurveFitter() {
  params.reserve(order_ + 1);
  A_.resize(40, order_ + 1);
  y_.resize(40);
  result_.resize(order_ + 1);
}
CurveFitter::CurveFitter(int order) : order_(order) {
  params.reserve(order_ + 1);
  A_.resize(40, order_ + 1);
  y_.resize(40);
  result_.resize(order_ + 1);
}
CurveFitter::CurveFitter(int order, int pt_num) : order_(order) {
  params.reserve(order_ + 1);
  A_.resize(pt_num, order_ + 1);
  y_.resize(pt_num);
  result_.resize(order_ + 1);
}

CurveFitter::CurveFitter(const LaneLineCurve& curve)
    : params(curve.coeffs), x_min(curve.min), x_max(curve.max) {}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
