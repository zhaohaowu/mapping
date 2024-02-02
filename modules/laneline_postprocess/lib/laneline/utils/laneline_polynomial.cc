// Copyright 2020 Hozon Inc. All Rights Reserved.
// File Name: laneline_polynomial.cc
// Author: YangYuhao (yangyuhao@hozon.com)
// Descriptions: polynomial struct

#include "modules/laneline_postprocess/lib/laneline/utils/laneline_polynomial.h"

namespace hozon {
namespace mp {
namespace environment {

float LaneLinePolynomial::eval(const float& x) const {
  float s = params[0];
  float t = x;
  for (int i = 1; i < static_cast<int>(params.size()); i++) {
    s += params[i] * t;
    t *= x;
  }
  return s;
}

}  // namespace environment
}  // namespace mp
}  // namespace hozon
