/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： eulerangle.h
 *   author     ： lilanxing
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <Eigen/Eigen>

namespace hozon {
namespace mp {
namespace loc {
namespace fc {

Eigen::Matrix<double, 3, 1> Rot2Euler312(
    const Eigen::Matrix<double, 3, 3>& mat) {
  Eigen::Matrix<double, 3, 1> v;
  v(0) = asinf(mat(2, 1));
  v(1) = atan2f(-mat(2, 0), mat(2, 2));
  v(2) = atan2f(-mat(0, 1), mat(1, 1));
  return v;
}

}  // namespace fc
}  // namespace loc
}  // namespace mp
}  // namespace hozon
