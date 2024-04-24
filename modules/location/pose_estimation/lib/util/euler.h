/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： eulerangle.h
 *   author     ： ouyanghailin
 *   date       ： 2023.10
 ******************************************************************************/

#pragma once

#include <Eigen/Eigen>

namespace hozon {
namespace mp {
namespace loc {

inline Eigen::Matrix<double, 3, 1> RotToEuler312(
    const Eigen::Matrix<double, 3, 3>& mat);
inline double CalHeading(double heading);

inline Eigen::Matrix<double, 3, 1> RotToEuler312(
    const Eigen::Matrix<double, 3, 3>& mat) {
  Eigen::Matrix<double, 3, 1> v;
  v(0) = asinf(mat(2, 1));
  v(1) = atan2f(-mat(2, 0), mat(2, 2));
  v(2) = atan2f(-mat(0, 1), mat(1, 1));
  return v;
}

inline double CalHeading(double heading) {
  double cal_heading = -heading * M_PI / 180.0 + M_PI / 2;
  if (cal_heading > M_PI) {
    cal_heading -= 2 * M_PI;
  } else if (cal_heading < -M_PI) {
    cal_heading += 2 * M_PI;
  }
  return cal_heading;
}

}  // namespace loc
}  // namespace mp
}  // namespace hozon
