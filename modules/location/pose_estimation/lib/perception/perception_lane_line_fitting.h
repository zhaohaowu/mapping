/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： perception_lane_line_fitting.h
 *   author     ： ouyanghailin
 *   date       ： 2024.03
 ******************************************************************************/

#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

namespace hozon {
namespace mp {
namespace loc {

class PerceptionLaneLineFitting {
 public:
  PerceptionLaneLineFitting() {}
  ~PerceptionLaneLineFitting() {}
  void Fitting(
      const std::vector<Eigen::Matrix<double, 3, 1>>& perception_points,
      int order, std::vector<double>* coeffs);
};

// extern PerceptionLaneLineFitting perception_line_fitting;

}  // namespace loc
}  // namespace mp
}  // namespace hozon
