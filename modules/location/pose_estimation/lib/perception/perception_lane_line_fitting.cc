/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： perception_lane_line_fitting.cc
 *   author     ： ouyanghailin
 *   date       ： 2024.03
 ******************************************************************************/

#include "modules/location/pose_estimation/lib/perception/perception_lane_line_fitting.h"

#include <Eigen/Dense>
#include <Eigen/QR>

#include "modules/util/include/util/mapping_log.h"

namespace hozon {
namespace mp {
namespace loc {

void PerceptionLaneLineFitting::Fitting(
    const std::vector<Eigen::Matrix<double, 3, 1>>& perception_points,
    int order, std::vector<double>* coeffs) {
  if (coeffs == nullptr) {
    return;
  }
  auto size = perception_points.size();
  if (order <= 0 || size < order) {
    return;
  }
  HLOG_ERROR << "size: " << size;
  Eigen::MatrixXd T(size, order + 1);
  Eigen::VectorXd V = Eigen::VectorXd::Map(&perception_points.front().y(),
                                           perception_points.size());
  for (size_t i = 0; i < perception_points.size(); ++i) {
    for (size_t j = 0; j < order + 1; ++j) {
      T(i, j) = pow(perception_points[i].x(), j);
    }
  }
  Eigen::VectorXd result = T.householderQr().solve(V);
  coeffs->resize(order + 1);
  for (int k = 0; k < order + 1; k++) {
    if (!std::isfinite(result[k])) {
      return;
    }
    (*coeffs)[k] = result[k];
  }
}

}  // namespace loc
}  // namespace mp
}  // namespace hozon
