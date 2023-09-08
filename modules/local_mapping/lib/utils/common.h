/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: wangjianguo
 *Date: 2023-09-06
 *****************************************************************************/
#pragma once

#include <iostream>
#include <memory>
#include <vector>

#include "Eigen/Dense"
#include "modules/local_mapping/lib/types/common.h"

namespace hozon {
namespace mp {
namespace lm {

class CommonUtil {
 public:
  /**
   * @brief y = c0 + c1 * x + c2 * x^2 + c3 * x^3
   *
   * @param x
   * @param c0
   * @param c1
   * @param c2
   * @param c3
   * @return double result y
   */
  static double f(const double &x, const double &c0, const double &c1,
                  const double &c2, const double &c3) {
    return c0 + c1 * x + c2 * x * x + c3 * x * x * x;
  }

  /**
   * @brief sample points from lane
   *
   * @param lane target lane
   * @param T_V_W translation from vehicle to world
   * @param pts sample points
   * @return
   */
  static void SampleLanePoints(
      std::shared_ptr<const Lane> lane, const Eigen::Matrix4d T_V_W,
      std::shared_ptr<std::vector<Eigen::Vector3d>> pts) {
    double interval = 0.1;
    double x, y, z;
    for (x = lane->x_start_vrf_; x <= lane->x_end_vrf_; x += interval) {
      y = f(x, lane->lane_fit_d_, lane->lane_fit_c_, lane->lane_fit_b_,
            lane->lane_fit_a_);
      z = 0;
      Eigen::Vector4d pt_v(x, y, z, 1);
      Eigen::Vector4d pt_w = T_V_W * pt_v;
      Eigen::Vector3d pt(pt_w(0), pt_w(1), pt_w(2));
      pts->emplace_back(pt);
    }
  }

  /**
   * @brief lane from sample points
   *
   * @param pts target points
   * @param T_V_W translation from vehicle to world
   * @param lane output lane
   * @return
   */
  static void FitLanePoints(
      std::shared_ptr<const std::vector<Eigen::Vector3d>> pts,
      const Eigen::Matrix4d &T_V_W, std::shared_ptr<Lane> lane) {
    size_t n = (*pts).size();
    Eigen::Matrix<double, Eigen::Dynamic, 4> A(n, 4);
    Eigen::VectorXd x(n);
    Eigen::VectorXd b(n);
    for (size_t i = 0; i < n; i++) {
      double xi = (*pts)[i][0];
      double yi = (*pts)[i][1];
      A(i, 0) = xi * xi * xi;
      A(i, 1) = xi * xi;
      A(i, 2) = xi;
      A(i, 3) = 1.0;
      b[i] = yi;
    }
    x = (A.transpose() * A).inverse() * A.transpose() * b;
    lane->lane_fit_a_ = x[0];
    lane->lane_fit_b_ = x[1];
    lane->lane_fit_c_ = x[2];
    lane->lane_fit_d_ = x[3];
    lane->x_start_vrf_ = (*pts)[0][0];
    lane->x_end_vrf_ = (*pts)[n - 1][0];
  }
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
