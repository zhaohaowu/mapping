/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： pose_estimate_solver.h
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <iostream>

#include "modules/location/pose_estimation/lib/pose_estimate/pose_estimate.h"

namespace hozon {
namespace mp {
namespace loc {

class MapMatchSolver {
 public:
  //   static Sophus::SE3d solve(const Connect &connect, Sophus::SE3d pose,
  //                             Sophus::SE3d pre_pose, const Sophus::SE3d
  //                             &T_V_C, bool &is_ok);
  static Sophus::SE3d solve2D(const Connect &connect, const Sophus::SE3d &pose,
                              const Sophus::SE3d &pre_pose, bool *is_ok);
  MapMatchSolver();
};

}  // namespace loc
}  // namespace mp
}  // namespace hozon
