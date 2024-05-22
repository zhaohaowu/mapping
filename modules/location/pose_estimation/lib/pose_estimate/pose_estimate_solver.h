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
  static bool solve2D(const Connect& connect, const Sophus::SE3d& input_pose,
                      Sophus::SE3d* result_pose);
};

}  // namespace loc
}  // namespace mp
}  // namespace hozon
