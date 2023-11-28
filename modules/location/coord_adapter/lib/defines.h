/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： defines.h
 *   author     ： lilanxing
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <Eigen/Eigen>

namespace hozon {
namespace mp {
namespace loc {
namespace ca {

struct Params {
  uint32_t dr_deque_capacity = 100.0;
};

struct Node {
  double ticktime = -1;
  Eigen::Vector3d enu = Eigen::Vector3d::Zero();
  Eigen::Vector3d orientation = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
};

}  // namespace ca
}  // namespace loc
}  // namespace mp
}  // namespace hozon
