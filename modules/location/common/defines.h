/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： defines.h
 *   author     ： lilanxing
 *   date       ： 2023.10
 ******************************************************************************/

#pragma once

#include <Eigen/Eigen>
#include <memory>
#include <Sophus/se3.hpp>

namespace hozon {
namespace mp {
namespace loc {
namespace cm {

struct BaseNode {
  double ticktime = -1;
  Eigen::Vector3d enu = Eigen::Vector3d::Zero();
  Eigen::Vector3d orientation = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
};

Sophus::SE3d Node2SE3(const BaseNode& n) {
  return Sophus::SE3d(Sophus::SO3d::exp(n.orientation), n.enu);
}

Sophus::SE3d Node2SE3(const std::shared_ptr<BaseNode>& n) {
  return Node2SE3(*n);
}

}  // namespace cm
}  // namespace loc
}  // namespace mp
}  // namespace hozon
