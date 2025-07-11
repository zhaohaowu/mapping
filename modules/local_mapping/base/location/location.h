/*================================================================
*   Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
*   file       ：laneline_struct.h
*   author     ：WuXiaopeng
*   date       ：2021.11.11
================================================================*/

#pragma once

#include <memory>

#include "Eigen/Dense"
#include "Eigen/Geometry"

namespace hozon {
namespace mp {
namespace lm {

struct Location {
  double timestamp;
  Eigen::Vector3d position;
  Eigen::Quaterniond quaternion;
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  Eigen::Vector3d euler_angle;
  Eigen::Vector3d linear_vrf;
  Eigen::Vector3d angular_vrf;
  float heading;
};

using LocationPtr = std::shared_ptr<Location>;
using LocationConstPtr = std::shared_ptr<const Location>;

}  // namespace lm
}  // namespace mp
}  // namespace hozon
