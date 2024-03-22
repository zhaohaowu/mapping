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

struct InsData {
 public:
  double timestamp;
  Eigen::Vector3d position;
  Eigen::Quaterniond quaternion;
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
};

using InsDataPtr = std::shared_ptr<InsData>;
using InsDataConstPtr = std::shared_ptr<const InsData>;

}  // namespace lm
}  // namespace mp
}  // namespace hozon
