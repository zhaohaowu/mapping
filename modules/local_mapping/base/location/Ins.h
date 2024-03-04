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
namespace lm2 {

struct InsData {
 public:
  double timestamp_;
  Eigen::Vector3d position_;
  Eigen::Quaterniond quaternion_;
};

using InsDataPtr = std::shared_ptr<InsData>;
using InsDataConstPtr = std::shared_ptr<const InsData>;

}  // namespace lm2
}  // namespace mp
}  // namespace hozon
