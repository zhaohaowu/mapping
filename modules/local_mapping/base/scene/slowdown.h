/******************************************************************************
 Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved
 *Author: HOZON
 *Date: 2023-08-05
 *****************************************************************************/

#pragma once

#include <memory>
#include <vector>

#include "Eigen/Core"

namespace hozon {
namespace mp {
namespace lm {

struct SlowDown {
  uint8_t id;
  float heading;  // 弧度值
  float confidence;

  std::vector<Eigen::Vector3d> vehicle_points;  // 车身系点
};

using SlowDownPtr = std::shared_ptr<SlowDown>;
using SlowDownConstPtr = std::shared_ptr<const SlowDown>;

struct SlowDowns {
  std::vector<SlowDownPtr> slow_downs;
};

using SlowDownsPtr = std::shared_ptr<SlowDowns>;

}  // namespace lm
}  // namespace mp
}  // namespace hozon
