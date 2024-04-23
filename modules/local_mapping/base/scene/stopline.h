/******************************************************************************
 Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved
 *Author: HOZON
 *Date: 2023-08-05
 *****************************************************************************/

#pragma once

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "modules/local_mapping/base/scene/base.h"
namespace hozon {
namespace mp {
namespace lm {

struct StopLine : public BaseData {
  uint8_t id{};
  float confidence = 0.0;
  Eigen::Vector3d left_point{};
  Eigen::Vector3d right_point{};
  Eigen::Vector3d center_point{};
  double heading{};
  double length{};
};

using StopLinePtr = std::shared_ptr<StopLine>;
using StopLineConstPtr = std::shared_ptr<const StopLine>;

struct StopLines {
  std::vector<StopLinePtr> stoplines;
};

using StopLinesPtr = std::shared_ptr<StopLines>;

}  // namespace lm
}  // namespace mp
}  // namespace hozon
