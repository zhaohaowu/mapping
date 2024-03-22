/******************************************************************************
 Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved
 *Author: HOZON
 *Date: 2023-08-05
 *****************************************************************************/

#pragma once

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "depend/perception-base/base/point/point.h"

namespace hozon {
namespace mp {
namespace lm {
struct NoParking {
  uint8_t id;
  float confidence;
  std::vector<Eigen::Vector3d> vehicle_points;  // 车身系点
};

using NoParkingPtr = std::shared_ptr<NoParking>;
using NoParkingConstPtr = std::shared_ptr<const NoParking>;

struct NoParkings {
  std::vector<NoParkingPtr> noparkings;
};

using NoParkingsPtr = std::shared_ptr<NoParkings>;

}  // namespace lm
}  // namespace mp
}  // namespace hozon
