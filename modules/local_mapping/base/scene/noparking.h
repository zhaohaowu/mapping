/******************************************************************************
 Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved
 *Author: HOZON
 *Date: 2023-08-05
 *****************************************************************************/

#pragma once

#include <memory>
#include <vector>

#include "depend/perception-base/base/point/point.h"

namespace hozon {
namespace mp {
namespace lm2 {

namespace perception_base = hozon::perception::base;

struct NoParking {
  uint8_t id;
  float confidence;
  std::vector<perception_base::Point2DF>
      point_set_2d;  // 图像系点（左上角为起点， 逆时针顺序）
  std::vector<perception_base::Point3DF> point_set_3d;  // 车身系点
};

using NoParkingPtr = std::shared_ptr<NoParking>;
using NoParkingConstPtr = std::shared_ptr<const NoParking>;

struct NoParkings {
  std::vector<NoParkingPtr> noparkings;
};

using NoParkingsPtr = std::shared_ptr<NoParkings>;

}  // namespace lm2
}  // namespace mp
}  // namespace hozon
