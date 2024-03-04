/******************************************************************************
 Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved
 *Author: HOZON
 *Date: 2023-08-05
 *****************************************************************************/

#pragma once

#include <memory>
#include <vector>

#include "depend/perception-base/base/measurement/waitzone_measurement.h"
#include "depend/perception-base/base/point/point.h"

namespace hozon {
namespace mp {
namespace lm2 {

namespace perception_base = hozon::perception::base;

struct WaitZone {
  uint8_t id;
  perception_base::ZoneType type;
  float confidence;
  std::vector<perception_base::Point2DF>
      point_set_2d;  // 图像系点（左上角为起点， 逆时针顺序）
  std::vector<perception_base::Point3DF> point_set_3d;  // 车身系点
};

using WaitZonePtr = std::shared_ptr<WaitZone>;
using WaitZoneConstPtr = std::shared_ptr<const WaitZone>;

struct WaitZones {
  std::vector<WaitZonePtr> waitzones;
};

using WaitZonesPtr = std::shared_ptr<WaitZones>;

}  // namespace lm2
}  // namespace mp
}  // namespace hozon
