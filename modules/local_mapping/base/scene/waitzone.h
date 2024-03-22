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

enum class ZoneType {
  LEFTWAIT_ZONE = 0,
  STRAIGHTWAIT_ZONE = 1,
  UNKNOWN = 2,  // unknown
};
struct WaitZone {
  uint8_t id;
  ZoneType type;
  float confidence;
  std::vector<Eigen::Vector3d> vehicle_points;  // 车身系点
};

using WaitZonePtr = std::shared_ptr<WaitZone>;
using WaitZoneConstPtr = std::shared_ptr<const WaitZone>;

struct WaitZones {
  std::vector<WaitZonePtr> waitzones;
};

using WaitZonesPtr = std::shared_ptr<WaitZones>;

}  // namespace lm
}  // namespace mp
}  // namespace hozon
