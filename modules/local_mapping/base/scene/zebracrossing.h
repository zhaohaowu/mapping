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

struct ZebraCrossing : public BaseData {
  uint8_t id;
  float confidence;
  double heading;
  std::vector<Eigen::Vector3d> vehicle_points;  // 车身系点
  Eigen::Vector3d center_point;
  double length{};
  double width{};
};

using ZebraCrossingPtr = std::shared_ptr<ZebraCrossing>;
using ZebraCrossingConstPtr = std::shared_ptr<const ZebraCrossing>;

struct ZebraCrossings {
  std::vector<ZebraCrossingPtr> zebra_crossings;
};

using ZebraCrossingsPtr = std::shared_ptr<ZebraCrossings>;
using ZebraCrossingsConstPtr = std::shared_ptr<const ZebraCrossings>;

}  // namespace lm
}  // namespace mp
}  // namespace hozon
