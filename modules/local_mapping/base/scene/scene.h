/******************************************************************************
 Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved
 *Author: HOZON
 *Date: 2023-08-05
 *****************************************************************************/

#pragma once

#include <memory>
#include <vector>

#include "modules/local_mapping/base/scene/arrow.h"
#include "modules/local_mapping/base/scene/laneline.h"
#include "modules/local_mapping/base/scene/noparking.h"
#include "modules/local_mapping/base/scene/roadedges.h"
#include "modules/local_mapping/base/scene/slowdown.h"
#include "modules/local_mapping/base/scene/stopline.h"
#include "modules/local_mapping/base/scene/waitzone.h"
#include "modules/local_mapping/base/scene/zebra_crossing.h"

namespace hozon {
namespace mp {
namespace lm2 {

struct Scene {
 public:
  Scene() = default;
  ~Scene() = default;
  LaneLinesPtr lane_lines{};
  RoadEdgesPtr road_edges{};
  ZebraCrossingsPtr zebra_crossings{};
  StopLinesPtr stop_lines{};
  ArrowsPtr road_arrows{};
  WaitZonesPtr wait_zones{};
  NoParkingsPtr no_parkings{};
  SlowDownsPtr slow_downs{};

  double timestamp;
};

using ScenePtr = std::shared_ptr<Scene>;
using SceneConstPtr = std::shared_ptr<const Scene>;

}  // namespace lm2
}  // namespace mp
}  // namespace hozon
