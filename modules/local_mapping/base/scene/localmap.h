/******************************************************************************
 Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved
 *Author: HOZON
 *Date: 2023-08-05
 *****************************************************************************/

#pragma once

#include <memory>
#include <vector>

#include "boost/circular_buffer.hpp"
#include "boost/circular_buffer/base.hpp"
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

class LocalMap {
 public:
  LocalMap() = default;
  ~LocalMap() = default;
  LaneLinesPtr lane_lines{};
  RoadEdgesPtr road_edges{};
  ZebraCrossingsPtr zebra_crossings{};
  StopLinesPtr stop_lines{};
  ArrowsPtr road_arrows{};
  WaitZonesPtr wait_zones{};
  NoParkingsPtr no_parkings{};
  SlowDownsPtr slow_downs{};

  std::vector<boost::circular_buffer<StopLinesPtr>>
      history_per_stop_lines_;  // 保存10帧感知停止线
  std::vector<boost::circular_buffer<ArrowsPtr>>
      history_per_arrows_;  // 保存10帧感知箭头
  std::vector<boost::circular_buffer<ZebraCrossingsPtr>>
      history_per_zebra_crossings_;  // 保存10帧感知斑马线

  double timestamp;
};

using LocalMapPtr = std::shared_ptr<LocalMap>;
using LocalMapConstPtr = std::shared_ptr<const LocalMap>;

}  // namespace lm2
}  // namespace mp
}  // namespace hozon
