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
#include "modules/local_mapping/base/object/object.h"
#include "modules/local_mapping/base/scene/arrow.h"
#include "modules/local_mapping/base/scene/laneline.h"
#include "modules/local_mapping/base/scene/noparking.h"
#include "modules/local_mapping/base/scene/roadedge.h"
#include "modules/local_mapping/base/scene/slowdown.h"
#include "modules/local_mapping/base/scene/stopline.h"
#include "modules/local_mapping/base/scene/waitzone.h"
#include "modules/local_mapping/base/scene/zebracrossing.h"
#include "perception-base/base/frame/frame.h"
namespace hozon {
namespace mp {
namespace lm {

struct Header {
  double timestamp = 0.0;
  uint32_t sequence_num = 0;
};

struct Frame {
  Header header;
};

struct MeasurementFrame : public Frame {
  LaneLinesPtr lane_lines_ptr{};
  RoadEdgesPtr road_edges_ptr{};
  ZebraCrossingsPtr zebra_crossings_ptr{};
  StopLinesPtr stop_lines_ptr{};
  ArrowsPtr road_arrows_ptr{};
  WaitZonesPtr wait_zones_ptr{};
  NoParkingsPtr no_parkings_ptr{};
  SlowDownsPtr slow_downs_ptr{};
  ObjectsPtr objects_ptr{};
};

struct LocalMapFrame : public Frame {
  LaneLinesPtr lane_lines_ptr{};
  RoadEdgesPtr road_edges_ptr{};
  ZebraCrossingsPtr zebra_crossings_ptr{};
  StopLinesPtr stop_lines_ptr{};
  ArrowsPtr road_arrows_ptr{};
  WaitZonesPtr wait_zones_ptr{};
  NoParkingsPtr no_parkings_ptr{};
  SlowDownsPtr slow_downs_ptr{};
};

using MeasurementFramePtr = std::shared_ptr<MeasurementFrame>;
using MeasurementFrameConstPtr = std::shared_ptr<const MeasurementFrame>;

using LocalMapFramePtr = std::shared_ptr<LocalMapFrame>;
using LocalMapFrameConstPtr = std::shared_ptr<const LocalMapFrame>;

}  // namespace lm
}  // namespace mp
}  // namespace hozon
