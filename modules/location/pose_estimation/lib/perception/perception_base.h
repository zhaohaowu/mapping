/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： perception_base.h
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <iostream>
#include <memory>

namespace hozon {
namespace mp {
namespace loc {

enum {
  PERCEPTYION_LANE_BOUNDARY_LINE = 0,
  PERCEPTYION_ROAD_EDGE = 1,
  PERCEPTYION_POLE = 2,
  PERCEPTYION_TRAFFIC_SIGN = 3,
};

class PerceptionElement {
 public:
  using Ptr = std::shared_ptr<PerceptionElement>;
  int type_;
};

}  // namespace loc
}  // namespace mp
}  // namespace hozon
