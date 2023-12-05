/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： poly_line.h
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <iostream>
#include <vector>

#include "proto/perception/transport_element.pb.h"
#include "modules/location/pose_estimation/lib/hd_map/hd_map_base.h"
namespace hozon {
namespace mp {
namespace loc {
using LaneLine = ::hozon::perception::LaneInfo;

template <typename T>
class PolyLine {
 public:
  PolyLine() {}
  explicit PolyLine(T t);
  int id_;
  float min_;
  float max_;
  float c0_;
  float c1_;
  float c2_;
  float c3_;
  float confidence_;
  int lane_position_type_;
  std::vector<hozon::mp::loc::V3> points;
};

}  // namespace loc
}  // namespace mp
}  // namespace hozon
