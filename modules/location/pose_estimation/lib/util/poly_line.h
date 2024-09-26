/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： poly_line.h
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <iostream>
#include <vector>

#include "depend/proto/local_mapping/local_map.pb.h"
#include "modules/location/pose_estimation/lib/hd_map/hd_map_base.h"
namespace hozon {
namespace mp {
namespace loc {
using LaneLine = ::hozon::mapping::LaneLine;

template <typename T>
class PolyLine {
 public:
  PolyLine() {}
  explicit PolyLine(T t);
  int id_;
  float min_;
  float max_;
  double confidence_;
  int lane_position_type_;
  std::vector<hozon::mp::loc::V3> points;
};

}  // namespace loc
}  // namespace mp
}  // namespace hozon
