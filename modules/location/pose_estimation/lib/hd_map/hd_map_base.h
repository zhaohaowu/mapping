/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： hd_map_base.h
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <interface/adsfi_proto/internal/slam_hd_submap.pb.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <map>
#include <memory>
#include <unordered_map>
#include <vector>

#include <Sophus/se3.hpp>
#include <Sophus/so3.hpp>

#include "modules/util/include/util/mapping_log.h"

namespace hozon {
namespace mp {
namespace loc {

using V3 = Eigen::Matrix<double, 3, 1>;
using VP = std::vector<V3>;
using SE3 = Sophus::SE3<double>;

enum {
  HD_MAP_LANE_BOUNDARY_LINE = 0,
  HD_MAP_LANE_CENTER_LINE = 1,
  HD_MAP_ROAD_EDGE = 2,
  HD_MAP_POLE = 3,
  HD_MAP_TRAFFIC_SIGN = 4,
};

struct ControlPoint {
  size_t line_type;
  size_t lane_type;
  V3 point;
  ControlPoint(size_t line_type, size_t lane_type, V3 point)
      : line_type(line_type), lane_type(lane_type), point(point) {}
};

class MapElement {
 public:
  using Ptr = std::shared_ptr<MapElement>;
  int type_;
};

// double type
enum DoubleLineType {
  DoubleSolidLine =
      adsfi_proto::internal::SubMap_LineType_LineType_DoubleSolidLine,
  DoubleDashedLine =
      adsfi_proto::internal::SubMap_LineType_LineType_DoubleSolidLine,
  LeftSolidRightDashed =
      adsfi_proto::internal::SubMap_LineType_LineType_LeftSolidRightDashed,
  RightSolidLeftDashed =
      adsfi_proto::internal::SubMap_LineType_LineType_RightSolidLeftDashed
};

}  // namespace loc
}  // namespace mp
}  // namespace hozon
