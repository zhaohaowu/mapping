/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： utils.h
 *   author     ： likuan
 *   date       ： 2024.08
 ******************************************************************************/
#pragma once

#include <depend/common/math/vec2d.h>

#include <algorithm>
#include <deque>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/map_fusion_02/base/group.h"
#include "modules/map_fusion_02/base/interface_option.h"
#include "modules/map_fusion_02/common/calc_util.h"
#include "modules/map_fusion_02/common/common_data.h"
#include "modules/util/include/util/mapping_log.h"

namespace hozon {
namespace mp {
namespace mf {

class TopoUtils {
 public:
  TopoUtils() = default;

  ~TopoUtils() = default;

  static float LaneDist(const Lane::Ptr& lane_in_curr,
                        const Lane::Ptr& lane_in_next);

  static float CalculateDistPt(const Lane::Ptr& lane_in_next,
                               const Lane::Ptr& lane_in_curr, size_t sizet);

  static float CalculatePoint2CenterLine(const Lane::Ptr& lane_in_next,
                                         const Lane::Ptr& lane_in_curr);

  static float Calculate2CenterlineAngle(const Lane::Ptr& lane_in_next,
                                         const Lane::Ptr& lane_in_curr,
                                         size_t sizet);

  static double CalcLaneLength(const Lane::Ptr& lane);

  static float PointToLineDis(const LineSegment& line, float line_front_x,
                              float line_front_y);

  static void FitCenterLine(Lane::Ptr lane);
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
