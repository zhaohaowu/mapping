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

const float kShrinkDiffThreshold = 0.5;
const double kMergeLengthThreshold = 10.;
const double kSplitLengthThreshold = 10.;

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

  static bool LineIdConsistant(const LineSegment::Ptr& line, Id id);

  static bool NeedToConnect(const Lane::Ptr& lane_in_curr,
                            const Lane::Ptr& lane_in_next);

  static bool NeedToConnect(
      Group::Ptr curr_group, Group::Ptr next_group, int i, int j,
      std::map<int, std::vector<int>>* curr_group_next_lane,
      std::map<int, std::vector<int>>* next_group_prev_lane);

  static bool IsAccessLane(const Lane::Ptr& lane_in_curr,
                           const Lane::Ptr& lane_in_next);

  static bool IsRightLane(const Group::Ptr& next_group, int cur_lane_index,
                          int right_lane_inex);

  static bool IsLeftLane(const Group::Ptr& next_group, int cur_lane_index,
                         int left_lane_index);

  static bool IsShrinkLane(const Lane::Ptr& lane, float min_lane_width);

  static bool IsBoundaryValid(const LineSegment& line);

  static bool IsIntersect(const Lane::Ptr& line1, const Lane::Ptr& line2);

  static std::vector<Point> SigmoidFunc(const std::vector<Point>& centerline,
                                        float sigma);
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
