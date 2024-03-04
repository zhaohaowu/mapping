/******************************************************************************
 Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved
 *Author: HOZON
 *Date: 2023-08-05
 *****************************************************************************/

#pragma once

#include <map>
#include <memory>
#include <vector>

#include "depend/perception-base/base/laneline/base_laneline.h"
#include "depend/perception-base/base/point/point.h"

namespace hozon {
namespace mp {
namespace lm2 {

namespace perception_base = hozon::perception::base;
// @brief lane line definition
struct LaneLine {
  // @brief lane line type
  perception_base::LaneLineType type = perception_base::LaneLineType::Unknown;
  // @brief lane line color
  perception_base::LaneLineColor color =
      perception_base::LaneLineColor::UNKNOWN;
  // @brief lane line position
  perception_base::LaneLinePosition position;
  // @brief lane line quality
  // @brief lane line scene type
  perception_base::LaneLineSceneType scene_type =
      perception_base::LaneLineSceneType::UNKNOWN;
  // @brief lane line curve with vehicle coordinate system
  perception_base::LaneLineCurve vehicle_curve;
  // @brief lane line point set
  std::vector<perception_base::LaneLinePoint> points;

  std::vector<perception_base::Point3DF> control_points;

  std::vector<perception_base::Point3DF> fit_points;

  // @brief lane line track id
  int id = -1;

  int track_count = -1;

  int lost_age = -1;

  // @brief age of the tracked lane line
  double tracking_time = 0.0;

  // @brief timestamp of latest measurement
  double latest_tracked_time = 0.0;

  float type_confidence = 0.0;

  float shape_confidence = 0.0;

  float color_confidence = 0.0;

  float geo_confidence = 0.0;

  // 这下面5个参数需要更优雅的表示出来。
  bool need_delete_ = false;
  bool has_matched_ = false;
  bool ismature_ = false;
  bool is_after_stop_line_ = false;
  double c0_for_lanepos_ = 1000.0;
};

using LaneLinePtr = std::shared_ptr<LaneLine>;
using LaneLineConstPtr = std::shared_ptr<const LaneLine>;

struct LaneLines {
  std::vector<LaneLinePtr> lanelines;
};

using LaneLinesPtr = std::shared_ptr<LaneLines>;

using LaneLinePointPtr = std::shared_ptr<perception::base::LaneLinePoint>;
using LaneLinePointConstPtr =
    std::shared_ptr<const perception::base::LaneLinePoint>;

}  // namespace lm2
}  // namespace mp
}  // namespace hozon
