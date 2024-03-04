/******************************************************************************
 Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved
 *Author: HOZON
 *Date: 2023-08-05
 *****************************************************************************/
#pragma once

#include <memory>
#include <vector>

#include "base/measurement/measurement.h"
#include "base/measurement/roadedges_measurement.h"
#include "base/point/point.h"

namespace hozon {
namespace mp {
namespace lm2 {

namespace perception_base = hozon::perception::base;

struct RoadEdge {
  uint8_t id;
  float confidence;
  perception_base::RoadEdgeType type;
  std::vector<perception_base::LaneLinePoint> point_set;
  perception_base::LaneLineCurve vehicle_curve;

  perception_base::LaneLinePosition position;
  // @brief age of the tracked lane line
  double tracking_time = 0.0;

  // @brief timestamp of latest measurement
  double latest_tracked_time = 0.0;

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

using RoadEdgePtr = std::shared_ptr<RoadEdge>;
using RoadEdgeConstPtr = std::shared_ptr<const RoadEdge>;

struct RoadEdges {
  std::vector<RoadEdgePtr> road_edges;
};
using RoadEdgesPtr = std::shared_ptr<RoadEdges>;

}  // namespace lm2
}  // namespace mp
}  // namespace hozon
