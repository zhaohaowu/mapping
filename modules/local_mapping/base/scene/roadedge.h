/******************************************************************************
 Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved
 *Author: HOZON
 *Date: 2023-08-05
 *****************************************************************************/
#pragma once

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "modules/local_mapping/base/scene/base.h"
#include "modules/local_mapping/base/scene/laneline.h"
namespace hozon {
namespace mp {
namespace lm {

enum class RoadEdgeType {
  ROAD_EDGE = 0,        // 马路路沿
  GROUND_EDGE = 1,      // 地面路沿
  CONE_EDGE = 2,        // 锥桶路沿
  WATERHORSE_EDGE = 3,  // 水马路沿
  FENCE_EDGE = 4,       // 围栏路沿
  UNKNOWN = 5,
};

struct RoadEdge : public BaseData {
  RoadEdgeType type;

  LaneLineCurve vehicle_curve;

  LaneLinePosition position;
  LaneLinePosition te_position;
  // @brief age of the tracked lane line
  double tracking_time = 0.0;

  // @brief timestamp of latest measurement
  double latest_tracked_time = 0.0;

  std::vector<Eigen::Vector3d> vehicle_points;

  std::vector<Eigen::Vector3d> world_points;

  std::vector<Eigen::Vector3d> control_points;

  std::vector<Eigen::Vector3d> fit_points;

  // @brief lane line track id
  int id = -1;

  float score = 0.0;

  float type_confidence = 0.0;

  float shape_confidence = 0.0;

  float color_confidence = 0.0;

  float geo_confidence = 0.0;

  bool send_postlane = false;  // 标志车道线为后处理发送给定位等下游
  // 这下面5个参数需要更优雅的表示出来。
};

using RoadEdgePtr = std::shared_ptr<RoadEdge>;
using RoadEdgeConstPtr = std::shared_ptr<const RoadEdge>;

struct RoadEdges {
  std::vector<RoadEdgePtr> road_edges;
};

using RoadEdgesPtr = std::shared_ptr<RoadEdges>;
using RoadEdgesConstPtr = std::shared_ptr<const RoadEdges>;

}  // namespace lm
}  // namespace mp
}  // namespace hozon
