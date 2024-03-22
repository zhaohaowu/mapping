/******************************************************************************
 Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved
 *Author: HOZON
 *Date: 2023-08-05
 *****************************************************************************/

#pragma once

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "depend/perception-base/base/measurement/arrow_measurement.h"
#include "depend/perception-base/base/point/point.h"
#include "modules/local_mapping/base/scene/base.h"

namespace hozon {
namespace mp {
namespace lm {

enum class ArrowType {
  STRAIGHT_FORWARD = 0,                              // 直行箭头
  STRAIGHT_FORWARD_OR_TURN_LEFT = 1,                 // 直行或左转
  STRAIGHT_FORWARD_OR_TURN_RIGHT = 2,                // 直行或右转
  STRAIGHT_FORWARD_OR_TURN_LEFT_OR_TURN_RIGHT = 3,   // 直行或左转或右转
  STRAIGHT_FORWARD_OR_TURN_AROUND = 4,               // 直行或掉头
  STRAIGHT_FORWARD_OR_TURN_AROUND_OR_TURN_LEFT = 5,  // 直行或掉头或左转

  TURN_LEFT = 6,                   // 左转
  TURN_LEFT_OR_MERGE_LEFT = 7,     // 左转或向左合流
  TURN_LEFT_OR_TURN_AROUND = 8,    // 左转或掉头
  TURN_LEFT_OR_TURN_RIGHT = 9,     // 左转或右转
  TURN_RIGHT = 10,                 // 右转
  TURN_RIGHT_OR_MERGE_RIGHT = 11,  // 右转或向右合流
  TURN_RIGHT_OR_TURN_AROUND = 12,  // 右转或掉头
  TURN_AROUND = 13,                // 掉头

  FORBID_TURN_LEFT = 14,      // 禁止左转
  FORBID_TURN_RIGHT = 15,     // 禁止右转
  FORBID_TURN_AROUND = 16,    // 禁止掉头
  FRONT_NEAR_CROSSWALK = 17,  // 前向斑马线
  UNKNOWN = 18,
};

struct Arrow : public BaseData {
  uint8_t id;
  ArrowType type;
  float confidence;
  double heading;                               // 弧度值
  std::vector<Eigen::Vector3d> vehicle_points;  // 车身系点
  Eigen::Vector3d center_point;

  // @brief age of the tracked lane line
  double tracking_time = 0.0;

  // @brief timestamp of latest measurement
  double latest_tracked_time = 0.0;

  // 考虑废弃
  double length;
  double width;
};

using ArrowPtr = std::shared_ptr<Arrow>;
using ArrowConstPtr = std::shared_ptr<const Arrow>;

struct Arrows {
  std::vector<ArrowPtr> arrows;
};

using ArrowsPtr = std::shared_ptr<Arrows>;

using ArrowsConstPtr = std::shared_ptr<const Arrows>;

}  // namespace lm
}  // namespace mp
}  // namespace hozon
