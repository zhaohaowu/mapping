/******************************************************************************
 Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved
 *Author: HOZON
 *Date: 2023-08-05
 *****************************************************************************/

#pragma once

#include <cfloat>
#include <map>
#include <memory>
#include <vector>

#include "Eigen/Core"
#include "depend/perception-base/base/laneline/base_laneline.h"
#include "depend/perception-base/base/point/point.h"
#include "depend/proto/local_mapping/local_map.pb.h"
#include "modules/local_mapping/base/scene/base.h"

namespace hozon {
namespace mp {
namespace lm {
// @brief lane line definition

enum class CrossType {
  SPLIT = 0,
  MERGE = 1,
  UNKNOWN = 2,
};

struct CrossPoint {
  uint8_t id{};
  CrossType type{};  // 0分流 1合流
  float confidence{};
  Eigen::Vector3d vehicle_point{};
  Eigen::Vector3d world_point{};
};

enum class LaneLineType {
  UNKNOWN = 0,                  // 未知
  SOLID_LINE = 1,               // 单实线
  DASHED_LINE = 2,              // 单虚线
  SHORT_DASHED_LINE = 3,        // 短虚线
  DOUBLE_SOLID_LINE = 4,        // 双实线
  DOUBLE_DASHED_LINE = 5,       // 双虚线
  LEFT_SOLID_RIGHT_DASHED = 6,  // 左实右虚
  RIGHT_SOLID_LEFT_DASHED = 7,  // 右实左虚
  FISHBONE = 16,                // 鱼骨线
  FISHBONE_DASHED_LINE = 17,    // 鱼骨虚线
  CROSSGUIDE_LINE = 18,         // 引导线
  OTHER = 99,                   // 其他 UNKNOWN
};

// @brief lane line color
enum class LaneLineColor {
  UNKNOWN = 0,
  WHITE = 1,
  YELLOW = 2,
  GREEN = 3,
  RED = 4,
  BLACK = 5,
  BLUE = 6,
  ORANGE = 7,
  OTHER = 8,
};

// @brief lane line position respect to the ego lane
enum class LaneLinePosition : int {
  BOLLARD_LEFT = -5,
  FOURTH_LEFT = -4,
  THIRD_LEFT = -3,
  ADJACENT_LEFT = -2,  // ego左边第二个
  EGO_LEFT = -1,       // ego左边
  EGO_RIGHT = 1,       // ego右边
  ADJACENT_RIGHT = 2,  // ego又边第二个
  THIRD_RIGHT = 3,
  FOURTH_RIGHT = 4,
  BOLLARD_RIGHT = 5,
  OTHER = 6,
};

// @brief lane line scene type
enum class LaneLineSceneType {
  UNKNOWN = 0,   // unknow scene
  NORMAL = 1,    // common lane line
  FORK = 2,      // fork line
  CONVERGE = 3,  // converge line
};

struct LaneLineCurve {
  float min = 0.0F;
  float max = 0.0F;

  // curve coefficient ordered from low to high
  // c0 + c1 x + c2 x^2 + c3 x^3
  std::vector<float> coeffs;
};

struct LaneLine : public BaseData {
  // @brief lane line type
  LaneLineType type = LaneLineType::UNKNOWN;
  // @brief lane line color
  LaneLineColor color = LaneLineColor::UNKNOWN;
  // @brief local map position
  LaneLinePosition position;
  // @brief 车道线后处理的lane position
  LaneLinePosition te_position;
  // @brief lane line quality
  // @brief lane line scene type
  LaneLineSceneType scene_type = LaneLineSceneType::UNKNOWN;
  // @brief lane line curve with vehicle coordinate system
  LaneLineCurve vehicle_curve;
  // @brief lane line point set
  std::vector<Eigen::Vector3d> vehicle_points;

  std::vector<Eigen::Vector3d> world_points;

  std::vector<Eigen::Vector3d> control_points;

  std::vector<Eigen::Vector3d> fit_points;
  // @brief lane line stability error
  mapping::LaneStabilityError stability_error;

  // @brief lane line track id
  int id = -1;

  float score = 0.0;

  float type_confidence = 0.0;

  float shape_confidence = 0.0;

  float color_confidence = 0.0;

  float geo_confidence = 0.0;

  bool after_intersection = false;

  bool send_postlane = false;  // 标志车道线为后处理发送给定位等下游

  // 这下面5个参数需要更优雅的表示出来。
  double refer_c0 = FLT_MAX;
  double theory_c0 = FLT_MAX;
  bool cross = false;
};

using LaneLinePtr = std::shared_ptr<LaneLine>;
using LaneLineConstPtr = std::shared_ptr<const LaneLine>;

using CrossPointPtr = std::shared_ptr<CrossPoint>;
using CrossPointConstPtr = std::shared_ptr<const CrossPoint>;

struct LaneLines {
  std::vector<LaneLinePtr> lanelines;
  std::vector<CrossPointPtr> crosspoints;
};

using LaneLinesPtr = std::shared_ptr<LaneLines>;

using LaneLinesConstPtr = std::shared_ptr<const LaneLines>;

// using LaneLinePointPtr = std::shared_ptr<perception::base::LaneLinePoint>;
// using LaneLinePointConstPtr =
//     std::shared_ptr<const perception::base::LaneLinePoint>;

}  // namespace lm
}  // namespace mp
}  // namespace hozon
