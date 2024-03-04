/******************************************************************************
 Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved
 *Author: HOZON
 *Date: 2023-08-05
 *****************************************************************************/

#pragma once

#include <memory>
#include <vector>

#include "depend/perception-base/base/measurement/arrow_measurement.h"
#include "depend/perception-base/base/point/point.h"

namespace hozon {
namespace mp {
namespace lm2 {

namespace perception_base = hozon::perception::base;

struct Arrow {
  uint8_t id;
  perception_base::ArrowType type;
  float confidence;
  float heading;  // 弧度值
  std::vector<perception_base::Point2DF>
      point_set_2d;  // 图像系点（左上角为起点， 逆时针顺序）
  std::vector<perception_base::Point3DF> point_set_3d;  // 车身系点

  double length = 1000.0;
  double width = 1000.0;
  perception_base::Point3DF mid_point;
  bool has_matched = false;
  bool ismature = false;
  int tracked_count = -1;
  int lost_age = -1;
};

using ArrowPtr = std::shared_ptr<Arrow>;
using ArrowConstPtr = std::shared_ptr<const Arrow>;

struct Arrows {
  std::vector<ArrowPtr> arrows;
};

using ArrowsPtr = std::shared_ptr<Arrows>;

}  // namespace lm2
}  // namespace mp
}  // namespace hozon
