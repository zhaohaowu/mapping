/******************************************************************************
 Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved
 *Author: HOZON
 *Date: 2023-08-05
 *****************************************************************************/
#pragma once
#include <memory>
#include <vector>

#include "depend/perception-base/base/point/point.h"

namespace hozon {
namespace mp {
namespace lm2 {

namespace perception_base = hozon::perception::base;

struct ZebraCrossing {
  uint8_t id;
  float confidence;
  float heading;
  std::vector<perception_base::Point2DF>
      point_set_2d;  // 图像系点（左上角为起点， 逆时针顺序）
  std::vector<perception_base::Point3DF> point_set_3d;  // 车身系点

  double length{};
  double width{};
  perception_base::Point3DF mid_point_;
  bool has_matched = false;
  bool ismature = false;
  int tracked_count = -1;
  int lost_age = -1;
};

using ZebraCrossingPtr = std::shared_ptr<ZebraCrossing>;
using ZebraCrossingConstPtr = std::shared_ptr<const ZebraCrossing>;

struct ZebraCrossings {
  std::vector<ZebraCrossingPtr> crosswalks;
};
using ZebraCrossingsPtr = std::shared_ptr<ZebraCrossings>;
}  // namespace lm2
}  // namespace mp
}  // namespace hozon
