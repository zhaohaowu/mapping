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

struct StopLine {
  uint8_t id{};
  float confidence = 0.0;
  perception_base::Point3DF left_point;
  perception_base::Point3DF right_point;
  perception_base::Point3DF mid_point;
  double length_{};
  double heading_{};
  bool has_matched = false;
  bool ismature = false;
  bool isstable = false;
  int tracked_count = 0;
};

using StopLinePtr = std::shared_ptr<StopLine>;
using StopLineConstPtr = std::shared_ptr<const StopLine>;

struct StopLines {
  std::vector<StopLinePtr> stoplines;
};

using StopLinesPtr = std::shared_ptr<StopLines>;

}  // namespace lm2
}  // namespace mp
}  // namespace hozon
