// Copyright 2020 Hozon Inc. All Rights Reserved.
// @file: roadedge_type_filter.h
// @brief: type filter

#pragma once

#include "modules/local_mapping/lib/filter/base_type_filter.h"
#include "modules/local_mapping/lib/target/base_target.h"
namespace hozon {
namespace mp {
namespace lm {
// 可以覆写基类的函数
class RoadEdgeTypeFilter
    : public BaseLaneLineTypeFilter<RoadEdgeType, RoadEdge, RoadEdgeTarget> {
 public:
  explicit RoadEdgeTypeFilter(RoadEdgeTargetPtr lane_target);
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
