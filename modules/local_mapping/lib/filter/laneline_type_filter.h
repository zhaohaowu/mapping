// Copyright 2020 Hozon Inc. All Rights Reserved.
// @file: laneline_type_filter.h
// @brief: type filter

#pragma once

#include "modules/local_mapping/lib/filter/base_type_filter.h"
#include "modules/local_mapping/lib/target/base_target.h"
namespace hozon {
namespace mp {
namespace lm {
// 可以覆写基类的函数
class LaneLineTypeFilter : public BaseLaneLineTypeFilter<LaneLineType, LaneLine, LaneTarget> {
 public:
    explicit LaneLineTypeFilter(LaneTargetPtr lane_target);
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
