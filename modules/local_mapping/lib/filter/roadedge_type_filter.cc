// Copyright 2020 Hozon Inc. All Rights Reserved.
// @file: roadedge_type_filter.cc
// @brief: type filter

#include "modules/local_mapping/lib/filter/roadedge_type_filter.h"

namespace hozon {
namespace mp {
namespace lm {

RoadEdgeTypeFilter::RoadEdgeTypeFilter(RoadEdgeTargetPtr roadedge_target)
    : BaseLaneLineTypeFilter(roadedge_target) {
  max_history_window_ = 10;
  type_keep_weight_ = 1.0;
  type_count_threshold_ = 3;
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
