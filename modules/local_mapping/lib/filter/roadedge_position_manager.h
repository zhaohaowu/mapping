// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Hozon (hozon@hozon.com)
// @file: roadedge_position_manager.h
// @brief: roadedge_position for local map

#pragma once
#include <float.h>

#include <functional>
#include <map>
#include <vector>

#include "modules/local_mapping/base/scene/localmap.h"
#include "modules/local_mapping/lib/target/base_target.h"
#include "modules/local_mapping/lib/tracker/laneline_tracker.h"
namespace hozon {
namespace mp {
namespace lm {

class RoadEdgePositionManager {
 public:
  void Init();

  void Process(const RoadEdgesPtr& roadedges_ptr);

 private:
  bool inited_ = false;
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
