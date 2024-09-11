/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： topo_construct.h
 *   author     ： likuan
 *   date       ： 2024.08
 ******************************************************************************/
#pragma once

#include <depend/common/math/vec2d.h>

#include <algorithm>
#include <deque>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/map_fusion_02/base/interface_option.h"
#include "modules/map_fusion_02/base/junction.h"
#include "modules/map_fusion_02/common/calc_util.h"
#include "modules/map_fusion_02/common/common_data.h"
#include "modules/map_fusion_02/data_manager/junction_status_manager.h"
#include "modules/map_fusion_02/data_manager/location_data_manager.h"
#include "modules/map_fusion_02/modules/lane/road_topo_builder/junction_topo.h"
#include "modules/map_fusion_02/modules/lane/road_topo_builder/lane_topo.h"
#include "modules/map_fusion_02/modules/lane/road_topo_builder/utils.h"
#include "modules/util/include/util/mapping_log.h"

namespace hozon {
namespace mp {
namespace mf {

class RoadTopoConstruct {
 public:
  RoadTopoConstruct() = default;

  ~RoadTopoConstruct() = default;

  void Init(const LaneFusionProcessOption& conf);

  void Clear();

  bool ConstructTopology(double stamp, std::vector<Group::Ptr>* groups,
                         const std::shared_ptr<std::vector<KinePosePtr>>& path,
                         const KinePosePtr& curr_pose);

 private:
 private:
  LaneFusionProcessOption conf_;
  LaneTopoConstructPtr lane_topo_ = nullptr;
  JunctionTopoConstructPtr junc_topo_ = nullptr;

  const float kShrinkDiffThreshold = 0.5;
  const double kMergeLengthThreshold = 10.;
  const double kSplitLengthThreshold = 10.;
};

using RoadTopoConstructPtr = std::unique_ptr<RoadTopoConstruct>;

}  // namespace mf
}  // namespace mp
}  // namespace hozon
