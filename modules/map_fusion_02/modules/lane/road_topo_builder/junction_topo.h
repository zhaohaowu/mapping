/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： junction_topo.h
 *   author     ： likuan
 *   date       ： 2024.08
 ******************************************************************************/
#pragma once

#include <depend/common/math/vec2d.h>

#include <cfloat>
#include <vector>
#include <memory>

#include "modules/map_fusion_02/base/element_base.h"
#include "modules/map_fusion_02/base/group.h"
#include "modules/map_fusion_02/base/interface_option.h"
#include "modules/map_fusion_02/base/junction.h"
#include "modules/map_fusion_02/common/calc_util.h"
#include "modules/map_fusion_02/common/common_data.h"
#include "modules/map_fusion_02/data_manager/junction_status_manager.h"
#include "modules/map_fusion_02/modules/lane/road_topo_builder/utils.h"
#include "modules/util/include/util/mapping_log.h"

namespace hozon {
namespace mp {
namespace mf {

class JunctionTopoConstruct {
 public:
  JunctionTopoConstruct() = default;

  ~JunctionTopoConstruct() = default;

  void Init(const LaneFusionProcessOption& conf);

  void ConstructTopology(std::vector<Group::Ptr>* groups);

 private:
 private:
  LaneFusionProcessOption conf_;
};

using JunctionTopoConstructPtr = std::unique_ptr<JunctionTopoConstruct>;

}  // namespace mf
}  // namespace mp
}  // namespace hozon
