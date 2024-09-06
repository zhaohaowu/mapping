/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： junction_topo.cc
 *   author     ： likuan
 *   date       ： 2024.08
 ******************************************************************************/
#include "modules/map_fusion_02/modules/lane/road_topo_builder/junction_topo.h"
#include <vector>

namespace hozon {
namespace mp {
namespace mf {

void JunctionTopoConstruct::Init(const LaneFusionProcessOption& conf) {
  conf_ = conf;
  HLOG_INFO << "Junction construct init";
}

void JunctionTopoConstruct::ConstructTopology(std::vector<Group::Ptr>* groups) {
  JunctionInfo junc_info = JUNC_MANAGER->GetJuncStatus();
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
