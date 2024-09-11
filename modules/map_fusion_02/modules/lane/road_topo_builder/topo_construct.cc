/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： topo_construct.cc
 *   author     ： likuan
 *   date       ： 2024.08
 ******************************************************************************/
#include "modules/map_fusion_02/modules/lane/road_topo_builder/topo_construct.h"

#include <memory>

#include "base/junction.h"
#include "base/utils/log.h"
#include "common/calc_util.h"
#include "modules/lane/road_builder/detect_cut_pt.h"
#include "modules/lane/road_topo_builder/junction_topo.h"
#include "noa/noa_define.h"

namespace hozon {
namespace mp {
namespace mf {

void RoadTopoConstruct::Init(const LaneFusionProcessOption& conf) {
  conf_ = conf;

  lane_topo_ = std::make_unique<LaneTopoConstruct>();
  lane_topo_->Init(conf);

  junc_topo_ = std::make_unique<JunctionTopoConstruct>();
  junc_topo_->Init(conf);

  HLOG_INFO << "Road topo construct init";
}

void RoadTopoConstruct::Clear() {
  lane_topo_->Clear();
  junc_topo_->Clear();
}

bool RoadTopoConstruct::ConstructTopology(
    double stamp, std::vector<Group::Ptr>* groups,
    const std::shared_ptr<std::vector<KinePosePtr>>& path,
    const KinePosePtr& curr_pose) {
  JunctionInfo junc_info = JUNC_MANAGER->GetJuncStatus();
  HLOG_INFO << "Road Scene Type: "
            << static_cast<int>(junc_info.road_scene_type);

  if (groups->size() < 2) {
    return true;
  }

  switch (junc_info.road_scene_type) {
    case RoadSceneType::GENERAL_ROAD:
    case RoadSceneType::NEAR_JUNCTION:
      lane_topo_->ConstructTopology(groups);
    case RoadSceneType::IN_JUNCTION:
      junc_topo_->ConstructTopology(stamp, groups, path, curr_pose);
  }

  return true;
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
