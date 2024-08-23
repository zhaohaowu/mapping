/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： topo_construct.cc
 *   author     ： likuan
 *   date       ： 2024.08
 ******************************************************************************/
#include "modules/map_fusion_02/modules/lane/road_topo_builder/topo_construct.h"

#include <memory>

#include "modules/lane/road_topo_builder/junction_topo.h"

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

bool RoadTopoConstruct::ConstructTopology(std::vector<Group::Ptr>* groups) {
  // 车道线补齐逻辑，建立车道线拓扑关系
  lane_topo_->ConstructTopology(groups);

  // 设置lane属性(is_ego、is_tran)
  SetLaneStatus(groups);
  return true;
}

bool RoadTopoConstruct::SetLaneStatus(std::vector<Group::Ptr>* groups) {
  // 添加主路和是否当前朝向属性
  for (auto& group : *groups) {
    int flag = 0;
    for (auto& lane : group->lanes) {
      if (lane->left_boundary->isego == IsEgo::Ego_Road) {
        lane->is_ego = 1;
      }
      if (flag == 0) {
        if (lane->left_boundary->color == Color::YELLOW) {
          flag = 1;
          lane->is_trans = 1;
        }
      } else {
        lane->is_trans = 1;
      }
    }
    if (flag == 0) {
      for (auto& lane : group->lanes) {
        if (lane->is_ego) {
          lane->is_trans = 1;
        }
      }
    }
  }
  return true;
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
