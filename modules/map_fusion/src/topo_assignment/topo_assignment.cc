/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： topo_assignment.cc
 *   author     ： xuliang
 *   date       ： 2023.09
 ******************************************************************************/

#include "map_fusion/topo_assignment/topo_assignment.h"

#include "util/temp_log.h"

namespace hozon {
namespace mp {
namespace mf {

int TopoAssignment::Init() {
  topo_map_ = std::make_shared<hozon::hdmap::Map>();
  hq_map_ = std::make_shared<hozon::hdmap::Map>();
  return 0;
}

void TopoAssignment::OnInsNodeInfo(
    const std::shared_ptr<adsfi_proto::internal::HafNodeInfo>& msg) {
  // ins的位置通过这个位置去拿取hq地图
  vehicle_pose_ << msg->pos_gcj02().x(), msg->pos_gcj02().y(),
      msg->pos_gcj02().z();
}
void TopoAssignment::OnHQMap(const std::shared_ptr<hozon::hdmap::Map>& msg) {
  // 通过vehicle pose拿取的hq地图
  hq_map_ = msg;
}
void TopoAssignment::OnLocalMap(
    const std::shared_ptr<hozon::mapping::LocalMap>& msg) {
  // 先不考虑历史信息，全部赋予拓扑
}

std::shared_ptr<hozon::hdmap::Map> TopoAssignment::GetTopoMap() {
  return topo_map_;
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
