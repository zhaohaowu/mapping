/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： cvt_roadedge.cc
 *   author     ： hozon
 *   date       ： 2023.09
 ******************************************************************************/

#include "depend/proto/local_mapping/local_map.pb.h"
#include "modules/map_fusion_02/base/element_map.h"
#include "modules/map_fusion_02/data_convert/data_convert.h"

namespace hozon {
namespace mp {
namespace mf {
void ElemMapAppendRoadEdge(
    const std::shared_ptr<hozon::mapping::LocalMap>& local_map,
    ElementMap::Ptr element_map_ptr) {
  for (const auto& road : local_map->road_edges()) {
    RoadEdge road_edge;
    road_edge.id = road.track_id();
    std::vector<Eigen::Vector3d> road_pts;
    for (const auto& pt : road.points()) {
      Eigen::Vector3d ptt(pt.x(), pt.y(), pt.z());
      road_pts.emplace_back(ptt);
    }
    if (road_pts.empty()) {
      continue;
    }
    road_edge.points.emplace_back(road_pts);
    element_map_ptr->road_edges[road_edge.id] =
        std::make_shared<RoadEdge>(road_edge);
  }
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
