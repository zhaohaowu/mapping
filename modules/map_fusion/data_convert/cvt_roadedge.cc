/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： cvt_roadedge.cc
 *   author     ： hozon
 *   date       ： 2023.09
 ******************************************************************************/

#include "depend/proto/local_mapping/local_map.pb.h"
#include "modules/map_fusion/base/element_map.h"
#include "modules/map_fusion/data_convert/data_convert.h"

namespace hozon {
namespace mp {
namespace mf {
void DataConvert::ElemMapAppendRoadEdge(
    const std::shared_ptr<hozon::mapping::LocalMap>& local_map,
    ElementMap::Ptr element_map_ptr) {
  for (const auto& road : local_map->road_edges()) {
    RoadEdge road_edge;
    road_edge.id = road.track_id();
    for (const auto& pt : road.points()) {
      if (std::isnan(pt.x()) || std::isnan(pt.y()) || std::isnan(pt.z())) {
        continue;
      }
      Eigen::Vector3f ptt(static_cast<float>(pt.x()),
                          static_cast<float>(pt.y()),
                          static_cast<float>(pt.z()));
      road_edge.points.emplace_back(ptt);
    }
    if (road_edge.points.empty()) {
      continue;
    }
    element_map_ptr->road_edges[road_edge.id] =
        std::make_shared<RoadEdge>(road_edge);
  }
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
