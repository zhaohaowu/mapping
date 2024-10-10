/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： cvt_zebracrossing.cc
 *   author     ： hozon
 *   date       ： 2023.09
 ******************************************************************************/

#include "depend/proto/local_mapping/local_map.pb.h"
#include "modules/map_fusion/base/element_map.h"
#include "modules/map_fusion/data_convert/data_convert.h"

namespace hozon {
namespace mp {
namespace mf {
void DataConvert::ElemMapAppendZebra(
    const std::shared_ptr<hozon::mapping::LocalMap>& local_map,
    ElementMap::Ptr element_map_ptr) {
  for (const auto& cross_walk : local_map->cross_walks()) {
    if (cross_walk.points().point_size() != 4) {
      continue;
    }
    CrossWalk crw;
    crw.id = cross_walk.track_id();
    for (const auto& pt : cross_walk.points().point()) {
      if (std::isnan(pt.x()) || std::isnan(pt.y()) || std::isnan(pt.z())) {
        continue;
      }
      Eigen::Vector3f point(static_cast<float>(pt.x()),
                            static_cast<float>(pt.y()),
                            static_cast<float>(pt.z()));
      crw.polygon.points.emplace_back(point);
    }
    element_map_ptr->cross_walks[crw.id] = std::make_shared<CrossWalk>(crw);
  }
}
}  // namespace mf
}  // namespace mp
}  // namespace hozon
