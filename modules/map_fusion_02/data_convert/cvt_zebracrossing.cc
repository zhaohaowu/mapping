/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： broken_point_search.h
 *   author     ： hozon
 *   date       ： 2023.09
 ******************************************************************************/

#include "depend/proto/local_mapping/local_map.pb.h"
#include "modules/map_fusion_02/base/element_map.h"

namespace hozon {
namespace mp {
namespace mf {
bool cvt_pb2zebra(const hozon::mapping::CrossWalk& cross_walk,
                  CrossWalk::Ptr cross_walk_ptr) {
  if (cross_walk.points().point_size() != 4) {
    return false;
  }
  cross_walk_ptr->id = cross_walk.track_id();
  for (const auto& pt : cross_walk.points().point()) {
    Eigen::Vector3f point(static_cast<float>(pt.x()),
                          static_cast<float>(pt.y()),
                          static_cast<float>(pt.z()));
    cross_walk_ptr->polygon.points.emplace_back(point);
  }
  return true;
}
}  // namespace mf
}  // namespace mp
}  // namespace hozon
