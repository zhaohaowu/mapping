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
bool cvt_pb2stopline(const hozon::mapping::StopLine& stop_line,
                     StopLine::Ptr elem_stopline_ptr) {
  elem_stopline_ptr->id = stop_line.track_id();
  Eigen::Vector3f slpt(static_cast<float>(stop_line.left_point().x()),
                       static_cast<float>(stop_line.left_point().y()),
                       static_cast<float>(stop_line.left_point().z()));
  elem_stopline_ptr->points.emplace_back(slpt);
  Eigen::Vector3f slpt2(static_cast<float>(stop_line.right_point().x()),
                        static_cast<float>(stop_line.right_point().y()),
                        static_cast<float>(stop_line.right_point().z()));
  elem_stopline_ptr->points.emplace_back(slpt2);
  return true;
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
