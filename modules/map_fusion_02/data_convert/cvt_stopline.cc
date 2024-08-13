/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： cvt_stopline.cc
 *   author     ： hozon
 *   date       ： 2023.09
 ******************************************************************************/

#include "depend/proto/local_mapping/local_map.pb.h"
#include "modules/map_fusion_02/base/element_map.h"
#include "modules/map_fusion_02/data_convert/data_convert.h"

namespace hozon {
namespace mp {
namespace mf {
void DataConvert::ElemMapAppendStopLine(
    const std::shared_ptr<hozon::mapping::LocalMap>& local_map,
    ElementMap::Ptr element_map_ptr) {
  for (const auto& stop_line : local_map->stop_lines()) {
    StopLine stopline;
    stopline.id = stop_line.track_id();
    Eigen::Vector3f slpt(static_cast<float>(stop_line.left_point().x()),
                         static_cast<float>(stop_line.left_point().y()),
                         static_cast<float>(stop_line.left_point().z()));
    stopline.points.emplace_back(slpt);
    Eigen::Vector3f slpt2(static_cast<float>(stop_line.right_point().x()),
                          static_cast<float>(stop_line.right_point().y()),
                          static_cast<float>(stop_line.right_point().z()));
    stopline.points.emplace_back(slpt2);
    element_map_ptr->stop_lines[stopline.id] =
        std::make_shared<StopLine>(stopline);
  }
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
