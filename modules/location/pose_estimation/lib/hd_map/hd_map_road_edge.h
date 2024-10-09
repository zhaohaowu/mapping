/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： hd_map_road_edge.h
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <Eigen/Core>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <unordered_set>
#include "depend/proto/common/types.pb.h"
#include "modules/location/pose_estimation/lib/hd_map/hd_map_base.h"
#include "modules/location/pose_estimation/lib/util/euler.h"
#include "modules/map_fusion_02/modules/map_hd/global_hd_map.h"
#include "modules/util/include/util/geo.h"
#include "depend/common/utm_projection/coordinate_convertor.h"
namespace hozon {
namespace mp {
namespace loc {

struct RoadEdge {
  std::string id_edge;
  std::vector<ControlPoint> control_point;
};

class MapRoadEdge : public MapElement {
 public:
  /**
   * @brief crop map
   *
   * @param T_W_V : vehicle pose in the gcj02 coordinates
   * @param front : range in front of the vehicle
   * @param width : range in side of the vehicle
   * @return
   */
  void Crop(const SE3& T_W_V, double front, double width);

  /**
   * @brief add road edge lines
   *
   * @param hd_map : hd_map message
   * @param ref_point : reference point
   * @return
   */
  void Set(const std::vector<hozon::hdmap::LaneInfoConstPtr>& lane_ptr_vec,
           const V3& ref_point);
  using Ptr = std::shared_ptr<MapRoadEdge>;
  std::unordered_map<std::string, RoadEdge> edge_line_;
};

}  // namespace loc
}  // namespace mp
}  // namespace hozon
