/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： hd_map_road_edge.h
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <interface/adsfi_proto/internal/slam_hd_submap.pb.h>

#include <Eigen/Core>
#include <iostream>
#include <memory>
#include <unordered_map>
#include <vector>

#include "modules/location/pose_estimation/lib/hd_map/hd_map_base.h"
#include "modules/util/include/util/geo.h"

namespace hozon {
namespace mp {
namespace loc {

struct RoadEdge {
  size_t id_edge;
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
  void Set(const adsfi_proto::internal::SubMap& hd_map, const V3& ref_point);
  using Ptr = std::shared_ptr<MapRoadEdge>;
  std::unordered_map<size_t, RoadEdge> edge_line_;
};

}  // namespace loc
}  // namespace mp
}  // namespace hozon
