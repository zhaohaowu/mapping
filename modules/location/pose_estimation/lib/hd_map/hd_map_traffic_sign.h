/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： hd_map_traffic_sign.h
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

struct TrafficSign {
  size_t id;
  int32_t shape;
  Eigen::Vector3d position;
  std::vector<Eigen::Vector3d> bounding_box;
};

class MapTrafficSign : public MapElement {
 private:
  const size_t max_len = 1e8;

 public:
  /**
   * @brief crop map
   *
   * @param T_W_V : vehicle pose in the gcj02 coordinates
   * @param front : range in front of the vehicle
   * @param width : range in side of the vehicle
   * @return
   */
  void Crop(const SE3 &T_W_V, const double &front, const double &width);

  /**
   * @brief add traffic signs
   *
   * @param hd_map : hd_map message
   * @param ref_point : reference point
   * @return
   */
  void Set(const adsfi_proto::internal::SubMap &hd_map, const V3 &ref_point);
  std::unordered_map<size_t, TrafficSign> traffic_sign_;
  using Ptr = std::shared_ptr<MapTrafficSign>;
};

}  // namespace loc
}  // namespace mp
}  // namespace hozon
