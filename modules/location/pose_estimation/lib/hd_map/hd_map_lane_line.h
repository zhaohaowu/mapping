/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： hd_map_lane_line.h
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

#include "depend/common/utm_projection/coordinate_convertor.h"
#include "depend/proto/common/types.pb.h"
#include "modules/location/pose_estimation/lib/hd_map/hd_map_base.h"
#include "modules/map_fusion/modules/map_hd/global_hd_map.h"
#include "modules/util/include/util/geo.h"

namespace hozon {
namespace mp {
namespace loc {

struct BoundaryLine {
  size_t line_type;
  size_t id_center;
  std::string id_boundary;
  std::vector<ControlPoint> control_point;
};

class MapBoundaryLine : public MapElement {
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
  void Print(
      const std::unordered_map<std::string, BoundaryLine>& boundarylines);
  /**
   * @brief set map lane line element
   *
   * @param position : vehicle position
   * @param rotation : vehicle rotation
   * @param distance : range before vehicle
   * @param ref_point : reference point for transform coordinate
   *
   * @return
   */
  void Set(const std::vector<hozon::hdmap::LaneInfoConstPtr>& lane_ptr_vec,
           const V3& ref_point);
  bool AddMapLine(const hozon::hdmap::LaneBoundary& lane_boundary,
                  const V3& ref_point);
  using Ptr = std::shared_ptr<MapBoundaryLine>;
  std::unordered_map<std::string, BoundaryLine> boundary_line_;
};

}  // namespace loc
}  // namespace mp
}  // namespace hozon
