/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： hd_map.h
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <map>
#include <memory>
#include <unordered_map>
#include <vector>

#include <Sophus/se3.hpp>
#include <Sophus/so3.hpp>

#include "depend/proto/map/map.pb.h"
#include "modules/location/pose_estimation/lib/hd_map/hd_map_base.h"
#include "modules/location/pose_estimation/lib/hd_map/hd_map_lane_line.h"
#include "modules/location/pose_estimation/lib/hd_map/hd_map_pole.h"
#include "modules/location/pose_estimation/lib/hd_map/hd_map_road_edge.h"
#include "modules/location/pose_estimation/lib/hd_map/hd_map_traffic_sign.h"
#include "modules/util/include/util/geo.h"
#include "modules/util/include/util/mapping_log.h"

namespace hozon {
namespace mp {
namespace loc {

template <typename Map_Type>
class Map {
 public:
  /**
   * @brief set map element
   *
   * @param position : vehicle position
   * @param rotation : vehicle rotation
   * @param distance : range before vehicle
   * @param ref_point : reference point for transform coordinate
   *
   * @return
   */
  void SetMap(const std::vector<hozon::hdmap::LaneInfoConstPtr>& lane_ptr_vec,
              const Eigen::Vector3d& ref_point);

  /**
   * @brief crop map to get different element
   *
   * @param T_W_V : vehicle pose in the gcj02 coordinates
   * @param front : range in front of the vehicle
   * @param width : range in side of the vehicle
   * @return
   */
  void Crop(const SE3& T_W_V, double front, double width);

  /**
   * @brief crop map
   *
   * @param T_W_V : vehicle pose in the enu coordinates
   * @param front : range in front of the vehicle
   * @param width : range in side of the vehicle
   * @return
   */
  void BoxUpdate(const SE3& T_W_V, double front, double width);

  /**
   * @brief get different map element
   *
   * @param type : element type
   * @return
   */
  MapElement::Ptr GetElement(int type) const;

  /**
   * @brief free variable memory
   *
   * @return
   */
  void Clear(void);

  /**
   * @brief set refrence point
   *
   * @param ref_point : refrence point
   * @return
   */
  void set_ref_point(const V3& ref_point);

  /**
   * @brief get refrence point
   *
   * @param
   * @return
   */
  V3 get_ref_point(void);
  std::vector<V3> box_;
  std::vector<MapElement::Ptr> elment_;

 private:
  V3 ref_point_;
};

}  // namespace loc
}  // namespace mp
}  // namespace hozon
