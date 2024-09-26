/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： perception.h
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <math.h>

#include <iostream>
#include <list>
#include <memory>
#include <vector>

#include "depend/proto/local_mapping/local_map.pb.h"
#include "modules/location/pose_estimation/lib/perception/perception_base.h"
#include "modules/location/pose_estimation/lib/perception/perception_lane_line.h"
#include "modules/location/pose_estimation/lib/util/poly_line.h"
#include "modules/util/include/util/mapping_log.h"
#include "proto/perception/transport_element.pb.h"
// #include
// "modules/location/pose_estimation/lib/perception/perception_road_edge.h"
// #include
//     "modules/location/pose_estimation/lib/perception/perception_traffic_sign.h"
// #include <interface/adsfi_proto/perception/object.pb.h>
// #include "modules/location/pose_estimation/lib/perception/perception_pole.h"

namespace hozon {
namespace mp {
namespace loc {
class Perception {
 public:
  Perception();
  explicit Perception(const ::hozon::mapping::LocalMap& transport_element);
  /**
   * @brief add perception element
   *
   * @param lane_line : perception lane line and road edge
   * @return
   */
  void Set(const ::hozon::mapping::LocalMap& transport_element);

  /**
   * @brief add perception lane line element
   *
   * @param lane_line : perception lane line
   * @return
   */
  void SetLaneLineList(const ::hozon::mapping::LocalMap& transport_element);

  /**
   * @brief add perception road edge element
   *
   * @param lane_line : perception lane line
   * @return
   */
  // void SetRoadEdgeList(const ::hozon::mapping::LocalMap
  // &transport_element);

  /**
   * @brief get perception element
   *
   * @param type : element type
   * @return
   */
  std::vector<PerceptionElement::Ptr> GetElement(int type) const;
  std::list<PerceptionElement::Ptr> element_;
};

}  // namespace loc
}  // namespace mp
}  // namespace hozon
