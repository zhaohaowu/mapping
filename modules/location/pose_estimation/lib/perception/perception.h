/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： perception.h
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <interface/adsfi_proto/perception/lanes.pb.h>
#include <interface/adsfi_proto/perception/object.pb.h>
#include <math.h>

#include <iostream>
#include <list>
#include <memory>
#include <vector>

#include "modules/location/pose_estimation/lib/perception/perception_base.h"
#include "modules/location/pose_estimation/lib/perception/perception_lane_line.h"
#include "modules/location/pose_estimation/lib/util/poly_line.h"
#include "modules/util/include/util/temp_log.h"
// #include "modules/location/pose_estimation/lib/perception/perception_pole.h"
// #include
// "modules/location/pose_estimation/lib/perception/perception_road_edge.h"
// #include
// "modules/location/pose_estimation/lib/perception/perception_traffic_sign.h"

namespace hozon {
namespace mp {
namespace loc {
class Perception {
 public:
  Perception();
  Perception(
      const ::adsfi_proto::hz_Adsfi::AlgLaneDetectionOutArray &lane_lines);
  /**
   * @brief add perception element
   *
   * @param lane_line : perception lane line and road edge
   * @return
   */
  void Set(const ::adsfi_proto::hz_Adsfi::AlgLaneDetectionOutArray &lane_lines);

  /**
   * @brief add perception lane line element
   *
   * @param lane_line : perception lane line
   * @return
   */
  void SetLaneLineList(
      const ::adsfi_proto::hz_Adsfi::AlgLaneDetectionOutArray &lane_lines);

  /**
   * @brief add perception road edge element
   *
   * @param lane_line : perception lane line
   * @return
   */
  // void SetRoadEdgeList(const ::adsfi_proto::hz_Adsfi::AlgLaneDetectionOut
  // &lane_line);

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
