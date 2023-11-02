
/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： perception_lane_line.h
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <math.h>

#include <iostream>
#include <list>
#include <memory>

#include "modules/location/pose_estimation/lib/perception/perception_base.h"
#include "modules/location/pose_estimation/lib/tracking/kalman.h"
#include "modules/location/pose_estimation/lib/util/globals.h"
#include "modules/location/pose_estimation/lib/util/poly_line.h"
#include "modules/util/include/util/temp_log.h"
#include "proto/perception/transport_element.pb.h"

namespace hozon {
namespace mp {
namespace loc {

class PerceptionLaneLine {
 public:
  PerceptionLaneLine();
  explicit PerceptionLaneLine(const hozon::perception::LaneInfo &lane_line);
  /**
   * @brief determine whether the point is on the curve equation
   *
   * @param x : point's x value
   * @return true : in ; false : out
   */
  bool IsIn(const float x);

  /**
   * @brief get the point's y value
   *
   * @param x : point's x value
   * @return point's y value
   */
  float Y(const float x);

  /**
   * @brief get the curve's confidence
   *
   * @return curve's confidence
   */
  float Probality();

  /**
   * @brief get the point's differential
   *
   * @param x : point's x value
   * @return differential value
   */
  float Theta(const float &x);

  /**
   * @brief get the lane line view range start
   *
   * @return view range start
   */
  float Min(void);

  /**
   * @brief get the lane line view range end
   *
   * @return view range end
   */
  float Max(void);

  /**
   * @brief print perception lane line data
   *
   * @return
   */
  void Print();

  /**
   * @brief get perception lane line id
   *
   * @return lane line id
   */
  int Id();

  PolyLine<LaneLine> curve_vehicle_coord_;
  using Ptr = std::shared_ptr<PerceptionLaneLine>;
  int track_id_ = 0;
  int track_state_ = -1;
  int track_num_ = 0;
  int pred_num_ = 0;
  Kalman track_kf_;
};

class PerceptionLaneLineList : public PerceptionElement {
 public:
  PerceptionLaneLineList(
      const hozon::perception::TransportElement &transport_element);
  PerceptionLaneLineList();

  /**
   * @brief print perception lane lines data
   *
   * @return
   */
  void Print(void);
  using Ptr = std::shared_ptr<PerceptionLaneLineList>;
  std::list<std::shared_ptr<PerceptionLaneLine>> lane_line_list_;
};

}  // namespace loc
}  // namespace mp
}  // namespace hozon
