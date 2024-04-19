/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * Wang Xiaofeng <wangxiaofeng@sensetime.com>
 */
#pragma once

#include <map>
#include <string>
#include <vector>

#include "localization/data_type/base.hpp"
#include "localization/data_type/gnss.hpp"
#include "localization/data_type/ins.hpp"

namespace senseAD {
namespace localization {

typedef struct {
  // The time of pose measurement, in nano second.
  uint64_t measurement_time_ns;

  // localization msf status
  NavStatus nav_status;

  // Position of the origin in the WGS84 Coordinate.
  PointLLH_t origin_position_lla;

  // Position of the Vehicle Reference Point(VRP) in the Map reference
  // frame(East/North/Up). The VRP is the car center(Forward/Left/Up).
  // x for East, y for North, z for Up Height, in meters.
  PointENU_t position;

  /** Position standard deviation in meters, represent in the Vehicle
   * reference frame **/
  Point3D_t position_std;

  /**
   * Position of the Vehicle Reference Point(VRP) in the WGS84 Coordinate.
   */
  PointLLH_t position_lla;

  // Attitude in euler angle form to describe the orientation of a VRP frame
  // with respect to a Map reference frame(ENU).
  EulerAngles_t euler_angle;

  /** Attitude standard deviation in radians **/
  Point3D_t attitude_std;

  /**
   * Linear velocity of the VRP in the Vehicle reference frame
   * x for Forward, y for Left, z for Up, in meters per second
   */
  Point3D_t linear_velocity;

  /**
   * Linear velocity of the VRP in the ENU reference frame
   * x for East, y for North, z for Up, in meters per second
   */
  Point3D_t global_linear_velocity;

  /**
   * Linear acceleration of the VRP in the Vehicle reference frame
   * x for Forward, y for Left, z for Up, in meters per power second
   */
  Point3D_t linear_acceleration;

  /**
   * Linear acceleration of the VRP in the ENU reference frame
   * x for East, y for North, z for Up, in meters per power second
   */
  Point3D_t global_linear_acceleration;

  /**
   * Angular velocity of the VRP in the Vehicle reference frame
   * x across Forward axes, y across Left axes,
   * z across Up axes, in radians per second
   */
  Point3D_t angular_velocity;
} NavStateInfo;

}  // namespace localization
}  // namespace senseAD
