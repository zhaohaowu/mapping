/*
 * Copyright (C) 2021 by SenseTime Group Limited. All rights reserved.
 * Du Jiankui <dujiankui@senseauto.com>
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

  // dead reckoning status
  OdomStatus odom_status;

  // The origin id is to represent the number of trajectories, start from 0,
  // plus by 1 if dr restart occurs
  uint64_t origin_id;

  // Position of the Vehicle Reference Point(VRP) in the Odometry reference
  // frame(Forward/Left/Up).
  // The VRP is the car center(Forward/Left/Up).
  // Odometry reference frame's origin is the VRP of start point.
  // x for Forward, y for Left, z for Up Height, in meters.
  Point3D_t position;

  /** Position standard deviation in meters, represent in the Vehicle
   * reference frame **/
  Point3D_t position_std;

  // Attitude in euler angle form to describe the orientation of a VRP frame
  // with respect to a Odometry reference frame(Forward/Left/Up).
  EulerAngles_t euler_angle;

  /** Attitude standard deviation in radians **/
  Point3D_t attitude_std;

  /**
   * Linear velocity of the VRP in the Vehicle reference frame
   * x for Forward, y for Left, z for Up, in meters per second
   */
  Point3D_t linear_velocity;

  /**
   * Linear acceleration of the VRP in the Vehicle reference frame
   * x for Forward, y for Left, z for Up, in meters per power second
   */
  Point3D_t linear_acceleration;

  /**
   * Angular velocity of the VRP in the Vehicle reference frame
   * x across Forward axes, y across Left axes,
   * z across Up axes, in radians per second
   */
  Point3D_t angular_velocity;
} OdomStateInfo;

}  // namespace localization
}  // namespace senseAD
