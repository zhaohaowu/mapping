/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * Chen Shengjie <chenshengjie@sensetime.com>
 * Wang Xiaofeng <wangxiaofeng@senseauto.com>
 *
 * Measurements from an Inertial Measurement Unit (IMU).
 * The solution is with respect to the IMU by default.
 */
#pragma once

#include <map>
#include <string>

#include "localization/data_type/base.hpp"

namespace senseAD {
namespace localization {

enum class ImuStatus {
  // Simple IMU error flag
  NOT_GOOD = 0,

  GOOD = 100
};

typedef struct {
  /**
   * IMU status
   */
  ImuStatus status = ImuStatus::NOT_GOOD;

  /**
   * The time of IMU measurement, in seconds
   */
  float64_t measurement_time;

  /**
   * Linear acceleration of the IMU in the IMU frame.
   * in meters per power second
   */
  Point3D_t linear_acceleration;

  /**
   * Angular velocity of the IMU in the IMU frame
   * in radians per second
   */
  Point3D_t angular_velocity;
} Imu;

static inline ImuStatus GetImuStatusEnum(const std::string& status_str) {
  ImuStatus status = ImuStatus::NOT_GOOD;
  static const std::map<std::string, ImuStatus> ImuStatusTable{
      {"NOT_GOOD", ImuStatus::NOT_GOOD}, {"GOOD", ImuStatus::GOOD}};
  const auto& it = ImuStatusTable.find(status_str);
  if (it != ImuStatusTable.end()) {
    status = it->second;
  }

  return status;
}

}  // namespace localization
}  // namespace senseAD
