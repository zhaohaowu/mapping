/*
 * Copyright (C) 2019-2020 by SenseTime Group Limited. All rights reserved.
 * Chen Shengjie <chenshengjie@sensetime.com>
 * Wang Xiaofeng <wangxiaofeng@senseauto.com>
 *
 * Solution from a Inertial Navigation System (INS), which usually fuses GNSS
 * and IMU measurements.
 */
#pragma once

#include <map>
#include <string>

#include "localization/data_type/base.hpp"

namespace senseAD {
namespace localization {

enum class InsStatus {
  /**
   * Do NOT use.
   * Invalid solution due to insufficient observations, no initial GNSS, ...
   */
  INVALID = 0,

  /**
   * Use with caution. The covariance matrix may be unavailable or incorrect.
   * High-variance result due to aligning, insufficient vehicle dynamics, ...
   */
  CONVERGING = 100,

  // Safe to use. The INS has fully converged.
  GOOD = 200
};

typedef struct {
  /**
   * INS status
   */
  InsStatus status = InsStatus::INVALID;

  /**
   * The time of INS measurement, in seconds since the GPS epoch (01/06/1980).
   */
  float64_t measurement_time;

  /**
   * Position of the INS(the IMU most of the time) in the WGS84 reference
   * frame. Longitude and Latitude are in degrees, Height in meters.
   */
  PointLLH_t position;

  /**
   * Attitude of the IMU with respect to the reference frame in radians.
   * reference frame may be NED or ENU, depending on INS device
   */
  EulerAngles_t euler_angle;

  /**
   * Linear velocity of the IMU in the reference frame
   */
  Point3D_t linear_velocity;
} Ins;

static inline InsStatus GetInsStatusEnum(const std::string& status_str) {
  InsStatus status = InsStatus::INVALID;
  static const std::map<std::string, InsStatus> InsStatusTable{
      {"INVALID", InsStatus::INVALID},
      {"CONVERGING", InsStatus::CONVERGING},
      {"GOOD", InsStatus::GOOD}};
  const auto& it = InsStatusTable.find(status_str);
  if (it != InsStatusTable.end()) {
    status = it->second;
  }

  return status;
}

static inline std::string GetInsStatusStr(const InsStatus& status) {
  std::string status_str = "";
  static const std::map<InsStatus, std::string> InsStatusTable{
      {InsStatus::INVALID, "INVALID"},
      {InsStatus::CONVERGING, "CONVERGING"},
      {InsStatus::GOOD, "GOOD"}};
  const auto& it = InsStatusTable.find(status);
  if (it != InsStatusTable.end()) {
    status_str = it->second;
  }

  return status_str;
}

}  // namespace localization
}  // namespace senseAD
