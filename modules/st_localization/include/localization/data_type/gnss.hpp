/*
 * Copyright (C) 2019-2020 by SenseTime Group Limited. All rights reserved.
 * Chen Shengjie <chenshengjie@sensetime.com>
 * Wang Xiaofeng <wangxiaofeng@senseauto.com>
 *
 * Solution from a Global Navigation Satellite System (GNSS) receiver without
 * fused with any IMU.
 */
#pragma once

#include <map>
#include <string>

#include "localization/data_type/base.hpp"

namespace senseAD {
namespace localization {

enum class GnssStatus {
  /**
   * Not using the GNSS solution if status is INVALID or PROPAGATED.
   * INVALID solution due to insufficient observations, integrity warning...
   */
  INVALID = 0,
  // PROPAGATED by a Kalman filter without new observations.
  PROPAGATED,

  /**
   * Recommended using the following types of solution.
   * Standard GNSS solution without any corrections.
   */
  SINGLE = 100,
  // Pseudorange differential solution, including WAAS/SBAS solution.
  PSRDIFF,
  // Precise Point Positioning (PPP) solution.
  PPP,
  // Real Time Kinematic (RTK) float solution.
  RTK_FLOAT,
  // SINGLE, PSRDIFF, PPP, RTK_FLOAT or ... are OK, for compatibility.
  OK = 199,

  // RTK integer solution.
  RTK_INTEGER = 200,
  // RTK_INTEGER or ... are GOOD, for compatibility.
  GOOD = 299
};

typedef struct {
  GnssStatus status = GnssStatus::INVALID;

  /**
   * The time of position measurement, in seconds since the GPS epoch, which
   * is Jan 6, 1980.
   */
  float64_t measurement_time;

  /**
   * Position of the GNSS antenna phase center. in the WGS84 reference frame.
   * Longitude and Latitude are in degrees, Height in meters.
   */
  PointLLH_t position;
  // Position measurement uncertainty(East/north/up in meters).
  PointENU_t position_std_dev;

  // East/north/up in meters per second.
  PointENU_t linear_velocity;
  // velocity measurement uncertainty(East/north/up in meters per second).
  PointENU_t linear_velocity_std_dev;

  // Number of satellites in position solution.
  uint32_t num_sats;
} Gnss;

static inline GnssStatus GetGnssStatusEnum(const std::string& status_str) {
  GnssStatus status = GnssStatus::INVALID;
  static const std::map<std::string, GnssStatus> GnssStatusTable{
      {"INVALID", GnssStatus::INVALID},
      {"PROPAGATED", GnssStatus::PROPAGATED},
      {"SINGLE", GnssStatus::SINGLE},
      {"PSRDIFF", GnssStatus::PSRDIFF},
      {"PPP", GnssStatus::PPP},
      {"RTK_FLOAT", GnssStatus::RTK_FLOAT},
      {"OK", GnssStatus::OK},
      {"RTK_INTEGER", GnssStatus::RTK_INTEGER},
      {"GOOD", GnssStatus::GOOD}};
  const auto& it = GnssStatusTable.find(status_str);
  if (it != GnssStatusTable.end()) {
    status = it->second;
  }

  return status;
}

static inline std::string GetGnssStatusStr(const GnssStatus& status) {
  std::string status_str = "";
  static const std::map<GnssStatus, std::string> GnssStatusTable{
      {GnssStatus::INVALID, "INVALID"},
      {GnssStatus::PROPAGATED, "PROPAGATED"},
      {GnssStatus::SINGLE, "SINGLE"},
      {GnssStatus::PSRDIFF, "PSRDIFF"},
      {GnssStatus::PPP, "PPP"},
      {GnssStatus::RTK_FLOAT, "RTK_FLOAT"},
      {GnssStatus::OK, "OK"},
      {GnssStatus::RTK_INTEGER, "RTK_INTEGER"},
      {GnssStatus::GOOD, "GOOD"}};
  const auto& it = GnssStatusTable.find(status);
  if (it != GnssStatusTable.end()) {
    status_str = it->second;
  }

  return status_str;
}

}  // namespace localization
}  // namespace senseAD
