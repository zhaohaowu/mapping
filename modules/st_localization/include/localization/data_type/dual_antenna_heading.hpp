/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * Wang Xiaofeng <wangxiaofeng@senseauto.com>
 *
 * Synchronous heading information from GNSS dual antenna.
 */
#pragma once

#include <map>
#include <string>

#include "localization/data_type/base.hpp"
#include "localization/data_type/gnss.hpp"

namespace senseAD {
namespace localization {

typedef struct {
  GnssStatus status = GnssStatus::INVALID;

  /**
   * The time of position measurement, in seconds since the GPS epoch, which
   * is Jan 6, 1980.
   */
  float64_t measurement_time;

  /**
   * The heading is the angle from True North of the primary antenna to
   * secondary antenna vector in clockwise direction, in radians
   */
  float32_t heading;
  // heading standard deviation in degrees
  float32_t heading_std;

  // pitch angle in degrees
  float32_t pitch;
  // pitch standard deviation in degrees
  float32_t pitch_std;

  // Number of satellites in position solution.
  uint32_t num_sats;
} DualAntennaHeading;

}  // namespace localization
}  // namespace senseAD
