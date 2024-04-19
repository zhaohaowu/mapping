/*
 * Copyright (C) 2023 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming<zhaoming@senseauto.com>
 */

#pragma once

#include <string>

#include "ad_common/config_utils/macros.hpp"

namespace senseAD {
namespace localization {

struct MsfSerialParam {
  uint64_t timestamp;
  std::string utc_time;
  double acc_bias_x;
  double acc_bias_y;
  double acc_bias_z;
  double gyro_bias_x;
  double gyro_bias_y;
  double gyro_bias_z;
  double acc_scale_x;
  double acc_scale_y;
  double acc_scale_z;
  double gyro_scale_x;
  double gyro_scale_y;
  double gyro_scale_z;
  double can_velocity_scale;
};

REGISTER_CEREAL_SERIALIZE(MsfSerialParam& msf_serial_param) {  // NOLINT
  CEREAL_PAIR(msf_serial_param, timestamp);
  CEREAL_PAIR(msf_serial_param, utc_time);
  CEREAL_PAIR(msf_serial_param, acc_bias_x);
  CEREAL_PAIR(msf_serial_param, acc_bias_y);
  CEREAL_PAIR(msf_serial_param, acc_bias_z);
  CEREAL_PAIR(msf_serial_param, gyro_bias_x);
  CEREAL_PAIR(msf_serial_param, gyro_bias_y);
  CEREAL_PAIR(msf_serial_param, gyro_bias_z);
  CEREAL_PAIR(msf_serial_param, acc_scale_x);
  CEREAL_PAIR(msf_serial_param, acc_scale_y);
  CEREAL_PAIR(msf_serial_param, acc_scale_z);
  CEREAL_PAIR(msf_serial_param, gyro_scale_x);
  CEREAL_PAIR(msf_serial_param, gyro_scale_y);
  CEREAL_PAIR(msf_serial_param, gyro_scale_z);
  CEREAL_PAIR(msf_serial_param, can_velocity_scale);
}

}  // namespace localization
}  // namespace senseAD
