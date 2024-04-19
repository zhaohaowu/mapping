/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Lei Bing<leibing@senseauto.com>
 */

#include "eval/evaluator_gnss_fault.hpp"

#include <fstream>
#include <iomanip>

#include "ad_common/data_type/base.hpp"
#include "common/coordinate_converter.hpp"

namespace senseAD {
namespace localization {

adLocStatus_t EvaluatorGnssFault::WriteResult(double timestamp,
                                              const GnssFaultData& data) {
  constexpr size_t kDataSize = 18;

  double output_arr[kDataSize];
  output_arr[0] = timestamp;
  output_arr[1] = data.fault_type;
  output_arr[2] = data.gnss_dr_rate;
  output_arr[3] = data.gnss_dr_consistency;
  // lla to enu
  PointLLH_t lla;
  lla.lat = data.gnss_data.position.lat;
  lla.lon = data.gnss_data.position.lon;
  lla.height = data.gnss_data.position.height;
  PointENU_t enu;
  CoordinateConverter::GetInstance()->LLA2ENU(lla, &enu);
  // px, py, pz
  output_arr[4] = enu.x;
  output_arr[5] = enu.y;
  output_arr[6] = enu.z;
  output_arr[7] = data.gnss_data.position_std_dev.x;
  output_arr[8] = data.gnss_data.position_std_dev.y;
  output_arr[9] = data.gnss_data.position_std_dev.z;
  // vx, vy, vz
  output_arr[10] = data.gnss_data.linear_velocity.x;
  output_arr[11] = data.gnss_data.linear_velocity.y;
  output_arr[12] = data.gnss_data.linear_velocity.z;
  output_arr[13] = data.gnss_data.linear_velocity_std_dev.x;
  output_arr[14] = data.gnss_data.linear_velocity_std_dev.y;
  output_arr[15] = data.gnss_data.linear_velocity_std_dev.z;

  // sat num
  output_arr[16] = 0;

  // gnss status
  output_arr[17] = static_cast<double>(data.gnss_data.status);

  output_stream_ << std::setprecision(6) << std::fixed;
  output_stream_.write(reinterpret_cast<char*>(output_arr),
                       sizeof(output_arr[0]) * kDataSize);
  output_stream_.flush();

  return LOC_SUCCESS;
}

}  // namespace localization
}  // namespace senseAD
