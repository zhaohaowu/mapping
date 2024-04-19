/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Du Jiankui <dujiankui@senseauto.com>
 */

#include "eval/evaluator_odometry.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <iomanip>
#include <string>

namespace senseAD {
namespace localization {

adLocStatus_t EvaluatorOdometry::WriteResult(double timestamp,
                                             const OdometryEvalData& data) {
  constexpr size_t kDataSize = 8;

  double output_arr[kDataSize];
  output_arr[0] = timestamp;
  output_arr[1] = data.gravity_align_confidence;
  output_arr[2] = data.roll_obs;
  output_arr[3] = data.pitch_obs;
  output_arr[4] = data.roll_cov;
  output_arr[5] = data.pitch_cov;
  output_arr[6] = data.velo_lat_cov;
  output_arr[7] = data.velo_vert_cov;

  output_stream_ << std::setprecision(6) << std::fixed;
  output_stream_.write(reinterpret_cast<char*>(output_arr),
                       sizeof(output_arr[0]) * kDataSize);
  output_stream_.flush();

  return LOC_SUCCESS;
}

}  // namespace localization
}  // namespace senseAD
