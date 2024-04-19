/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming<zhaoming@senseauto.com>
 */

#include "eval/evaluator_imu_intrinsic.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <iomanip>
#include <string>

namespace senseAD {
namespace localization {

adLocStatus_t EvaluatorImuIntrinsic::WriteResult(
    double timestamp, const ImuIntrinsicEvalData& data) {
  constexpr size_t kDataSize = 13;

  double output_arr[kDataSize];
  output_arr[0] = timestamp;

  for (int i = 0; i < 3; ++i) output_arr[i + 1] = data.acc_bias(i);
  for (int i = 0; i < 3; ++i) output_arr[i + 4] = data.gyro_bias(i);
  for (int i = 0; i < 3; ++i) output_arr[i + 7] = data.acc_scale(i);
  for (int i = 0; i < 3; ++i) output_arr[i + 10] = data.gyro_scale(i);

  output_stream_ << std::setprecision(6) << std::fixed;
  output_stream_.write(reinterpret_cast<char*>(output_arr),
                       sizeof(output_arr[0]) * kDataSize);
  output_stream_.flush();

  return LOC_SUCCESS;
}

}  // namespace localization
}  // namespace senseAD
