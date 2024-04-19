/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Fang Yu <fangyu1@senseauto.com>
 */

#include "eval/evaluator_motion.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <iomanip>
#include <string>

namespace senseAD {
namespace localization {

adLocStatus_t EvaluatorMotion::WriteResult(double timestamp,
                                           const MotionEvalData& data) {
  constexpr size_t kDataSize = 10;

  double output_arr[kDataSize];
  output_arr[0] = timestamp;
  for (int i = 0; i < 3; ++i) output_arr[i + 1] = data.linear_speed(i);
  for (int i = 0; i < 3; ++i) output_arr[i + 4] = data.angular_speed(i);
  for (int i = 0; i < 3; ++i) output_arr[i + 7] = data.linear_acceleration(i);

  output_stream_ << std::setprecision(6) << std::fixed;
  output_stream_.write(reinterpret_cast<char*>(output_arr),
                       sizeof(output_arr[0]) * kDataSize);
  output_stream_.flush();

  return LOC_SUCCESS;
}

}  // namespace localization
}  // namespace senseAD
