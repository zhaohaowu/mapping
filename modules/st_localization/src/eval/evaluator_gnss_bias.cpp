/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming<zhaoming@senseauto.com>
 */

#include "eval/evaluator_gnss_bias.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <iomanip>
#include <string>

namespace senseAD {
namespace localization {

adLocStatus_t EvaluatorGnssBias::WriteResult(double timestamp,
                                             const Eigen::Vector2d& data) {
  constexpr size_t kDataSize = 3;

  double output_arr[kDataSize];
  output_arr[0] = timestamp;
  output_arr[1] = data(0);
  output_arr[2] = data(1);

  output_stream_ << std::setprecision(6) << std::fixed;
  output_stream_.write(reinterpret_cast<char*>(output_arr),
                       sizeof(output_arr[0]) * kDataSize);
  output_stream_.flush();

  return LOC_SUCCESS;
}

}  // namespace localization
}  // namespace senseAD
