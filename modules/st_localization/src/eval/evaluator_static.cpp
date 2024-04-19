/*
 * Copyright (C) 2023 by SenseTime Group Limited. All rights reserved.
 * Lei Bing<leibing@senseauto.com>
 */

#include "eval/evaluator_static.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <iomanip>
#include <string>

namespace senseAD {
namespace localization {

adLocStatus_t EvaluatorStatic::WriteResult(double timestamp,
                                           const double& data) {
  constexpr size_t kDataSize = 2;

  double output_arr[kDataSize];
  output_arr[0] = data;
  output_arr[1] = timestamp;

  output_stream_ << std::setprecision(6) << std::fixed;
  output_stream_.write(reinterpret_cast<char*>(output_arr),
                       sizeof(output_arr[0]) * kDataSize);
  output_stream_.flush();

  return LOC_SUCCESS;
}

}  // namespace localization
}  // namespace senseAD
