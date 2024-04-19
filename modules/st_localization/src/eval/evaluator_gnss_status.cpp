/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming<zhaoming@senseauto.com>
 */

#include "eval/evaluator_gnss_status.hpp"

#include <fstream>
#include <iomanip>

namespace senseAD {
namespace localization {

adLocStatus_t EvaluatorGnssStatus::WriteResult(double timestamp,
                                               const uint16_t& data) {
  constexpr size_t kDataSize = 2;

  double output_arr[kDataSize];
  output_arr[0] = timestamp;
  output_arr[1] = static_cast<double>(data);

  output_stream_ << std::setprecision(6) << std::fixed;
  output_stream_.write(reinterpret_cast<char*>(output_arr),
                       sizeof(output_arr[0]) * kDataSize);
  output_stream_.flush();

  return LOC_SUCCESS;
}

}  // namespace localization
}  // namespace senseAD
