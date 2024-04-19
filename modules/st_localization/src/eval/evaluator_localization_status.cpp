/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming<zhaoming@senseauto.com>
 */

#include "eval/evaluator_localization_status.hpp"

#include <fstream>
#include <iomanip>

namespace senseAD {
namespace localization {

adLocStatus_t EvaluatorLocalizationStatus::WriteResult(
    double timestamp, const NavStatusEvalData& data) {
  constexpr size_t kDataSize = 8;

  double output_arr[kDataSize];
  output_arr[0] = timestamp;
  output_arr[1] = static_cast<double>(data.nav_status);
  output_arr[2] = static_cast<double>(data.frontend_status);
  output_arr[3] = static_cast<double>(data.backend_status);
  output_arr[4] = static_cast<double>(data.msf_status);
  output_arr[5] = static_cast<double>(data.output_status);
  output_arr[6] = static_cast<double>(data.gnss_status);
  output_arr[7] = static_cast<double>(data.smm_status);

  output_stream_ << std::setprecision(6) << std::fixed;
  output_stream_.write(reinterpret_cast<char*>(output_arr),
                       sizeof(output_arr[0]) * kDataSize);
  output_stream_.flush();

  return LOC_SUCCESS;
}

}  // namespace localization
}  // namespace senseAD
