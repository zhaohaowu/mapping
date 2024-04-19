/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * lixin<lixin2@sensetime.com>
 * liwenqiang<liwenqiang1@sensetime.com>
 */

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <iomanip>
#include <string>

#include "eval/evaluator_localziation.hpp"

namespace senseAD {
namespace localization {

adLocStatus_t EvaluatorLocalization::WriteResult(double timestamp,
                                                 const Eigen::Matrix4d& data) {
  // Save in the form of TUM: timestamp, tx, ty, tz, qx, qy, qz, qw
  constexpr size_t kPoseSize = 8;

  Eigen::Quaterniond q(data.block<3, 3>(0, 0));
  q.normalize();
  Eigen::Vector3d t = data.block<3, 1>(0, 3);

  double output_arr[kPoseSize];
  output_arr[0] = timestamp;
  for (int i = 1; i < 4; i++) {
    output_arr[i] = t[i - 1];
  }
  output_arr[4] = q.x();
  output_arr[5] = q.y();
  output_arr[6] = q.z();
  output_arr[7] = q.w();

  output_stream_ << std::setprecision(6) << std::fixed;
  output_stream_.write(reinterpret_cast<char*>(output_arr),
                       sizeof(output_arr[0]) * kPoseSize);
  output_stream_.flush();

  return LOC_SUCCESS;
}

}  // namespace localization
}  // namespace senseAD
