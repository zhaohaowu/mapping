/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * Wang Xiaofeng <wangxiaofeng@sensetime.com>
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <list>

#include <boost/circular_buffer.hpp>

#include "localization/common/log.hpp"
#include "localization/data_type/base.hpp"
#include "localization/data_type/imu.hpp"

namespace senseAD {
namespace localization {

using Eigen::Vector3d;
using Vector6d = Eigen::Matrix<double, 6, 1>;

class StaticDetector {
 public:
  explicit StaticDetector(size_t imu_freq);
  ~StaticDetector() = default;

  adLocStatus_t detector_GLRT();
  adLocStatus_t detector_AMV(double var_threshold = 0.03);
  adLocStatus_t detector_AMAG();
  adLocStatus_t detector_ARE();
  adLocStatus_t detector_DP();

  adLocStatus_t addIMU(const Imu& raw_imu);
  void clearIMU() {
    imuz_.clear();
    mov_ave_.clear();
  }
  bool isFull() { return (imuz_.size() == zupt_ws_); }

 private:
  size_t imu_frequency_;  // imu frequency

  size_t zupt_ws_;  // about 1 second detection window
  boost::circular_buffer<Vector6d> imuz_;

  const size_t mov_ws_ = 3;  // 3 frames of moving average window
  boost::circular_buffer<Vector6d> mov_ave_;
};

}  // namespace localization
}  // namespace senseAD
