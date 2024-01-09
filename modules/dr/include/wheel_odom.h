/******************************************************************************
 * Copyright (C) 2022 HOZON-AUTO Ltd. All rights reserved.
 * Licensed Hozon
 * Author: pengwei
 ******************************************************************************/
#pragma once

#include <deque>
// #include <iostream>
#include <memory>
// #include <iomanip>
// #include <limits>

#include "depend/perception-base/base/utils/log.h"
#include "modules/dr/include/odometry_base.h"
#include "proto/soc/chassis.pb.h"

namespace hozon {
namespace mp {
namespace dr {

class WheelOdom : public OdometryBase {
 public:
  WheelOdom(const std::string& conf_path)
      : OdometryBase(conf_path),
        pos_({0, 0, 0}),
        qat_(1, 0, 0, 0),
        vel_(0, 0, 0) {}

  ~WheelOdom() {}

  virtual bool update();

  bool process_wheel(const WheelDataHozon& last, const WheelDataHozon& cur);

 private:
  int mode_;
  Eigen::Vector3d pos_;
  Eigen::Quaterniond qat_;
  Eigen::Vector3d vel_;

  double v_by_wheel_;
  double w_by_wheel_;
};

}  // namespace dr
}  // namespace mp
}  // namespace hozon
