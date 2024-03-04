/*================================================================
*   Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
*   file       ：laneline_struct.h
*   author     ：WuXiaopeng
*   date       ：2021.11.11
================================================================*/

#pragma once

#include <memory>

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "Sophus/se3.hpp"

namespace hozon {
namespace mp {
namespace lm2 {

class DrData {
 public:
  double timestamp_{};
  Eigen::Vector3d pose{};
  Eigen::Quaterniond quaternion{};
  Eigen::Vector3d local_vel{};
  Eigen::Vector3d local_omg{};
  Eigen::Vector3d local_acc{};
  int gear = 100;

  DrData Interpolate(const double scale, const DrData& end,
                     double timestamp) const {
    DrData res;
    res.timestamp_ = timestamp;
    res.pose = this->pose + (end.pose - this->pose) * scale;
    res.quaternion = this->quaternion.slerp(scale, end.quaternion);

    if (scale >= 0.5) {
      res.local_vel = end.local_vel;
      res.local_omg = end.local_omg;
      res.local_acc = end.local_acc;
      res.gear = end.gear;
    } else {
      res.local_vel = this->local_vel;
      res.local_omg = this->local_omg;
      res.local_acc = this->local_acc;
      res.gear = this->gear;
    }
    return res;
  }
};

using DrDataPtr = std::shared_ptr<DrData>;
using DrDataConstPtr = std::shared_ptr<const DrData>;

}  // namespace lm2
}  // namespace mp
}  // namespace hozon
