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
#include "Eigen/src/Geometry/Transform.h"
#include "Sophus/se3.hpp"

namespace hozon {
namespace mp {
namespace lm {

class DrData {
 public:
  double timestamp = -1.0;
  Eigen::Vector3d translation; // 平移
  Eigen::Quaterniond quaternion; // 旋转
  Eigen::Affine3d pose; // 平移+旋转
  Eigen::Vector3d linear_velocity; // 线速度
  Eigen::Vector3d angular_velocity; // 角速度
  Eigen::Vector3d acceleration; // 加速度
  int gear = 100; 

  DrData Interpolate(const double scale, const DrData& end,
                     double timestamp) const {
    DrData res;
    res.timestamp = timestamp;
    res.translation = this->translation + (end.translation - this->translation) * scale;
    res.quaternion = this->quaternion.slerp(scale, end.quaternion);
    res.pose = Eigen::Translation3d(res.translation) * Eigen::Affine3d(res.quaternion);

    if (scale >= 0.5) {
      res.linear_velocity = end.linear_velocity;
      res.angular_velocity = end.angular_velocity;
      res.acceleration = end.acceleration;
      res.gear = end.gear;
    } else {
      res.linear_velocity = this->linear_velocity;
      res.angular_velocity = this->angular_velocity;
      res.acceleration = this->acceleration;
      res.gear = this->gear;
    }
    return res;
  }
};

using DrDataPtr = std::shared_ptr<DrData>;
using DrDataConstPtr = std::shared_ptr<const DrData>;

}  // namespace lm
}  // namespace mp
}  // namespace hozon
