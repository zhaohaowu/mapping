/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: likuan
 *Date: 2023-09-06
 *****************************************************************************/
#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <vector>

#include "opencv2/core/core.hpp"
#include "opencv2/features2d.hpp"

namespace hozon {
namespace mp {
namespace lm {

using Vec2d = Eigen::Vector2d;
using Vec3d = Eigen::Vector3d;
using Mat3d = Eigen::Matrix3d;
using Matxd = Eigen::MatrixXd;

using VecMatxd = std::vector<Matxd>;

using KDTreePtr = std::shared_ptr<cv::flann::Index>;

using LanePointsPtr = std::shared_ptr<std::vector<Eigen::Vector3d>>;

}  // namespace lm
}  // namespace mp
}  // namespace hozon
