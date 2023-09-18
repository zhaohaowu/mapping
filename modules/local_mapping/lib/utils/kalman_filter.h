/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: luerwei
 *Date: 2023-09-08
 *****************************************************************************/
#pragma once

#include <Eigen/Dense>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>

namespace hozon {
namespace mp {
namespace lm {

class KFFilter {
 public:
  explicit KFFilter(const Eigen::Vector2d& pt);

  Eigen::Vector2d Predict();

  void Update(const Eigen::Vector2d& pt);

 private:
  const int stateNum_ = 4;
  const int measureNum_ = 2;
  cv::KalmanFilter kf_;
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
