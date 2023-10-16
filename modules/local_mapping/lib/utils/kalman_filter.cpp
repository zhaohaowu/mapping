/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: luerwei
 *Date: 2023-09-08
 *****************************************************************************/
#include "modules/local_mapping/lib/utils/kalman_filter.h"

namespace hozon {
namespace mp {
namespace lm {

KFFilter::KFFilter(const Eigen::Vector2d& pt) {
  F_ = Eigen::Matrix2d::Identity();  //
  H_ = Eigen::Matrix2d::Identity();
  Q_ = Eigen::Matrix2d::Identity() * 1e-5;  //
  R_ = Eigen::Matrix2d::Identity() * 1e-3;
  pre_P_ = Eigen::Matrix2d::Identity();
  post_P_ = Eigen::Matrix2d::Identity();
  post_x_ << pt.x(), pt.y();
}

Eigen::Vector2d KFFilter::Predict(double theta, const Eigen::Vector2d& T) {
  F_ << std::cos(theta), 0, 0, std::cos(theta);
  pre_x_ = TransposeFunction(post_x_, theta, T);
  pre_P_ = F_ * post_P_ * F_.transpose() + R_;
  return pre_x_;
}

Eigen::Vector3d KFFilter::Update(const Eigen::Vector2d& z) {
  Eigen::MatrixXd tmp = pre_P_ * H_.transpose();
  Eigen::MatrixXd K = tmp * (H_ * tmp + Q_).inverse();
  post_x_ = pre_x_ + K * (z - MeasureFunction(pre_x_));
  post_P_ = (Eigen::Matrix2d::Identity() - K * H_) * pre_P_;

  return Eigen::Vector3d(post_x_.x(), post_x_.y(), 0.0);
}

Eigen::Vector2d KFFilter::TransposeFunction(const Eigen::Vector2d& x_1,
                                            double theta,
                                            const Eigen::Vector2d& T) {
  Eigen::Matrix2d R;
  R << std::cos(theta), -std::sin(theta), std::sin(theta), std::cos(theta);
  return R * x_1 + T;
}

Eigen::Vector2d KFFilter::MeasureFunction(const Eigen::Vector2d& x) {
  return x;
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
