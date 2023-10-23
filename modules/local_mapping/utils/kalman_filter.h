/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: luerwei
 *Date: 2023-09-08
 *****************************************************************************/
#pragma once

#include <Eigen/Dense>

namespace hozon {
namespace mp {
namespace lm {

class KFFilter {
 public:
  explicit KFFilter(const Eigen::Vector2d& pt);

  Eigen::Vector2d Predict(double theta, const Eigen::Vector2d& T);

  Eigen::Vector3d Update(const Eigen::Vector2d& pt);

 private:
  Eigen::Vector2d TransposeFunction(const Eigen::Vector2d& x_1, double theta,
                                    const Eigen::Vector2d& T);
  Eigen::Vector2d MeasureFunction(const Eigen::Vector2d& x);

 private:
  const int stateNum_ = 2;
  const int measureNum_ = 2;

  Eigen::Matrix2d F_;
  Eigen::MatrixXd H_;
  Eigen::Matrix2d Q_;
  Eigen::Matrix2d R_;  // 2d
  Eigen::Matrix2d pre_P_;
  Eigen::Matrix2d post_P_;
  Eigen::Vector2d pre_x_;
  Eigen::Vector2d post_x_;
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
