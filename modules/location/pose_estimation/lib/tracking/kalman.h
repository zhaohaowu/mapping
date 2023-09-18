/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： kalman.h
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <Eigen/Core>
#include <iostream>

#include "modules/util/include/util/temp_log.h"

namespace hozon {
namespace mp {
namespace loc {

class Kalman {
 public:
#define DIMX 4
  Kalman(const Eigen::Matrix<double, DIMX, 1> &x,
         const Eigen::Matrix<double, DIMX, 1> &p) {
    init(x, p);
  }
  Kalman() {}
  void init(const Eigen::Matrix<double, DIMX, 1> &x,
            const Eigen::Matrix<double, DIMX, 1> &p) {
    p_.setZero();
    x_ = x;
    p_.diagonal() = p;
  }
  void predict(double dt) {
    Eigen::Matrix<double, 4, 4> A;
    A << 1.0, 0, dt, 0, 0, 1.0, 0, dt, 0, 0, 1.0, 0, 0, 0, 0, 1.0;

    Eigen::Matrix<double, 4, 4> Q;
    Q << 50.0, 0, 0, 0, 0, 50.0, 0, 0, 0, 0, 100.0, 0, 0, 0, 0, 100.0;
    // step 1
    x_ = A * x_;
    // step 2
    p_ = A * p_ * A.transpose() + Q;
  }
  void update(const Eigen::Vector2d &z, double dt) {
    Eigen::Matrix4d I;
    I.setIdentity();
    Eigen::Matrix<double, 2, 2> R;
    R << 0.1, 0, 0, 0.1;
    Eigen::Matrix<double, 2, 4> H;
    H << 1.0, 0, dt, 0, 0, 1.0, 0, dt;
    // step 3
    Eigen::Matrix<double, 2, 2> temp = (H * p_ * H.transpose() + R);
    Eigen::Matrix<double, 4, 2> K = p_ * H.transpose() * temp.inverse();
    // step 4
    Eigen::Matrix<double, 4, 1> x_hat = x_ + K * (z - H * x_);
    Eigen::Matrix<double, 4, 4> p_hat = (I - K * H) * p_;
    x_ = x_hat;
    // step 5
    p_ = p_hat;
  }
  Eigen::Matrix<double, DIMX, 1> x() { return x_; }
  Eigen::Matrix<double, DIMX, 1> x_;
  Eigen::Matrix<double, DIMX, DIMX> p_;
};

}  // namespace loc
}  // namespace mp
}  // namespace hozon
