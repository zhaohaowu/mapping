/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： smoother.cpp
 *   author     ： nihongjie
 *   date       ： 2023.09
 ******************************************************************************/
#include "modules/location/ins_fusion/lib/smoother.h"

namespace hozon {
namespace mp {
namespace loc {

Smoother::Smoother(const uint32_t& window_size,
                   const double& gcj02_enu_east_diff_thr,
                   const double& gcj02_enu_north_diff_thr,
                   const double& gcj02_enu_norm_diff_thr,
                   const double& momentum) {
  window_size_ = window_size;
  gcj02_enu_east_diff_thr_ = gcj02_enu_east_diff_thr;
  gcj02_enu_north_diff_thr_ = gcj02_enu_north_diff_thr;
  gcj02_enu_norm_diff_thr_ = gcj02_enu_norm_diff_thr;
  momentum_ = momentum;

  curr_err_pos_enu_ = Eigen::MatrixXd::Zero(3, 1);
  curr_pos_gcj02_enu_ = Eigen::MatrixXd::Zero(3, 1);
  smooth_err_pos_enu_ = Eigen::MatrixXd::Zero(3, 1);
  last_pos_gcj02_enu_ = Eigen::MatrixXd::Zero(3, 1);

  err_enu_deque_.clear();
  err_enu_sum_ = Eigen::MatrixXd::Zero(3, 1);
  err_enu_mean_ = Eigen::MatrixXd::Zero(3, 1);
}

void Smoother::SetSmoothInputData(
    const Eigen::Matrix<double, 3, 1>& err_pos_enu,
    const Eigen::Matrix<double, 3, 1>& pos_gcj02_enu) {
  curr_err_pos_enu_ = err_pos_enu;
  curr_pos_gcj02_enu_ = pos_gcj02_enu;

  if (EnuInit()) {
    UpdateEnuError();
  }

  last_pos_gcj02_enu_ = curr_pos_gcj02_enu_;
}

bool Smoother::EnuInit() {
  if (enu_init_) {
    return true;
  }

  double gcj02_enu_east_diff =
      fabs(curr_pos_gcj02_enu_(0) - last_pos_gcj02_enu_(0));
  double gcj02_enu_north_diff =
      fabs(curr_pos_gcj02_enu_(1) - last_pos_gcj02_enu_(1));
  if (gcj02_enu_east_diff > gcj02_enu_east_diff_thr_ ||
      gcj02_enu_north_diff > gcj02_enu_north_diff_thr_) {
    err_enu_deque_.push_back(curr_err_pos_enu_);
  }

  if (err_enu_deque_.size() > window_size_) {
    err_enu_sum_ = Eigen::MatrixXd::Zero(3, 1);
    for (int i = 0; i < err_enu_deque_.size(); ++i) {
      err_enu_sum_ += err_enu_deque_[i];
    }
    err_enu_mean_ =
        err_enu_sum_ / static_cast<double>(err_enu_deque_.size() + 1);
    smooth_err_pos_enu_ = err_enu_mean_;
    enu_init_ = true;
    output_ready_ = true;
  }

  return enu_init_;
}

void Smoother::UpdateEnuError() {
  if (err_enu_deque_.empty()) {
    return;
  }

  const Eigen::Matrix<double, 3, 1> gcj02_enu_diff =
      curr_pos_gcj02_enu_ - last_pos_gcj02_enu_;
  if (gcj02_enu_diff.norm() < gcj02_enu_norm_diff_thr_) {
    return;
  }

  const Eigen::Matrix<double, 3, 1> front = err_enu_deque_.front();
  err_enu_deque_.pop_front();
  err_enu_deque_.push_back(curr_err_pos_enu_);

  // update sum and mean
  err_enu_sum_ -= front;
  err_enu_sum_ += err_enu_deque_.back();
  err_enu_mean_ = err_enu_sum_ / static_cast<double>(err_enu_deque_.size());
  smooth_err_pos_enu_ =
      smooth_err_pos_enu_ * momentum_ + err_enu_mean_ * (1.0 - momentum_);
}

bool Smoother::OutputReady() const { return output_ready_; }

const Eigen::Matrix<double, 3, 1>& Smoother::GetSmoothOutputData() const {
  return smooth_err_pos_enu_;
}

}  // namespace loc
}  // namespace mp
}  // namespace hozon
