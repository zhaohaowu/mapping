/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： smoother.h
 *   author     ： nihongjie
 *   date       ： 2023.09
 ******************************************************************************/
#pragma once

#include <Eigen/Eigen>
#include <deque>

namespace hozon {
namespace mp {
namespace loc {

class Smoother {
 public:
  Smoother(const uint32_t& window_size, const double& gcj02_enu_east_diff_thr,
           const double& gcj02_enu_north_diff_thr,
           const double& gcj02_enu_norm_diff_thr, const double& momentum);
  ~Smoother() = default;

  void SetSmoothInputData(const Eigen::Matrix<double, 3, 1>& err_pos_enu,
                          const Eigen::Matrix<double, 3, 1>& pos_gcj02_enu);
  bool OutputReady() const;
  const Eigen::Matrix<double, 3, 1>& GetSmoothOutputData() const;

 private:
  bool EnuInit();
  void UpdateEnuError();

 private:
  bool output_ready_ = false;
  uint32_t window_size_ = 0;
  double gcj02_enu_east_diff_thr_ = 0.0;
  double gcj02_enu_north_diff_thr_ = 0.0;
  double gcj02_enu_norm_diff_thr_ = 0.0;
  double momentum_ = 0.99;

  Eigen::Matrix<double, 3, 1> curr_err_pos_enu_;
  Eigen::Matrix<double, 3, 1> curr_pos_gcj02_enu_;
  Eigen::Matrix<double, 3, 1> smooth_err_pos_enu_;
  Eigen::Matrix<double, 3, 1> last_pos_gcj02_enu_;

  bool enu_init_ = false;
  std::deque<Eigen::Matrix<double, 3, 1>> err_enu_deque_;
  Eigen::Matrix<double, 3, 1> err_enu_sum_;
  Eigen::Matrix<double, 3, 1> err_enu_mean_;
};

}  // namespace loc
}  // namespace mp
}  // namespace hozon
