/******************************************************************************
 Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved
 *Author: hozon
 *Date: 2024-07-08
 *****************************************************************************/
#include "dr/dr_test.h"

#include <Eigen/Dense>

namespace hozon {
namespace mp {
namespace dr {
TEST_F(DrTest, dr_interface_check) {
  std::cout << " dr_function_check " << std::endl;
  if (!dr_interface_) {
    return;
  }
  // 1. check Process result
  bool res = dr_interface_->Process();
  EXPECT_TRUE(res);
}

TEST_F(DrTest, odom_2d_func_check) {
  std::cout << " dr_function_check " << std::endl;
  if (!odom_datas_) {
    return;
  }
  // 1. check CanInitialize
  bool res = odom_datas_->CanInitialize();
  EXPECT_TRUE(!res);

  // 2. check get_cross_mat
  Eigen::Vector3d w(1.0, 2.0, 3.0);
  Eigen::Matrix3d cross_mat = odom_datas_->get_cross_mat(w);
  Eigen::Matrix3d expected_a_mat;
  expected_a_mat << 0.0, -w(2), w(1), w(2), 0.0, -w(0), -w(1), w(0), 0.0;
  res = cross_mat.isApprox(expected_a_mat, 1e-6);
  if (res) {
    std::cout << "Element-wise verification passed." << std::endl;
  } else {
    std::cerr << "Element-wise verification failed!" << std::endl;
    std::cerr << "Expected matrix:\n" << expected_a_mat << std::endl;
    std::cerr << "Actual matrix:\n" << cross_mat << std::endl;
  }
  EXPECT_TRUE(res);

  // 3. check Initialize
  EXPECT_TRUE(odom_datas_->Initialize());
}

}  // namespace dr
}  // namespace mp
}  // namespace hozon
