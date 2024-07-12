/******************************************************************************
 Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved
 *Author: hozon
 *Date: 2024-07-08
 *****************************************************************************/
#include "map_fusion/map_fusion_test.h"

namespace hozon {
namespace mp {
namespace mf {
TEST_F(map_fusion_test, map_fusion_check) {
  std::cout << " map_fusion_test start " << std::endl;
  int res = map_prediction_->Init();
  EXPECT_EQ(res, 0);
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
