/******************************************************************************
 Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved
 *Author: hozon
 *Date: 2024-07-08
 *****************************************************************************/
#include "location/fusion_center/fusion_center_test.h"

namespace hozon {
namespace mp {
namespace loc {
namespace fc {
TEST_F(fusion_center_test, fusion_center_check) {
  std::cout << " local_mapping_test start " << std::endl;
  bool res = fc_center_->Init(fc_config_, fc_kf_config_, fc_eskf_config_,
                              fc_monitor_config_);
  EXPECT_TRUE(res);

  // LoadParams
  res = fc_center_->LoadParams(fc_config_);
  EXPECT_TRUE(res);
}

}  // namespace fc
}  // namespace loc
}  // namespace mp
}  // namespace hozon
