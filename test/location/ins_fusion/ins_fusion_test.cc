/******************************************************************************
 Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved
 *Author: hozon
 *Date: 2024-07-08
 *****************************************************************************/
#include "location/ins_fusion/ins_fusion_test.h"
#include "location/ins_fusion/lib/defines.h"

namespace hozon {
namespace mp {
namespace loc {
TEST_F(ins_fusion_test, ins_fusion_check) {
  std::cout << " ins_fusion check start " << std::endl;
  InsInitStatus status = ins_fusion_->Init(configfile_);
  bool res = (InsInitStatus::OK == status);
  EXPECT_TRUE(res);
}

}  // namespace loc
}  // namespace mp
}  // namespace hozon
