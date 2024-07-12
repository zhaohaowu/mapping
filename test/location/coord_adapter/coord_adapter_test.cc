/******************************************************************************
 Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved
 *Author: hozon
 *Date: 2024-07-08
 *****************************************************************************/
#include "location/coord_adapter/coord_adapter_test.h"

namespace hozon {
namespace mp {
namespace loc {
namespace ca {
TEST_F(coord_adapter_test, coord_adapter_check) {
  std::cout << " coord_adapter_test start " << std::endl;
  bool res = coord_adapter_->Init(config_);
  EXPECT_TRUE(res);

  // LoadParams
  res = coord_adapter_->LoadParams(config_);
  EXPECT_TRUE(res);
}

}  // namespace ca
}  // namespace loc
}  // namespace mp
}  // namespace hozon
