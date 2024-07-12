/******************************************************************************
 Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved
 *Author: hozon
 *Date: 2024-07-08
 *****************************************************************************/
#include "local_mapping/local_mapping_test.h"
#include "local_mapping/lib/datalogger/data_buffer.h"

namespace hozon {
namespace mp {
namespace lm {
TEST_F(local_mapping_test, local_mapping_check) {
  std::cout << " local_mapping_test start " << std::endl;
  auto local_map = std::make_shared<LocalMapApp>();
  // local_map->Init();
  // EXPECT_TRUE(res);
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
