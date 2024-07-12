/******************************************************************************
 Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved
 *Author: hozon
 *Date: 2024-07-08
 *****************************************************************************/
#include "laneline_postprocess/laneline_postprocess_test.h"

namespace hozon {
namespace mp {
namespace environment {
TEST_F(laneline_postprocess_test, laneline_postprocess_check) {
  std::cout << " laneline_postprocess_test start " << std::endl;
  auto lane_post_process = std::make_shared<LanePostProcess>();
  std::string expected_name = "LanePostProcess";
  std::string get_name = lane_post_process->Name();
  bool res = (expected_name == get_name);
  EXPECT_TRUE(res);
}

}  // namespace environment
}  // namespace mp
}  // namespace hozon
