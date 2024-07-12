/******************************************************************************
 Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved
 *Author: hozon
 *Date: 2024-07-08
 *****************************************************************************/
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "gtest/gtest.h"

#define private public
#define protected public
#include "onboard/onboard_lite/phm_comment_lite/phm_commenet.h"

namespace hozon {
namespace perception {
namespace common_onboard {
class PhmCommentTest : public ::testing::Test {
 public:
  PhmCommentTest() {}
  ~PhmCommentTest() {}

  virtual void SetUp() {
    phm_comment_ = std::make_shared<PhmComponent>();
  }

  virtual void TearDown() {}

 public:
  std::shared_ptr<PhmComponent> phm_comment_;
};

extern "C" void RegisterPhmCommentTests() {}

}  //  namespace common_onboard
}  //  namespace perception
}  //  namespace hozon
