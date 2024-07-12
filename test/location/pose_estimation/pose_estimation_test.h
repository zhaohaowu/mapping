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
#include "perception-lib/lib/environment/environment.h"

#define private public
#define protected public
#include "location/pose_estimation/lib/pose_estimate/pose_estimate.h"

namespace hozon {
namespace mp {
namespace loc {
class pose_estimation_test : public ::testing::Test {
 public:
  pose_estimation_test() {}
  ~pose_estimation_test() {}

  virtual void SetUp() {
    map_match_ = std::make_shared<MapMatch>();
  }

  virtual void TearDown() {}

 public:
  std::shared_ptr<MapMatch> map_match_;
};

extern "C" void Registerpose_estimation_tests() {}

}  // namespace loc
}  // namespace mp
}  // namespace hozon

