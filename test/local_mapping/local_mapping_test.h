/******************************************************************************
 Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved
 *Author: hozon
 *Date: 2024-07-08
 *****************************************************************************/
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include "perception-lib/lib/environment/environment.h"
#include "gtest/gtest.h"

#define private public
#define protected public
#include "local_mapping/app/local_mapping.h"

namespace hozon {
namespace mp {
namespace lm {
class local_mapping_test : public ::testing::Test {
 public:
  local_mapping_test() {}
  ~local_mapping_test() {}

  virtual void SetUp() {
    std::string default_work_root;
    std::string work_root =
        hozon::perception::lib::GetEnv("ADFLITE_ROOT_PATH", default_work_root);
    if (work_root.empty()) {
      HLOG_ERROR << "ENV: ADFLITE_ROOT_PATH is not set.";
      return;
    }
  }

  virtual void TearDown() {}
};

extern "C" void Registerlocal_mapping_tests() {}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
