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
#include "laneline_postprocess/app/laneline_postprocess.h"

namespace hozon {
namespace mp {
namespace environment {
class laneline_postprocess_test : public ::testing::Test {
 public:
  laneline_postprocess_test() {}
  ~laneline_postprocess_test() {}

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

extern "C" void Registerlaneline_postprocess_tests() {}

}  // namespace environment
}  // namespace mp
}  // namespace hozon
