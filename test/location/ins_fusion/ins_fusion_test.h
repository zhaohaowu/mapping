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
#include "location/ins_fusion/lib/ins_fusion.h"

namespace hozon {
namespace mp {
namespace loc {
class ins_fusion_test : public ::testing::Test {
 public:
  ins_fusion_test() {}
  ~ins_fusion_test() {}

  virtual void SetUp() {
    std::string adflite_root_path =
        hozon::perception::lib::GetEnv("ADFLITE_ROOT_PATH", ".");

    configfile_ = adflite_root_path + "/" +
                  "runtime_service/mapping/conf/mapping/location/ins_fusion/"
                  "ins_config.yaml";
    ins_fusion_ = std::make_shared<InsFusion>();
  }

  virtual void TearDown() {}

 public:
  std::shared_ptr<InsFusion> ins_fusion_;
  std::string configfile_;
};

extern "C" void Registerins_fusion_tests() {}

}  // namespace loc
}  // namespace mp
}  // namespace hozon
