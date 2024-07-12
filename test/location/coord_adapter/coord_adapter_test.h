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
#include "location/coord_adapter/lib/coord_adapter.h"

namespace hozon {
namespace mp {
namespace loc {
namespace ca {
class coord_adapter_test : public ::testing::Test {
 public:
  coord_adapter_test() {}
  ~coord_adapter_test() {}

  virtual void SetUp() {
    std::string adflite_root_path =
        hozon::perception::lib::GetEnv("ADFLITE_ROOT_PATH", ".");

    config_ = adflite_root_path + "/" +
                 "runtime_service/mapping/conf/mapping/location/coord_adapter/config.yaml";
    coord_adapter_ = std::make_shared<CoordAdapter>();
  }

  virtual void TearDown() {}

 public:
  std::string config_;
  std::shared_ptr<CoordAdapter> coord_adapter_;
};

extern "C" void Registercoord_adapter_tests() {}

}  // namespace ca
}  // namespace loc
}  // namespace mp
}  // namespace hozon

