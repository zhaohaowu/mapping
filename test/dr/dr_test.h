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
#include "modules/dr/include/dr.h"

namespace hozon {
namespace mp {
namespace dr {
class DrTest : public ::testing::Test {
 public:
  DrTest() {}
  ~DrTest() {}

  virtual void SetUp() {
    std::string default_work_root;
    std::string work_root =
        hozon::perception::lib::GetEnv("ADFLITE_ROOT_PATH", default_work_root);
    if (work_root.empty()) {
      HLOG_ERROR << "ENV: ADFLITE_ROOT_PATH is not set.";
      return;
    }
    std::string config_file = work_root +
                              "/runtime_service/mapping/conf/mapping/"
                              "dr/dr_config.yaml";
    std::cout << " config_file " << config_file << std::endl;
    dr_interface_ = std::make_shared<DRInterface>(config_file);

    odom_datas_ = std::make_shared<Odometry2D>(config_file);
  }

  virtual void TearDown() {}
  std::shared_ptr<DRInterface> dr_interface_;
  std::shared_ptr<Odometry2D> odom_datas_;
};

extern "C" void RegisterDrTests() {}

}  // namespace dr
}  // namespace mp
}  // namespace hozon

