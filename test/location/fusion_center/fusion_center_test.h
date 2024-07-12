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
#include "location/fusion_center/lib/fusion_center.h"

namespace hozon {
namespace mp {
namespace loc {
namespace fc {
class fusion_center_test : public ::testing::Test {
 public:
  fusion_center_test() {}
  ~fusion_center_test() {}

  virtual void SetUp() {
    std::string adflite_root_path =
        hozon::perception::lib::GetEnv("ADFLITE_ROOT_PATH", ".");

    fc_config_ = adflite_root_path + "/" +
                 "runtime_service/mapping/conf/mapping/"
                 "location/fusion_center/fc_config.yaml";
    fc_kf_config_ = adflite_root_path + "/" +
                    "runtime_service/mapping/conf/mapping/"
                    "location/fusion_center/kalman.yaml";
    fc_eskf_config_ =
        adflite_root_path + "/" +
        "runtime_service/mapping/conf/mapping/location/fusion_center/eskf.yaml";
    fc_monitor_config_ =
        adflite_root_path + "/" +
        "runtime_service/mapping/conf/mapping/location/fusion_center/"
        "monitor.yaml";
    fc_center_ = std::make_shared<FusionCenter>();
  }

  virtual void TearDown() {}

 public:
  std::shared_ptr<FusionCenter> fc_center_;
  std::string fc_config_;
  std::string fc_kf_config_;
  std::string fc_eskf_config_;
  std::string fc_monitor_config_;
};

extern "C" void Registerfusion_center_tests() {}

}  // namespace fc
}  // namespace loc
}  // namespace mp
}  // namespace hozon
