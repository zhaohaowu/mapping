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
#include "yaml-cpp/yaml.h"

#define private public
#define protected public
#include "map_fusion/include/map_fusion/map_fusion.h"
#include "map_fusion/include/map_fusion/map_prediction/map_prediction.h"

namespace hozon {
namespace mp {
namespace mf {
class map_fusion_test : public ::testing::Test {
 public:
  map_fusion_test() {}
  ~map_fusion_test() {}

  virtual void SetUp() {
    std::string default_work_root;
    std::string work_root =
        hozon::perception::lib::GetEnv("ADFLITE_ROOT_PATH", default_work_root);
    if (work_root.empty()) {
      // HLOG_ERROR << "ENV: ADFLITE_ROOT_PATH is not set.";
      return;
    }

    config_file_ = work_root +
                   "/runtime_service/mapping/conf/mapping/"
                   "map_fusion/map_fusion.yaml";
    config_ = YAML::LoadFile(config_file_);
    map_fusion_ = std::make_shared<MapFusion>();
    map_prediction_ = std::make_shared<MapPrediction>();
  }
  virtual void TearDown() {}

 private:
  std::shared_ptr<MapFusion> map_fusion_;
  std::shared_ptr<MapPrediction> map_prediction_;
  std::string config_file_;
  YAML::Node config_;
};

extern "C" void Registermap_fusion_tests() {}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
