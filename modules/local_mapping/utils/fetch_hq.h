/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: shenliangchen
 *Date: 2023-09-14
 *****************************************************************************/
#pragma once

#include <depend/proto/map/map.pb.h>
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <Sophus/se3.hpp>

#include "modules/local_mapping/types/types.h"
#include "modules/util/include/util/temp_log.h"
namespace hozon {
namespace mp {
namespace lm {

class PriorProvider {
 public:
  PriorProvider() = default;
  ~PriorProvider() = default;

  int Init(const std::string& conf);

  std::shared_ptr<hozon::hdmap::Map> GetPrior();

 private:
  std::shared_ptr<hozon::hdmap::Map> prior_ = nullptr;
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
