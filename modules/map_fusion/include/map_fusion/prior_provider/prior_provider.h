/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： prior_provider.h
 *   author     ： taoshaoyuan/zuodongsheng
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <depend/proto/map/map.pb.h>

#include <memory>
#include <string>

namespace hozon {
namespace mp {
namespace mf {

class PriorProvider {
 public:
  PriorProvider() = default;
  ~PriorProvider() = default;

  int Init(const std::string& conf);

  std::shared_ptr<hozon::hdmap::Map> GetPrior();

 private:
  std::shared_ptr<hozon::hdmap::Map> prior_ = nullptr;
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
