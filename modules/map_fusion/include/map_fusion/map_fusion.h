/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： map_fusion.h
 *   author     ： taoshaoyuan
 *   date       ： 2023.08
 ******************************************************************************/

#pragma once

#include <string>

namespace hozon {
namespace mp {
namespace mf {

class MapFusion {
 public:
  MapFusion() = default;
  ~MapFusion() = default;
  int Init(const std::string& conf);
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon