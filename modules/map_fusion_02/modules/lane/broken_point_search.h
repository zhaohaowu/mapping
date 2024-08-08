/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： broken_point_search.h
 *   author     ： hozon
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include "modules/map_fusion_02/base/element_map.h"

namespace hozon {
namespace mp {
namespace mf {
class BrokenPointSearch {
 public:
  BrokenPointSearch() : point_num_(-1) {}
  ~BrokenPointSearch() = default;
  BrokenPointSearch(const BrokenPointSearch&) = delete;
  BrokenPointSearch& operator=(const BrokenPointSearch&) = delete;
  bool Init();
  bool Process(ElementMap::Ptr element_map_ptr);
  void Search();

 private:
  int point_num_;
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
