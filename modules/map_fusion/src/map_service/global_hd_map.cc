/******************************************************************************
 *   Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： global_hd_map.cc
 *   author     ： mashaoping
 *   date       ： 2023.10
 ******************************************************************************/

#include "map_fusion/map_service/global_hd_map.h"

namespace hozon {
namespace mp {

int GlobalHdMap::Init() {
  hd_map_ = std::make_shared<hdmap::HDMap>();
  return 0;
}

void GlobalHdMap::ResetHdMap(const std::shared_ptr<hdmap::HDMap>& hd_map) {
  hd_map_ = hd_map;
}

}  // namespace mp
}  // namespace hozon
