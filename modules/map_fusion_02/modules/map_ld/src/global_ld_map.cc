/******************************************************************************
 *   Copyright (C) 2024 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： global_ld_map.cc
 *   date       ： 2024.8
 ******************************************************************************/

#include "modules/map_fusion_02/modules/map_ld/include/global_ld_map.h"

namespace hozon {
namespace mp {

int GlobalLDMap::Init() {
  std::lock_guard<std::mutex> lock(mtx_);
  ld_map_ = std::make_shared<hdmap::HDMap>();
  return 0;
}

void GlobalLDMap::ResetLDMap(const std::shared_ptr<hdmap::HDMap>& ld_map) {
  std::lock_guard<std::mutex> lock(mtx_);
  ld_map_ = ld_map;
}

}  // namespace mp
}  // namespace hozon
