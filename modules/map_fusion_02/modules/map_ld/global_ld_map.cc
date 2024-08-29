/******************************************************************************
 *   Copyright (C) 2024 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： global_ld_map.cc
 *   date       ： 2024.8
 ******************************************************************************/

#include "modules/map_fusion_02/modules/map_ld/global_ld_map.h"

namespace hozon {
namespace mp {

int GlobalLDMap::Init() {
  std::lock_guard<std::mutex> lock(ld_mtx_);
  ld_map_ = std::make_shared<hdmap::HDMap>();
  return 0;
}

void GlobalLDMap::ResetLDMap(const std::shared_ptr<hdmap::HDMap>& ld_map) {
  std::lock_guard<std::mutex> lock(ld_mtx_);
  ld_map_ = ld_map;
}
void GlobalLDMap::UpdateLDMap(const hozon::hdmap::Map& map) {
  std::lock_guard<std::mutex> lock(ld_mtx_);
  ld_map_->LoadMapFromProto(map);
}

}  // namespace mp
}  // namespace hozon
