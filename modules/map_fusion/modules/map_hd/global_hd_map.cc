/******************************************************************************
 *   Copyright (C) 2024 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： global_hd_map.cc
 *   date       ： 2024.08
 ******************************************************************************/

#include "modules/map_fusion/modules/map_hd/global_hd_map.h"

namespace hozon {
namespace mp {

int GlobalHdMap::Init() {
  std::lock_guard<std::mutex> lock(hd_mtx_);
  hd_map_ = std::make_shared<hdmap::HDMap>();
  return 0;
}

void GlobalHdMap::ResetHdMap(const std::shared_ptr<hdmap::HDMap>& hd_map) {
  std::lock_guard<std::mutex> lock(hd_mtx_);
  hd_map_ = hd_map;
}

void GlobalHdMap::UpdateHdMap(
    const std::list<hozon::hdmap::Map>& extended_map_protos,
    const std::list<hozon::hdmap::Map>& shrinked_map_protos) {
  std::lock_guard<std::mutex> lock(hd_mtx_);
  hd_map_->UpdateMapFromProto(extended_map_protos, shrinked_map_protos);
}

}  // namespace mp
}  // namespace hozon
