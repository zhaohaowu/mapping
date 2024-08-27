/******************************************************************************
 *   Copyright (C) 2024 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： global_ld_map.h
 *   date       ： 2024.8
 ******************************************************************************/

#pragma once
#include <depend/map/hdmap/hdmap.h>

#include <atomic>
#include <memory>
#include <mutex>
#include <shared_mutex>

namespace hozon {
namespace mp {

class GlobalLDMap {
 public:
  std::shared_ptr<hdmap::HDMap> GetLDMap() {
    std::lock_guard<std::mutex> lock(mtx_);
    return ld_map_;
  }

  void ResetLDMap(const std::shared_ptr<hdmap::HDMap>& ld_map);

  static GlobalLDMap* Instance() {
    static GlobalLDMap instance;
    return &instance;
  }

 private:
  int Init();
  std::shared_ptr<hdmap::HDMap> ld_map_ = nullptr;
  std::mutex mtx_;
  GlobalLDMap() { Init(); }
  ~GlobalLDMap() = default;
  GlobalLDMap(const GlobalLDMap&) { Init(); }
  GlobalLDMap& operator=(const GlobalLDMap&);
};

#define GLOBAL_LD_MAP hozon::mp::GlobalLDMap::Instance()->GetLDMap()

}  // namespace mp
}  // namespace hozon
