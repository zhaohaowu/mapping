/******************************************************************************
 *   Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： global_hd_map.h
 *   author     ： mashaoping
 *   date       ： 2023.10
 ******************************************************************************/

#pragma once
#include <depend/map/hdmap/hdmap.h>

#include <atomic>
#include <memory>
#include <mutex>
#include <shared_mutex>

namespace hozon {
namespace mp {

class GlobalHdMap {
 public:
  std::shared_ptr<hdmap::HDMap> GetHdMap() {
    std::lock_guard<std::mutex> lock(mtx_);
    return hd_map_;
  }

  void ResetHdMap(const std::shared_ptr<hdmap::HDMap>& hd_map);

  static GlobalHdMap* Instance() {
    static GlobalHdMap instance;
    return &instance;
  }

 private:
  int Init();
  std::shared_ptr<hdmap::HDMap> hd_map_ = nullptr;
  std::mutex mtx_;
  GlobalHdMap() { Init(); }
  ~GlobalHdMap() = default;
  GlobalHdMap(const GlobalHdMap&) { Init(); }
  GlobalHdMap& operator=(const GlobalHdMap&);
};

#define GLOBAL_HD_MAP hozon::mp::GlobalHdMap::Instance()->GetHdMap()

}  // namespace mp
}  // namespace hozon
