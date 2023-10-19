/******************************************************************************
 *   Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： global_hd_map.h
 *   author     ： mashaoping
 *   date       ： 2023.10
 ******************************************************************************/

#pragma once

#include <depend/map/hdmap/hdmap.h>

#include <memory>

namespace hozon {
namespace mp {

class GlobalHdMap {
 public:
  int Init();
  std::shared_ptr<hdmap::HDMap> GetHdMap();

 private:
  std::shared_ptr<hdmap::HDMap> hd_map_ = nullptr;

  // declare singleton
 private:
  GlobalHdMap() = default;
  ~GlobalHdMap() = default;
  GlobalHdMap(const GlobalHdMap&);
  GlobalHdMap& operator=(const GlobalHdMap&);

 public:
  static GlobalHdMap& Instance() {
    static GlobalHdMap instance;
    return instance;
  }
};

#define GLOBAL_HD_MAP hozon::mp::GlobalHdMap::Instance().GetHdMap()

}  // namespace mp
}  // namespace hozon
