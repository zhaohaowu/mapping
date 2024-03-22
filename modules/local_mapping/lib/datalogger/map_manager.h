/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: chenlongxi
 *Date: 2023-09-06
 *****************************************************************************/
#pragma once

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "modules/local_mapping/base/scene/localmap.h"
#include "modules/local_mapping/lib/datalogger/data_buffer.h"
#include "perception-base/base/utils/macros.h"

namespace hozon {
namespace mp {
namespace lm {

class MapManager {
 public:
  bool Init();
  LocalMapFramePtr GetLocalMap();

  bool SetLocalMap(const LocalMapFramePtr& input_localmap);
  void GetOutLocalMap(LocalMapFramePtr* out_localmap_ptr);
  void GetDeepLocalMap(LocalMapFramePtr& deep_localmap_ptr);  // NOLINT
  LocalMapFramePtr GetWriteLocalMap();
  void SwapLocalMap();
  ~MapManager() = default;

 private:
  // 类初始化相关
  std::mutex mutex_;
  bool inited_ = false;

  std::vector<LocalMapFramePtr> buffer_map_;
  std::atomic<int> read_idx_ = 0;
  std::atomic_flag flag_ = {false};

  // get instance by Instance()
  DECLARE_SINGLETON_PERCEPTION(MapManager)
};

#define MAP_MANAGER MapManager::Instance()

#define MANAGER_RVIZ_GET_MAP(out_localmap_ptr) \
  MapManager::Instance()->GetOutLocalMap(&(out_localmap_ptr))

// 外部多线程获取map必须使用该宏
#define MANAGER_GET_OUT_MAP(out_localmap_ptr) \
  MapManager::Instance()->GetDeepLocalMap(out_localmap_ptr)

}  // namespace lm
}  // namespace mp
}  // namespace hozon
