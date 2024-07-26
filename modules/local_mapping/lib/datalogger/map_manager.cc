/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: chenlongxi
 *Date: 2023-09-06
 *****************************************************************************/
#include "modules/local_mapping/lib/datalogger/map_manager.h"

#include <memory>

#include "depend/common/util/perf_util.h"
#include "modules/local_mapping/utils/lane_utils.h"
#include "modules/util/include/util/mapping_log.h"

namespace hozon {
namespace mp {
namespace lm {

MapManager::MapManager() { CHECK_EQ(this->Init(), true); }

bool MapManager::Init() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (inited_) {
    return true;
  }
  buffer_map_.resize(2);
  // map数据初始化
  for (int i = 0; i <= 1; ++i) {
    LocalMapFramePtr map_ = std::make_shared<LocalMapFrame>();
    map_->lane_lines_ptr = std::make_shared<LaneLines>();
    map_->road_edges_ptr = std::make_shared<RoadEdges>();
    map_->road_arrows_ptr = std::make_shared<Arrows>();
    map_->stop_lines_ptr = std::make_shared<StopLines>();
    map_->zebra_crossings_ptr = std::make_shared<ZebraCrossings>();
    map_->occ_edges_ptr = std::make_shared<OccEdges>();
    buffer_map_[i] = map_;
  }

  inited_ = true;
  return true;
}

// 内部线程不需要做原子变量判断
LocalMapFramePtr MapManager::GetLocalMap() { return buffer_map_[read_idx_]; }
// 内部rviz使用接口
void MapManager::GetOutLocalMap(LocalMapFramePtr* out_localmap_ptr) {
  while (flag_.test_and_set(std::memory_order_acquire)) {
    cpu_relax();
  }
  *out_localmap_ptr = buffer_map_[read_idx_];
  flag_.clear(std::memory_order_release);
}
// 外部共享内存使用deep copy出来使用接口
void MapManager::GetDeepLocalMap(LocalMapFramePtr& deep_localmap_ptr) {
  while (flag_.test_and_set(std::memory_order_acquire)) {
    cpu_relax();
  }
  auto read_localmap_ptr = buffer_map_[read_idx_];
  flag_.clear(std::memory_order_release);
  DeepCopy(read_localmap_ptr, deep_localmap_ptr);
}

// 只有一个线程执行写操作
LocalMapFramePtr MapManager::GetWriteLocalMap() {
  while (flag_.test_and_set(std::memory_order_acquire)) {
    cpu_relax();
  }
  int write_idx = 1 - read_idx_;
  while (buffer_map_[write_idx].use_count() != 1) {
    std::this_thread::sleep_for(std::chrono::microseconds(10));
  }
  flag_.clear(std::memory_order_release);
  return buffer_map_[write_idx];
}

void MapManager::SwapLocalMap() { read_idx_ = 1 - read_idx_; }

}  // namespace lm
}  // namespace mp
}  // namespace hozon
