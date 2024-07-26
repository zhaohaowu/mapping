/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: shenliangchen
 *Date: 2024-04-29
 *****************************************************************************/
#pragma once

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "modules/local_mapping/base/scene/freespace.h"
#include "modules/local_mapping/lib/datalogger/data_buffer.h"
#include "perception-base/base/utils/macros.h"

namespace hozon {
namespace mp {
namespace lm {

class FreeSpaceManager {
 public:
  bool Init();
  MessageBuffer<FreeSpacesConstPtr>& GetFreeSpaceBuffer();
  FreeSpacesConstPtr GetFreeSpaceByTimeStamp(double timestamp);
  Eigen::Affine3d GetDeltaPose();
  Eigen::Affine3d GetCurrentPose();
  bool IsStaticState();
  bool IsTurnState();
  bool GetTurnState() const;
  bool PushFreeSpaceData(const FreeSpacesConstPtr& latest_freespace);
  ~FreeSpaceManager() = default;

 private:
  /** @brief sensor data buffer. */
  MessageBuffer<FreeSpacesConstPtr> origin_freespace_buffer_;

  // 类初始化相关
  std::mutex mutex_;
  bool inited_ = false;

  // get instance by Instance()
  DECLARE_SINGLETON_PERCEPTION(FreeSpaceManager)
};

#define OCC_MANAGER FreeSpaceManager::Instance()

}  // namespace lm
}  // namespace mp
}  // namespace hozon
