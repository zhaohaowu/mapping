/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： junction_status_manager.h
 *   author     ： chenlongxi
 *   date       ： 2023.09
 ******************************************************************************/
#pragma once
#include <memory>

#include "modules/map_fusion_02/base/junction.h"
#include "perception-base/base/utils/macros.h"

namespace hozon {
namespace mp {
namespace mf {

class JuncStatusManager {
 public:
  ~JuncStatusManager() = default;
  bool Init();
  void UpdateStatus(const JunctionInfo& junc_info);
  JunctionInfo GetJuncStatus();

 private:
  // 类初始化相关
  bool inited_ = false;
  std::mutex mutex_;
  JunctionInfo junc_status;

  // get instance by Instance()
  DECLARE_SINGLETON_PERCEPTION(JuncStatusManager)
};

#define JUNC_MANAGER JuncStatusManager::Instance()

}  // namespace mf
}  // namespace mp
}  // namespace hozon
