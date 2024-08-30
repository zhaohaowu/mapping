/******************************************************************************
 *   Copyright (C) 2024 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： ins_data_manager.h
 *   author     ： zhaohaowu
 *   date       ： 2024.08
 ******************************************************************************/
#pragma once

#include <Eigen/Dense>
#include <memory>
#include <mutex>

#include "perception-base/base/utils/macros.h"
#include "proto/localization/node_info.pb.h"

namespace hozon {
namespace mp {
namespace mf {

class InsDataManager {
 public:
  bool Init();
  ~InsDataManager() = default;
  std::shared_ptr<hozon::localization::HafNodeInfo> GetIns();
  void PushIns(const hozon::localization::HafNodeInfo& latest_ins);

 private:
  std::mutex ins_ptr_mutex_;
  std::shared_ptr<hozon::localization::HafNodeInfo> ins_ptr_ = nullptr;

  // get instance by Instance()
  DECLARE_SINGLETON_PERCEPTION(InsDataManager)
};

#define INS_MANAGER InsDataManager::Instance()

}  // namespace mf
}  // namespace mp
}  // namespace hozon
