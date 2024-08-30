/******************************************************************************
 *   Copyright (C) 2024 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： ins_data_manager.cc
 *   author     ： zhaohaowu
 *   date       ： 2024.08
 ******************************************************************************/

#include "modules/map_fusion_02/data_manager/ins_data_manager.h"

#include <memory>
#include <mutex>

#include "proto/localization/node_info.pb.h"

namespace hozon {
namespace mp {
namespace mf {

InsDataManager::InsDataManager() { this->Init(); }

bool InsDataManager::Init() {
  std::lock_guard<std::mutex> lg_mutex(ins_ptr_mutex_);
  ins_ptr_ = std::make_shared<hozon::localization::HafNodeInfo>();
  return true;
}

std::shared_ptr<hozon::localization::HafNodeInfo> InsDataManager::GetIns() {
  std::lock_guard<std::mutex> lg_mutex(ins_ptr_mutex_);
  auto ins_ptr = std::make_shared<hozon::localization::HafNodeInfo>(*ins_ptr_);
  return ins_ptr;
}

void InsDataManager::PushIns(const hozon::localization::HafNodeInfo& ins) {
  std::lock_guard<std::mutex> lg_mutex(ins_ptr_mutex_);
  *ins_ptr_ = ins;
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
