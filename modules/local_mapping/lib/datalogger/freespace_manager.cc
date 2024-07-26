/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: shenliangchen
 *Date: 2024-04-29
 *****************************************************************************/
#include "modules/local_mapping/lib/datalogger/freespace_manager.h"

#include "perception-lib/lib/config_manager/config_manager.h"
namespace hozon {
namespace mp {
namespace lm {

namespace perception_lib = hozon::perception::lib;
FreeSpaceManager::FreeSpaceManager() { CHECK_EQ(this->Init(), true); }

MessageBuffer<FreeSpacesConstPtr>& FreeSpaceManager::GetFreeSpaceBuffer() {
  return origin_freespace_buffer_;
}

bool FreeSpaceManager::Init() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (inited_) {
    return true;
  }
  origin_freespace_buffer_.set_capacity(2);
  origin_freespace_buffer_.clear();
  inited_ = true;
  return true;
}

bool FreeSpaceManager::PushFreeSpaceData(
    const FreeSpacesConstPtr& latest_freespace) {
  HLOG_DEBUG << "latest_freespace timestamp: " << SET_PRECISION(20)
             << latest_freespace->timestamp;
  if (origin_freespace_buffer_.is_empty() ||
      origin_freespace_buffer_.back()->timestamp <
          latest_freespace->timestamp) {
    origin_freespace_buffer_.push_new_message(latest_freespace->timestamp,
                                              latest_freespace);
    return true;
  }
  return false;
}
}  // namespace lm
}  // namespace mp
}  // namespace hozon
