/******************************************************************************
 * Copyright 2023 The Hozon Authors. All Rights Reserved.
 *****************************************************************************/
#include "util/orin_trigger_manager.h"

#include "util/mapping_log.h"

namespace hozon {
namespace mapping {
namespace util {
// NOLINTBEGIN
#ifdef ISORIN
using hozon::netaos::dc::DcResultCode;
#endif
OrinTriggerManager::OrinTriggerManager() : exit_thread_flag_(false) {
#ifdef ISORIN
  dc_client_ = std::make_unique<hozon::netaos::dc::DcClient>();
  auto ret = dc_client_->Init("mapping_trigger", 2000);
  HLOG_ERROR << "DC init = " << ret;
  trigger_id_queue_ = std::make_shared<std::deque<uint32_t>>(20, 9999);
  thread_ = std::make_shared<std::thread>(&OrinTriggerManager::Core, this);
#endif
}

OrinTriggerManager::~OrinTriggerManager() {  // NOLINT
#ifdef ISORIN
  exit_thread_flag_ = true;
  {
    std::unique_lock<std::mutex> lk(mtx_);
    cv_.notify_one();
  }
  if (thread_->joinable()) {
    thread_->join();
  }
  dc_client_->DeInit();
  trigger_id_queue_->clear();
#endif
}

void OrinTriggerManager::Core() {  // NOLINT
#ifdef ISORIN
  pthread_setname_np(pthread_self(), "OrinTriggerManager_core");
  while (true) {
    std::unique_lock<std::mutex> lk(mtx_);
    cv_.wait(lk, [this] { return !trigger_id_queue_->empty(); });

    uint32_t id = 9999;
    id = trigger_id_queue_->front();
    trigger_id_queue_->pop_front();
    if (id != 9999) {
      HLOG_INFO << "send trigger id " << id;
      SendTrigger(id);
    }
    if (exit_thread_flag_) {
      break;
    }
  }
#endif
}

void OrinTriggerManager::SendTrigger(uint32_t trigger_id) {
#ifdef ISORIN
  HLOG_INFO << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<";
  const auto type = dc_client_->CollectTrigger(trigger_id);
  HLOG_ERROR << "DC return type = " << type;
  if (type != DcResultCode::DC_OK) {
    HLOG_ERROR << "event Trigger Fail id " << trigger_id;
  } else {
    HLOG_ERROR << "event Trigger Successful id " << trigger_id;
  }
#endif
}

void OrinTriggerManager::TriggerCollect(uint32_t trigger_id) {
#ifdef ISORIN
  std::lock_guard<std::mutex> lk(mtx_);
  trigger_id_queue_->push_back(trigger_id);
  HLOG_ERROR << "push_back id is: " << trigger_id;
  if (trigger_id_queue_->size() > 20) {
    trigger_id_queue_->pop_front();
    HLOG_WARN << "too much trigger_id!!";
  }
  cv_.notify_one();
#endif
  //   LOC_INFO << "Data collect " << trigger_id << " success";
}
}  // namespace util
}  // namespace mapping
}  // namespace hozon
