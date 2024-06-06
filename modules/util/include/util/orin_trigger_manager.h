/******************************************************************************
 * Copyright 2024 The Hozon Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once
#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <deque>
#include <condition_variable>


#ifdef ISORIN
#include "dc/include/dc_client.h"
using namespace hozon::netaos::cm;// NOLINT
#endif
namespace hozon {
namespace mapping {
namespace util {
#ifdef ISORIN
using hozon::netaos::dc::DcClient;
#endif
/**
 * @brief 数据收集上云
 *
 */
class OrinTriggerManager {  // NOLINT
 public:
  OrinTriggerManager();
  ~OrinTriggerManager();

  void Core();
  void SendTrigger(uint32_t trigger_id);
  /**
   * @brief 根据事件信息触发数据收集上云接口
   *
   * @param trigger_id 触发的事件的id
   */
  void TriggerCollect(uint32_t trigger_id);

 private:
#ifdef ISORIN
  std::unique_ptr<hozon::netaos::dc::DcClient> dc_client_ = nullptr;
#endif
  std::shared_ptr<std::deque<uint32_t>> trigger_id_queue_;
  std::shared_ptr<std::thread> thread_;
  std::atomic<bool> exit_thread_flag_;
  std::mutex mtx_;
  std::condition_variable cv_;

 public:
  static OrinTriggerManager& Instance() {
    static OrinTriggerManager instance;
    return instance;
  }
};
#define GLOBAL_DC_TRIGGER hozon::mapping::util::OrinTriggerManager::Instance()
}  // namespace util
}  // namespace mapping
}  // namespace hozon
