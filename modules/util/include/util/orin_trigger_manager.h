/******************************************************************************
 * Copyright 2024 The Hozon Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once
#include <memory>
#include <string>
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
