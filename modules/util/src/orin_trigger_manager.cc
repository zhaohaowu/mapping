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
OrinTriggerManager::OrinTriggerManager() {
#ifdef ISORIN
  dc_client_ = std::make_unique<hozon::netaos::dc::DcClient>();
  auto ret = dc_client_->Init("mapping_trigger", 2000);
  HLOG_ERROR << "DC init = " << ret;
#endif
}

OrinTriggerManager::~OrinTriggerManager() {  // NOLINT
#ifdef ISORIN
  dc_client_->DeInit();
#endif
}

void OrinTriggerManager::TriggerCollect(uint32_t trigger_id) {
#ifdef ISORIN
  HLOG_ERROR << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<";
  const auto type = dc_client_->CollectTrigger(trigger_id);
  HLOG_ERROR << "DC return type = " << type;
  if (type != DcResultCode::DC_OK) {
    HLOG_ERROR << "event Trigger Fail id " << trigger_id;
  } else {
    HLOG_ERROR << "event Trigger Successful id " << trigger_id;
  }
#endif
  //   LOC_INFO << "Data collect " << trigger_id << " success";
}
}  // namespace util
}  // namespace mapping
}  // namespace hozon
