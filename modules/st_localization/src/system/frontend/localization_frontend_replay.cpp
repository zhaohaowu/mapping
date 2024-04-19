/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */

#include "system/frontend/localization_frontend_replay.hpp"

#include "ad_time/ad_time.hpp"
#include "localization/common/log.hpp"

namespace senseAD {
namespace localization {

adLocStatus_t LocalizationFrontendReplay::InitLocator() { return LOC_SUCCESS; }

adLocStatus_t LocalizationFrontendReplay::InitInitializer() {
  return LOC_SUCCESS;
}

adLocStatus_t LocalizationFrontendReplay::InitDataBuffer() {
  InitCapacity();
  return LOC_SUCCESS;
}

void LocalizationFrontendReplay::SetThreadName() {
  pthread_setname_np(thread_->native_handle(), "LC_front_replay");
}

adLocStatus_t LocalizationFrontendReplay::ResetSubData() {
  ClearData();
  return LOC_SUCCESS;
}

adLocStatus_t LocalizationFrontendReplay::SwitchOriginSubProc() {
  return LOC_SUCCESS;
}

InitStage LocalizationFrontendReplay::Initialize() {
  return InitStage::INIT_DONE;
}

NavStatus LocalizationFrontendReplay::GetCurrentNavStatus(
    const NavState& nav_state) {
  return GetNavStatusEnum(nav_state.nav_status);
}

adLocStatus_t LocalizationFrontendReplay::LocatorProcess(
    NavState* nav_state, bool* continue_to_process) {
  if (nullptr == nav_state || nullptr == continue_to_process) {
    LC_LERROR(SYSTEM) << "nullptr";
    return LOC_NULL_PTR;
  }

  uint64_t frontend_time;
  {
    std::lock_guard<std::mutex> guard(data_mutex_);
    if (!data_list_.empty()) {
      *nav_state = data_list_.front().second;
      frontend_time = data_list_.front().first;
      data_list_.erase(data_list_.begin());
      (*continue_to_process) = !data_list_.empty();
    } else {
      return LOC_LOCALIZATION_ERROR;
    }
  }

  LC_LDEBUG_EVERY_SEC(SYSTEM, 1) << "frontend process replay: " << frontend_time
                                 << "/" << Time::ToString(frontend_time);

  if (LOC_SUCCESS != TimeOrderCheck(frontend_time)) {
    return LOC_LOCALIZATION_ERROR;
  }

  // update front end time
  frontend_time_ = frontend_time;

  return LOC_SUCCESS;
}

}  // namespace localization
}  // namespace senseAD
