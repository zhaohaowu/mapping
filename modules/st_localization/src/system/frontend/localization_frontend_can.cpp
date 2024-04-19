/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */

#include "system/frontend/localization_frontend_can.hpp"

#include <memory>

#include "ad_time/ad_time.hpp"
#include "localization/common/log.hpp"
#include "msf/msf_fusion/msf_fusion.hpp"

namespace senseAD {
namespace localization {

adLocStatus_t LocalizationFrontendCAN::InitLocator() {
  locator_ = std::make_shared<CANLocator>();
  if (nullptr == locator_) {
    LC_LERROR(SYSTEM) << "Failed to create CAN locator.";
    return LOC_ALLOC_MEMORY_FAILED;
  }
  if (locator_->Init(param_) != LOC_SUCCESS) {
    LC_LERROR(SYSTEM) << "Failed to init CAN locator.";
    return LOC_LOCALIZATION_ERROR;
  }
  return LOC_SUCCESS;
}

adLocStatus_t LocalizationFrontendCAN::InitInitializer() {
  initializer_ = std::make_shared<Initialization>();
  if (nullptr == initializer_) {
    LC_LERROR(SYSTEM) << "Failed to create CAN Initializer";
    return LOC_LOCALIZATION_ERROR;
  }
  if (initializer_->Init(param_) != LOC_SUCCESS) {
    LC_LERROR(SYSTEM) << "Failed to init CAN Initializer.";
    return LOC_LOCALIZATION_ERROR;
  }
  initializer_->SetFrontendLocator(locator_);
  return LOC_SUCCESS;
}

adLocStatus_t LocalizationFrontendCAN::InitDataBuffer() {
  InitCapacity();
  return LOC_SUCCESS;
}

void LocalizationFrontendCAN::SetThreadName() {
  pthread_setname_np(thread_->native_handle(), "LC_front_can");
}

adLocStatus_t LocalizationFrontendCAN::ResetSubData() {
  ClearData();
  return LOC_SUCCESS;
}

adLocStatus_t LocalizationFrontendCAN::SwitchOriginSubProc() {
  return LOC_SUCCESS;
}

InitStage LocalizationFrontendCAN::Initialize() {
  if (initializer_->GetInitStage() != InitStage::INIT_DONE) {
    initializer_->Initialize();
  }
  return initializer_->GetInitStage();
}

NavStatus LocalizationFrontendCAN::GetCurrentNavStatus(
    const NavState& nav_state) {
  // TODO(zm): implement nav status calculation as CAN is frontend
  return NavStatus::MID_ACCURACY;
}

adLocStatus_t LocalizationFrontendCAN::LocatorProcess(
    NavState* nav_state, bool* continue_to_process) {
  if (nullptr == nav_state || nullptr == continue_to_process) {
    LC_LERROR(SYSTEM) << "nullptr";
    return LOC_NULL_PTR;
  }
  uint64_t frontend_time;
  static auto can_data = std::make_shared<VehicleInfo>();
  {
    std::lock_guard<std::mutex> guard(data_mutex_);
    if (!data_list_.empty()) {
      *can_data = data_list_.front().second;
      frontend_time = data_list_.front().first;
      data_list_.erase(data_list_.begin());
      (*continue_to_process) = !data_list_.empty();
    } else {
      (*continue_to_process) = false;
      return LOC_LOCALIZATION_ERROR;
    }
  }

  LC_LDEBUG_EVERY_SEC(SYSTEM, 1) << "frontend process can: " << frontend_time
                                 << "/" << Time::ToString(frontend_time);

  if (LOC_SUCCESS != TimeOrderCheck(frontend_time)) {
    return LOC_LOCALIZATION_ERROR;
  }

  // msf state update
  if (msf_fusion_->IsStarted()) msf_fusion_->StateUpdate();

  // TODO(zm): as can_integrate is visited in msf_fusion_, should consider
  // add mutex if using in other threads
  if (LOC_SUCCESS != locator_->Process(frontend_time, can_data)) {
    LC_LDEBUG(SYSTEM) << "front can locator process failed";
    return LOC_LOCALIZATION_ERROR;
  }

  // get nav state and caculate localization status
  locator_->GetState(nav_state);
  nav_state->nav_status = GetNavStatusNum(GetCurrentNavStatus(*nav_state));

  // add to MSF can data buffer and update MSF current state
  if (msf_fusion_->IsStarted()) {
    CANMeasurement can_reading;
    // using compensated can measurements, as not made in can integrator
    can_reading << nav_state->linear_speed, nav_state->angular_speed;
    msf_fusion_->AddData(nav_state->timestamp, can_reading);
  }

  // update front end time
  frontend_time_ = frontend_time;

  return LOC_SUCCESS;
}

}  // namespace localization
}  // namespace senseAD
