/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */

#include "system/frontend/localization_frontend_ins.hpp"

#include <memory>

#include "ad_time/ad_time.hpp"
#include "common/msf_common.hpp"
#include "localization/common/log.hpp"

namespace senseAD {
namespace localization {

void LocalizationFrontendINS::SetImuData(uint64_t timestamp, const Imu& imu) {
  std::lock_guard<std::mutex> lock(imu_mutex_);
  imu_data_list_.push_back(std::make_pair(timestamp, imu));
}

adLocStatus_t LocalizationFrontendINS::InitLocator() {
  locator_ = std::make_shared<INSLocator>();
  if (nullptr == locator_) {
    LC_LERROR(SYSTEM) << "Failed to create INS locator.";
    return LOC_ALLOC_MEMORY_FAILED;
  }
  if (locator_->Init(param_) != LOC_SUCCESS) {
    LC_LERROR(SYSTEM) << "Failed to init INS locator.";
    return LOC_LOCALIZATION_ERROR;
  }
  return LOC_SUCCESS;
}

adLocStatus_t LocalizationFrontendINS::InitInitializer() {
  initializer_ = std::make_shared<Initialization>();
  if (nullptr == initializer_) {
    LC_LERROR(SYSTEM) << "Failed to create INS Initializer";
    return LOC_LOCALIZATION_ERROR;
  }
  if (initializer_->Init(param_) != LOC_SUCCESS) {
    LC_LERROR(SYSTEM) << "Failed to init INS Initializer.";
    return LOC_LOCALIZATION_ERROR;
  }
  initializer_->SetFrontendLocator(locator_);

  return LOC_SUCCESS;
}

adLocStatus_t LocalizationFrontendINS::InitDataBuffer() {
  InitCapacity();
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_data_list_.set_capacity(kDataSizeUpperLimit);
  }
  return LOC_SUCCESS;
}

void LocalizationFrontendINS::SetThreadName() {
  pthread_setname_np(thread_->native_handle(), "LC_front_ins");
}

adLocStatus_t LocalizationFrontendINS::ResetSubData() {
  ClearData();
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_data_list_.clear();
  }
  return LOC_SUCCESS;
}

adLocStatus_t LocalizationFrontendINS::SwitchOriginSubProc() {
  return LOC_SUCCESS;
}

InitStage LocalizationFrontendINS::Initialize() {
  if (initializer_->GetInitStage() != InitStage::INIT_DONE) {
    initializer_->Initialize();
  }
  return initializer_->GetInitStage();
}

NavStatus LocalizationFrontendINS::GetCurrentNavStatus(
    const NavState& nav_state) {
  return NavStatus::MID_ACCURACY;  // INS mode treat as middle accuracy
}

adLocStatus_t LocalizationFrontendINS::LocatorProcess(
    NavState* nav_state, bool* continue_to_process) {
  if (nullptr == nav_state || nullptr == continue_to_process) {
    LC_LERROR(SYSTEM) << "nullptr";
    return LOC_NULL_PTR;
  }

  uint64_t frontend_time;
  static auto ins_data = std::make_shared<Ins>();
  {
    std::lock_guard<std::mutex> guard(data_mutex_);
    if (!data_list_.empty()) {
      *ins_data = data_list_.front().second;
      frontend_time = data_list_.front().first;
      data_list_.erase(data_list_.begin());
      (*continue_to_process) = !data_list_.empty();
    } else {
      return LOC_LOCALIZATION_ERROR;
    }
  }

  static auto imu_data = std::make_shared<Imu>();
  {
    std::lock_guard<std::mutex> guard(imu_mutex_);
    // to ensure always get imu data, here is somewhat brute force
    if (!imu_data_list_.empty() &&
        std::fabs(imu_data_list_.back().first * kNanoSecToSec -
                  frontend_time * kNanoSecToSec) < 0.05) {
      *imu_data = imu_data_list_.back().second;
    } else {
      return LOC_LOCALIZATION_ERROR;
    }
  }

  LC_LDEBUG_EVERY_SEC(SYSTEM, 1) << "frontend process ins: " << frontend_time
                                 << "/" << Time::ToString(frontend_time);

  if (LOC_SUCCESS != TimeOrderCheck(frontend_time)) {
    return LOC_LOCALIZATION_ERROR;
  }

  // locator process
  if (LOC_SUCCESS != locator_->Process(frontend_time, ins_data, imu_data)) {
    LC_LDEBUG(SYSTEM) << "front ins locator process failed";
    return LOC_LOCALIZATION_ERROR;
  }

  // get nav state and caculate localization status
  locator_->GetState(nav_state);
  nav_state->nav_status = GetNavStatusNum(GetCurrentNavStatus(*nav_state));

  // update front end time
  frontend_time_ = frontend_time;

  return LOC_SUCCESS;
}

}  // namespace localization
}  // namespace senseAD
