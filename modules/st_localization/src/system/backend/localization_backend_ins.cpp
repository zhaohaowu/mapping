/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */

#include "system/backend/localization_backend_ins.hpp"

#include <memory>

#include "common/utility.hpp"
#include "localization/common/log.hpp"
#include "msf/msf_factor/msf_factor.hpp"
#include "msf/msf_fusion/msf_fusion.hpp"
#include "system/localization_frontend.hpp"

namespace senseAD {
namespace localization {

adLocStatus_t LocalizationBackendINS::InitLocator() {
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

adLocStatus_t LocalizationBackendINS::InitLocEvaluator() { return LOC_SUCCESS; }

adLocStatus_t LocalizationBackendINS::InitDataBuffer() {
  InitCapacity();
  return LOC_SUCCESS;
}

void LocalizationBackendINS::SetThreadName() {
  pthread_setname_np(thread_->native_handle(), "LC_back_ins");
}

adLocStatus_t LocalizationBackendINS::ResetSubData() {
  ClearData();
  last_fins_timestamp_ = 0;
  return LOC_SUCCESS;
}

adLocStatus_t LocalizationBackendINS::SwitchOriginSubProc() {
  return LOC_SUCCESS;
}

adLocStatus_t LocalizationBackendINS::LocatorProcess() {
  // get ins data from cached buffer
  uint64_t backend_time_stamp;
  static auto ins_data = std::make_shared<Ins>();
  {
    std::lock_guard<std::mutex> guard(data_mutex_);
    if (data_list_.empty()) return LOC_LOCALIZATION_ERROR;
    *ins_data = data_list_.front().second;
    backend_time_stamp = data_list_.front().first;
    // wanna data delay process
    auto loc_frontend = loc_frontend_.lock();
    if (backend_time_stamp >= loc_frontend->GetFrontendTime())
      return LOC_LOCALIZATION_ERROR;
    data_list_.erase(data_list_.begin());
  }

  // as fused ins is about 100hz, so time order check is important
  if (backend_time_stamp <= last_fins_timestamp_) {
    LC_LWARN(SYSTEM) << "backend_time_stamp is less than last_fins_time.";
    return LOC_LOCALIZATION_ERROR;
  }
  last_fins_timestamp_ = backend_time_stamp;

  // start fused ins process
  adLocStatus_t status =
      locator_->Process(backend_time_stamp, ins_data, nullptr);
  if (status != LOC_SUCCESS) {
    LC_LWARN(SYSTEM) << "Fins localization error.";
    return LOC_LOCALIZATION_ERROR;
  }

  // get processed state, and convert to observation factor
  NavState nav_state;
  locator_->GetState(&nav_state);

  Eigen::Matrix<double, 7, 1> obs;
  obs.segment<3>(0) = nav_state.pose.translation();
  obs.segment<4>(3) =
      Utility::EigenQtoHamiltonVec(nav_state.pose.unit_quaternion());

  auto front_locator_type =
      LocatorTypeFromString(param_.common_param.front_locator_type);
  std::shared_ptr<msf::MSFFactor> msf_factor_fins =
      msf::MSFFactorFactory::CreateMSFFactor(front_locator_type, FINS);
  if (msf_factor_fins == nullptr) {
    LC_LERROR(SYSTEM) << "unsupported fusion mode !!!";
    return LOC_LOCALIZATION_ERROR;
  }
  msf_factor_fins->SetObservationState(obs);
  msf_factor_fins->SetObservationCov(nav_state.pose_cov);
  msf_factor_fins->SetSource(nav_state.state_source);
  // add FINS observation factor into queue
  if (msf_fusion_ && msf_fusion_->IsStarted()) {
    msf_fusion_->AddFactor(nav_state.timestamp, msf_factor_fins);
  }

  return LOC_SUCCESS;
}

}  // namespace localization
}  // namespace senseAD
