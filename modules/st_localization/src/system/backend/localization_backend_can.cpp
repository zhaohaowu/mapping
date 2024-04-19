/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */

#include "system/backend/localization_backend_can.hpp"

#include <memory>

#include "common/transform_config.hpp"
#include "localization/common/log.hpp"
#include "msf/msf_factor/msf_factor.hpp"
#include "msf/msf_fusion/msf_fusion.hpp"
#include "system/localization_frontend.hpp"

namespace senseAD {
namespace localization {

adLocStatus_t LocalizationBackendCAN::InitLocator() { return LOC_SUCCESS; }

adLocStatus_t LocalizationBackendCAN::InitLocEvaluator() { return LOC_SUCCESS; }

void LocalizationBackendCAN::SetThreadName() {
  pthread_setname_np(thread_->native_handle(), "LC_back_can");
}

adLocStatus_t LocalizationBackendCAN::InitDataBuffer() {
  InitCapacity();
  return LOC_SUCCESS;
}

adLocStatus_t LocalizationBackendCAN::ResetSubData() {
  ClearData();
  last_can_timestamp_ = 0;
  return LOC_SUCCESS;
}

adLocStatus_t LocalizationBackendCAN::SwitchOriginSubProc() {
  return LOC_SUCCESS;
}

adLocStatus_t LocalizationBackendCAN::LocatorProcess() {
  uint64_t backend_time_stamp;
  static auto can_data = std::make_shared<VehicleInfo>();
  {
    std::lock_guard<std::mutex> guard(data_mutex_);
    if (data_list_.empty()) return LOC_LOCALIZATION_ERROR;
    *can_data = data_list_.front().second;
    backend_time_stamp = data_list_.front().first;
    // wanna data delay process
    auto loc_frontend = loc_frontend_.lock();
    if (backend_time_stamp >= loc_frontend->GetFrontendTime())
      return LOC_LOCALIZATION_ERROR;
    data_list_.erase(data_list_.begin());
  }

  // add odometer constraints at 10 HZ
  constexpr uint64_t interval_time = 1e8;
  if (backend_time_stamp < last_can_timestamp_ ||
      (backend_time_stamp - last_can_timestamp_) <= interval_time)
    return LOC_LOCALIZATION_ERROR;
  last_can_timestamp_ = backend_time_stamp;

  double vehicle_vel =
      0.5 * (can_data->wheel_speed_rl + can_data->wheel_speed_rr);

  // data in the dead zone is not fused
  if (std::fabs(vehicle_vel) < 0.2) return LOC_LOCALIZATION_ERROR;

  // Coordinate system of v_obs for fusion is FRD
  Vector3d v_obs(vehicle_vel, 0.0, 0.0);
  // Considering the accuracy of wheel velocity increase the observation noise
  Matrix3d I3 = Matrix3d::Identity();
  I3(0, 0) =
      param_.msf_param.can_sigma_vel * param_.msf_param.can_sigma_vel * 10;
  I3(1, 1) = 0.4;
  I3(2, 2) = 0.4;

  std::shared_ptr<msf::MSFFactor> msf_factor_motion =
      msf::MSFFactorFactory::CreateMSFFactor(IMU, CAN);
  if (msf_factor_motion == nullptr) {
    LC_LERROR(SYSTEM) << "unsupported fusion mode !!!";
    return LOC_LOCALIZATION_ERROR;
  }
  msf_factor_motion->SetObservationState(v_obs);
  msf_factor_motion->SetObservationCov(I3);
  msf_factor_motion->SetSource(CAN);
  // add motion constraint factor into queue
  if (msf_fusion_ && msf_fusion_->IsStarted()) {
    msf_fusion_->AddFactor(backend_time_stamp, msf_factor_motion);
  }

  return LOC_SUCCESS;
}

}  // namespace localization
}  // namespace senseAD
