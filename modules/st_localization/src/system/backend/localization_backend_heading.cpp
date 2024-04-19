/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */

#include "system/backend/localization_backend_heading.hpp"

#include <memory>

#include "localization/common/log.hpp"
#include "msf/msf_factor/msf_factor.hpp"
#include "msf/msf_fusion/msf_fusion.hpp"
#include "system/localization_frontend.hpp"

namespace senseAD {
namespace localization {

adLocStatus_t LocalizationBackendHeading::InitLocator() { return LOC_SUCCESS; }

adLocStatus_t LocalizationBackendHeading::InitLocEvaluator() {
  return LOC_SUCCESS;
}

adLocStatus_t LocalizationBackendHeading::InitDataBuffer() {
  InitCapacity();
  return LOC_SUCCESS;
}

void LocalizationBackendHeading::SetThreadName() {
  pthread_setname_np(thread_->native_handle(), "LC_back_head");
}

adLocStatus_t LocalizationBackendHeading::ResetSubData() {
  ClearData();
  for (size_t i = 0; i < heading_arr_size_; ++i) heading_arr_[i] = 0;
  return LOC_SUCCESS;
}

adLocStatus_t LocalizationBackendHeading::SwitchOriginSubProc() {
  return LOC_SUCCESS;
}

adLocStatus_t LocalizationBackendHeading::LocatorProcess() {
  uint64_t backend_time_stamp;
  DualAntennaHeading heading_data;
  {
    std::lock_guard<std::mutex> guard(data_mutex_);
    if (data_list_.empty()) return LOC_LOCALIZATION_ERROR;
    heading_data = data_list_.front().second;
    backend_time_stamp = data_list_.front().first;
    // wanna data delay process
    auto loc_frontend = loc_frontend_.lock();
    if (backend_time_stamp >= loc_frontend->GetFrontendTime())
      return LOC_LOCALIZATION_ERROR;
    data_list_.erase(data_list_.begin());
  }

  // calculate heading covariance according to buffer datas
  double time_d = backend_time_stamp * kNanoSecToSec;
  if (std::fabs(time_d - std::floor(time_d) - 0.500) > 0.010) {
    return LOC_LOCALIZATION_ERROR;
  }
  for (size_t index = 0; index < heading_arr_size_ - 1; ++index) {
    heading_arr_[index] = heading_arr_[index + 1];
  }
  heading_arr_[heading_arr_size_ - 1] = heading_data.heading;
  float32_t ave_heading = 0.0;
  for (size_t index = 0; index < heading_arr_size_; ++index) {
    ave_heading += heading_arr_[index];
  }
  ave_heading /= heading_arr_size_;
  float32_t heading_arr__std = 0.0;
  for (size_t index = 0; index < heading_arr_size_; ++index) {
    heading_arr__std += std::pow(heading_arr_[index] - ave_heading, 2);
  }
  heading_arr__std /= heading_arr_size_;
  heading_arr__std = std::sqrt(heading_arr__std);
  if (heading_arr__std * r2d > 0.5) {
    heading_data.heading_std *= 10;
  } else {
    heading_data.heading_std *= 3;  // vehicle driving in straight line
  }

  Eigen::Matrix<double, 1, 1> obs(heading_data.heading);
  Eigen::Matrix<double, 1, 1> obs_cov;
  obs_cov << heading_data.heading_std * heading_data.heading_std;

  std::shared_ptr<msf::MSFFactor> msf_factor_heading =
      msf::MSFFactorFactory::CreateMSFFactor(IMU, HEADING);
  if (msf_factor_heading == nullptr) {
    LC_LERROR(SYSTEM) << "unsupported fusion mode !!!";
    return LOC_LOCALIZATION_ERROR;
  }
  msf_factor_heading->SetObservationState(obs);
  msf_factor_heading->SetObservationCov(obs_cov);
  msf_factor_heading->SetSource(HEADING);
  // add heading constraint factor into queue
  if (msf_fusion_ && msf_fusion_->IsStarted()) {
    msf_fusion_->AddFactor(backend_time_stamp, msf_factor_heading);
  }

  return LOC_SUCCESS;
}

}  // namespace localization
}  // namespace senseAD
