/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */

#include "system/backend/localization_backend_gnss.hpp"

#include <memory>
#include <string>

#include "common/coordinate_converter.hpp"
#include "localization/common/log.hpp"
#include "msf/msf_factor/msf_factor.hpp"
#include "msf/msf_fusion/msf_fusion.hpp"
#include "system/localization_dead_reckoning.hpp"
#include "system/localization_frontend.hpp"

namespace senseAD {
namespace localization {

NavStatus LocalizationBackendGNSS::GetCurrentLocatorStatus() const {
  // check for long periods of missing GNSS signal
  auto loc_frontend = loc_frontend_.lock();
  double last_gnss_time_gap = 0.0;
  {
    std::lock_guard<std::mutex> lock(last_valid_gnss_mutex_);
    last_gnss_time_gap = loc_frontend->GetFrontendTime() * 1.0e-9 -
                         last_valid_gnss_.first * 1.0e-9;
  }
  if (last_gnss_time_gap > 180.0) {  // FATAL as missing GNSS for 3min
    LC_LINFO_EVERY_SEC(SYSTEM, 1)
        << "long time missing GNSS signal, sec: " << last_gnss_time_gap;
    return NavStatus::FATAL_ACCURACY;
  }

  // check whether backend location has huge time delay
  std::lock_guard<std::mutex> lock(conf_mutex_);
  if (conf_list_.empty()) return NavStatus::LOW_ACCURACY;
  double delay_ts = loc_frontend->GetFrontendTime() * 1.0e-9 -
                    conf_list_.back().first * 1.0e-9;
  if (delay_ts > 3.0) return NavStatus::LOW_ACCURACY;

  return gnss_status_;
}

bool LocalizationBackendGNSS::CheckNeedRestart() const {
  std::lock_guard<std::mutex> lock(chi_square_mutex_);
  int counter = std::count_if(chi_square_list_.begin(), chi_square_list_.end(),
                              [](const auto& chi2) { return chi2 > 1000.0; });
  // need restart if continue inconsistency with gnss
  if (counter / static_cast<double>(chi_square_list_.size()) > 0.5) {
    LC_LERROR(SYSTEM) << "navstate and gnss are continuously inconsist";
    return true;
  }
  return false;
}

bool LocalizationBackendGNSS::CheckNeedSwitchOrigin() const {
  std::lock_guard<std::mutex> lick(last_valid_gnss_mutex_);
  if (!last_valid_gnss_.second) return false;
  PointLLH_t origin;
  CoordinateConverter::GetInstance()->GetOrigin(&origin);
  double distance = CoordinateConverter::GetInstance()->ComputeDistanceByLLH(
      origin, last_valid_gnss_.second->position);
  // need switch origin if 100km far from the origin
  if (distance > 1e5) return true;
  return false;
}

adLocStatus_t LocalizationBackendGNSS::InitLocator() {
  locator_ = std::make_shared<GNSSLocator>();
  if (nullptr == locator_) {
    LC_LERROR(SYSTEM) << "Failed to create GNSS locator.";
    return LOC_ALLOC_MEMORY_FAILED;
  }
  if (locator_->Init(param_) != LOC_SUCCESS) {
    LC_LERROR(SYSTEM) << "Failed to init GNSS locator.";
    return LOC_LOCALIZATION_ERROR;
  }
  return LOC_SUCCESS;
}

adLocStatus_t LocalizationBackendGNSS::InitLocEvaluator() {
  return LOC_SUCCESS;
}

adLocStatus_t LocalizationBackendGNSS::InitDataBuffer() {
  InitCapacity();
  static int real_rate = param_.common_param.ins_device == "CHNAV" ? 5 : 1;
  {
    std::lock_guard<std::mutex> lock(conf_mutex_);
    conf_list_.set_capacity(kConfDataWindowSize * real_rate);
  }
  {
    std::lock_guard<std::mutex> lock(chi_square_mutex_);
    chi_square_list_.set_capacity(kChi2DataWindowSize * real_rate);
  }
  return LOC_SUCCESS;
}

void LocalizationBackendGNSS::SetThreadName() {
  pthread_setname_np(thread_->native_handle(), "LC_back_gnss");
}

adLocStatus_t LocalizationBackendGNSS::ResetSubData() {
  ClearData();
  {
    std::lock_guard<std::mutex> lock(last_valid_gnss_mutex_);
    last_valid_gnss_ = std::make_pair(0, nullptr);
  }
  {
    std::lock_guard<std::mutex> lock(conf_mutex_);
    conf_list_.clear();
  }
  {
    std::lock_guard<std::mutex> lock(chi_square_mutex_);
    chi_square_list_.clear();
  }
  gnss_status_ = NavStatus::FATAL_ACCURACY;
  return LOC_SUCCESS;
}

adLocStatus_t LocalizationBackendGNSS::SwitchOriginSubProc() {
  return locator_->SwitchOriginProc();
}

adLocStatus_t LocalizationBackendGNSS::LocatorProcess() {
  // get gnss data from cached buffer
  uint64_t backend_time_stamp;
  static auto gnss_data = std::make_shared<Gnss>();
  {
    std::lock_guard<std::mutex> guard(data_mutex_);
    if (data_list_.empty()) return LOC_LOCALIZATION_ERROR;
    *gnss_data = data_list_.front().second;
    backend_time_stamp = data_list_.front().first;
    // wanna data delay process
    auto loc_frontend = loc_frontend_.lock();
    if (loc_frontend == nullptr ||
        backend_time_stamp >= loc_frontend->GetFrontendTime())
      return LOC_LOCALIZATION_ERROR;
    data_list_.erase(data_list_.begin());
  }

  if (gnss_data->status != GnssStatus::INVALID) {
    std::lock_guard<std::mutex> lock(last_valid_gnss_mutex_);
    static auto save_gnss_data = std::make_shared<Gnss>();
    *save_gnss_data = *gnss_data;
    last_valid_gnss_ = std::make_pair(backend_time_stamp, save_gnss_data);
  }

  // search predicted current backend time's pose, avoid using frontend
  // query pose API, as has postprocessed for smoothness
  NavState nav_state;
  if (!msf_fusion_ || msf_fusion_->SearchNominalState(
                          backend_time_stamp, &nav_state) != LOC_SUCCESS) {
    LC_LDEBUG(SYSTEM) << "Failed to search frontend pose for GNSS, ts "
                      << backend_time_stamp;
  }
  // query odom data
  OdomState odom_state;
  if (LOC_SUCCESS !=
      loc_dr_->QueryOdomPoseByTime(backend_time_stamp, &odom_state)) {
    LC_LDEBUG(SYSTEM) << "Failed to search odom pose for GNSS, ts "
                      << backend_time_stamp;
  }

  // start gnss process
  locator_->SetState(nav_state);
  locator_->SetOdomState(odom_state);
  adLocStatus_t status = locator_->Process(backend_time_stamp, gnss_data);
  if (status != LOC_SUCCESS) {
    LC_LDEBUG(SYSTEM) << "GNSS localization error";
    return LOC_LOCALIZATION_ERROR;
  }

  // get processed state, and convert to observation factor
  double locator_confidence;
  locator_->GetState(&nav_state, &locator_confidence);
  double locator_chi_square = locator_->GetLocatorChiSquare();

  // update gnss localization status
  UpdateNavStatus(nav_state.timestamp, locator_confidence);

  // buffer gnss localization chi square datas
  {
    std::lock_guard<std::mutex> lock(chi_square_mutex_);
    chi_square_list_.push_back(locator_chi_square);
  }

  Eigen::Vector4d obs;
  obs.segment<3>(0) = nav_state.pose.translation();
  // obs(3) = nav_state.pose.so3().toYPR()(0); old Sophus
  obs(3) =
      nav_state.pose.so3().unit_quaternion().toRotationMatrix().eulerAngles(
          2, 1, 0)(0);
  Eigen::Matrix4d obs_cov = nav_state.pose_cov.block<4, 4>(0, 0);

  auto front_locator_type =
      LocatorTypeFromString(param_.common_param.front_locator_type);
  std::shared_ptr<msf::MSFFactor> msf_factor_gnss =
      msf::MSFFactorFactory::CreateMSFFactor(front_locator_type, GNSS);
  if (msf_factor_gnss == nullptr) {
    LC_LERROR(SYSTEM) << "unsupported fusion mode !!!";
    return LOC_LOCALIZATION_ERROR;
  }
  msf_factor_gnss->SetObservationState(obs);
  msf_factor_gnss->SetObservationCov(obs_cov);
  msf_factor_gnss->SetSource(nav_state.state_source);
  // add GNSS observation factor into queue
  if (gnss_data->status != GnssStatus::INVALID && msf_fusion_ &&
      msf_fusion_->IsStarted()) {
    msf_fusion_->AddFactor(backend_time_stamp, msf_factor_gnss);
  }

  return LOC_SUCCESS;
}

void LocalizationBackendGNSS::UpdateNavStatus(uint64_t timestamp,
                                              double confidence) {
  static int real_rate = param_.common_param.ins_device == "CHNAV" ? 5 : 1;
  {
    // buffer locator confidence datas
    std::lock_guard<std::mutex> lock(conf_mutex_);
    conf_list_.push_back(std::make_pair(timestamp, confidence));
    if (conf_list_.size() < kConfDataWindowSize * real_rate) {
      gnss_status_ = NavStatus::LOW_ACCURACY;
      return;
    }
  }

  double rate_gap = 0.0, latest_confidence = 0.0;
  {
    // check backend location frame rate
    std::lock_guard<std::mutex> lock(conf_mutex_);
    double time_gap =
        (conf_list_.back().first * 1.0e-9 - conf_list_.front().first * 1.0e-9);
    rate_gap = std::fabs((conf_list_.size() - 1) / time_gap - real_rate);
    latest_confidence = conf_list_.back().second;
  }

  if (rate_gap < 0.3 * real_rate && latest_confidence > 0.7) {
    gnss_status_ = NavStatus::HIGH_ACCURACY;
  } else if (rate_gap < 0.5 * real_rate && latest_confidence > 0.5) {
    gnss_status_ = NavStatus::MID_ACCURACY;
  } else {
    gnss_status_ = NavStatus::LOW_ACCURACY;
  }
}

}  // namespace localization
}  // namespace senseAD
