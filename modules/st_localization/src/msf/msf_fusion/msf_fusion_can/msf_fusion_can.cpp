/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */

#include "msf/msf_fusion/msf_fusion_can/msf_fusion_can.hpp"

#include "base_locator/base_locator.hpp"
#include "can/can_locator.hpp"
#include "msf/msf_core/msf_core_factory.hpp"

namespace senseAD {
namespace localization {
namespace msf {

MSFFusionCAN::MSFFusionCAN(std::shared_ptr<BaseLocator> locator,
                           const LocalizationParam& param)
    : MSFFusion() {
  (void)param;
  main_filter_ = msf::MSFCoreFactory::CreateMSFCore<MSFCore>(CAN, locator);
}

MSFFusionCAN::~MSFFusionCAN() = default;

adLocStatus_t MSFFusionCAN::Init() { return LOC_SUCCESS; }

adLocStatus_t MSFFusionCAN::AddData(uint64_t time_ns,
                                    const VectorXd& data_reading) {
  std::lock_guard<std::mutex> lock(msf_core_mutex_);
  auto status = main_filter_->AddData(time_ns, data_reading, false);
  latest_frontend_ts_ = time_ns;
  return status;
}

adLocStatus_t MSFFusionCAN::GetLatestMaxErrorState(
    Eigen::Vector4d* error_state) {
  return main_filter_->GetLatestMaxErrorState(error_state);
}

adLocStatus_t MSFFusionCAN::SearchNominalState(uint64_t time_ns,
                                               NavState* nav_state) {
  std::lock_guard<std::mutex> lock(msf_core_mutex_);
  return main_filter_->SearchNominalState(time_ns, nav_state);
}

adLocStatus_t MSFFusionCAN::SwitchOriginSubProc() {
  return main_filter_->SwitchOriginProc();
}

adLocStatus_t MSFFusionCAN::StateUpdate(
    uint64_t timestamp, const std::shared_ptr<MSFFactor>& obs_factor) {
  std::lock_guard<std::mutex> lock(msf_core_mutex_);
  return main_filter_->StateUpdate(timestamp, obs_factor);
}

}  // namespace msf
}  // namespace localization
}  // namespace senseAD
