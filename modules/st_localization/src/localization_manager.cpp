/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */

#include "localization/localization_manager.hpp"

#include <Sophus/se3.hpp>

#include "localization_manager_impl.hpp"

namespace senseAD {
namespace localization {

LocalizationManager::LocalizationManager() {
  localization_manager_impl_.reset(new LocalizationManagerImpl);
}

adLocStatus_t LocalizationManager::Init(const LocalizationParam& param) {
  return localization_manager_impl_->Init(param);
}

adLocStatus_t LocalizationManager::Restart() {
  return localization_manager_impl_->Restart();
}

adLocStatus_t LocalizationManager::ShutDown() {
  return localization_manager_impl_->ShutDown();
}

void LocalizationManager::SetCanData(uint64_t timestamp,
                                     const VehicleInfo& can_data) {
  localization_manager_impl_->SetCanData(timestamp, can_data);
}

void LocalizationManager::SetImuData(uint64_t timestamp, const Imu& imu_data) {
  localization_manager_impl_->SetImuData(timestamp, imu_data);
}

void LocalizationManager::SetInsData(uint64_t timestamp, const Ins& ins_data) {
  localization_manager_impl_->SetInsData(timestamp, ins_data);
}

void LocalizationManager::SetGnssData(uint64_t timestamp,
                                      const Gnss& gnss_data) {
  localization_manager_impl_->SetGnssData(timestamp, gnss_data);
}

void LocalizationManager::SetPerceptData(
    uint64_t timestamp, const std::shared_ptr<PerceptData>& percept_data) {
  localization_manager_impl_->SetPerceptData(timestamp, percept_data);
}

void LocalizationManager::SetLocalMapData(
    uint64_t timestamp, const std::shared_ptr<RoadStructure>& local_map_data) {
  localization_manager_impl_->SetLocalMapData(timestamp, local_map_data);
}

void LocalizationManager::SetNavData(uint64_t timestamp,
                                     const NavState& nav_state,
                                     const Eigen::Vector3d& t,
                                     const Eigen::Vector3d& ypr) {
  NavState full_nav_state = nav_state;
  // full_nav_state.pose = SE3d(SO3d(ypr), t); old Sophus
  full_nav_state.pose = SE3d(SO3d::exp(ypr), t);
  localization_manager_impl_->SetNavData(timestamp, full_nav_state);
}

void LocalizationManager::SetDualAntHeadingData(
    uint64_t timestamp, const DualAntennaHeading& dual_ant_heading) {
  localization_manager_impl_->SetDualAntHeadingData(timestamp,
                                                    dual_ant_heading);
}

adLocStatus_t LocalizationManager::GetNavStateInfo(NavStateInfo* info) const {
  return localization_manager_impl_->GetNavStateInfo(info);
}

adLocStatus_t LocalizationManager::GetOdomStateInfo(OdomStateInfo* info) const {
  return localization_manager_impl_->GetOdomStateInfo(info);
}

}  // namespace localization
}  // namespace senseAD
