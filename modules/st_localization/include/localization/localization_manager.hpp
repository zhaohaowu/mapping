/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */
#pragma once

#include <Eigen/Core>
#include <memory>

#include "localization/data_type/base.hpp"
#include "localization/data_type/dual_antenna_heading.hpp"
#include "localization/data_type/gnss.hpp"
#include "localization/data_type/imu.hpp"
#include "localization/data_type/ins.hpp"
#include "localization/data_type/navstate_info.hpp"
#include "localization/data_type/odomstate_info.hpp"
#include "localization/data_type/percept_data.hpp"
#include "localization/data_type/road_structure.hpp"
#include "localization/data_type/vehicle_info.hpp"
#include "localization/localization_param.hpp"

namespace senseAD {
namespace localization {

class LocalizationManagerImpl;

class LocalizationManager {
 public:
  LocalizationManager();

  ~LocalizationManager() = default;

  adLocStatus_t Init(const LocalizationParam& param);

  adLocStatus_t Restart();

  adLocStatus_t ShutDown();

  // set sensor source data

  void SetCanData(uint64_t timestamp, const VehicleInfo& can_data);

  void SetImuData(uint64_t timestamp, const Imu& imu_data);

  void SetInsData(uint64_t timestamp, const Ins& ins_data);

  void SetGnssData(uint64_t timestamp, const Gnss& gnss_data);

  void SetPerceptData(uint64_t timestamp,
                      const std::shared_ptr<PerceptData>& percept_data);

  void SetLocalMapData(uint64_t timestamp,
                       const std::shared_ptr<RoadStructure>& local_map_data);

  void SetNavData(uint64_t timestamp, const NavState& nav_state,
                  const Eigen::Vector3d& t, const Eigen::Vector3d& ypr);

  void SetDualAntHeadingData(uint64_t timestamp,
                             const DualAntennaHeading& dual_ant_heading);

  // get localization results

  adLocStatus_t GetNavStateInfo(NavStateInfo* info) const;

  adLocStatus_t GetOdomStateInfo(OdomStateInfo* info) const;

 private:
  // localization manager implement instance
  std::shared_ptr<LocalizationManagerImpl> localization_manager_impl_;
};

}  // namespace localization
}  // namespace senseAD
