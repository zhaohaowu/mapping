/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */

#pragma once

#include <Eigen/Core>
#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "common/thread_base.hpp"
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

// class holds the localization state monitor, contains data and timestamp valid
// and consistency, frame rate etc.
class LocalizationStateMonitor : public ThreadBase {
 public:
  DEFINE_PTR(LocalizationStateMonitor)

  LocalizationStateMonitor() = delete;
  explicit LocalizationStateMonitor(LocalizationManagerImpl* loc_manager);
  ~LocalizationStateMonitor();

  adLocStatus_t Init(const LocalizationParam& param);

  //////////////////////////// check input data //////////////////////////////

  adLocStatus_t CheckCanData(uint64_t timestamp,
                             const VehicleInfo& vehinfo_data);

  adLocStatus_t CheckImuData(uint64_t timestamp, const Imu& imu_data);

  adLocStatus_t CheckInsData(uint64_t timestamp, const Ins& ins_data);

  adLocStatus_t CheckGnssData(uint64_t timestamp, const Gnss& gnss_data);

  adLocStatus_t CheckPerceptData(
      uint64_t timestamp, const std::shared_ptr<PerceptData>& percept_data);

  adLocStatus_t CheckLocalMapData(
      uint64_t timestamp, const std::shared_ptr<RoadStructure>& local_map_data);

  adLocStatus_t CheckNavData(uint64_t timestamp, const NavState& nav_state);

  adLocStatus_t CheckDualAntHeadingData(
      uint64_t timestamp, const DualAntennaHeading& dual_ant_heading);

  /////////////////////////// check output data //////////////////////////////

  adLocStatus_t CheckNavStateInfoData(const NavStateInfo& info);

  adLocStatus_t CheckOdomStateInfoData(const OdomStateInfo& info);

 private:
  // @brief: monitor process core
  void RunMonitorProc();

  // @brief: check timestamp valid
  adLocStatus_t CheckTimestampValid(const std::string& data_tag,
                                    uint64_t timestamp);

  void SetThreadName() final;

 private:
  static constexpr double g_norm = 9.8;
  static constexpr double window_secs_ = 2.0;

  // Localization config parameters
  LocalizationParam param_;

  // loc manager impl instance (shallow copy)
  LocalizationManagerImpl* loc_manager_;

  // flag for enable restart and switch origin
  bool enable_dr_restart_{false};
  bool enable_loc_restart_{false};
  bool enable_switch_origin_{false};

  // flag for start frame rate check
  std::atomic<bool> is_can_full_{false}, is_imu_full_{false};

  // buffer sensor latest timestamps
  std::mutex st_mutex_;
  std::unordered_map<std::string, std::deque<uint64_t>> sensor_timestamps_;

  // buffer nav state latest timestamps
  std::mutex nav_mutex_;
  std::deque<uint64_t> nav_timestamps_;

  // buffer odom state latest timestamps
  std::mutex odom_mutex_;
  std::deque<uint64_t> odom_timestamps_;

  // buffer sensor data for consistency check
  std::mutex can_mutex_;
  std::pair<uint64_t, VehicleInfo> latest_can_data_{0, VehicleInfo()};
  std::mutex imu_mutex_;
  std::pair<uint64_t, Imu> latest_imu_data_{0, Imu()};
  std::mutex ins_mutex_;
  std::pair<uint64_t, Ins> latest_ins_data_{0, Ins()};
  std::mutex gnss_mutex_;
  std::pair<uint64_t, Gnss> latest_gnss_data_{0, Gnss()};
};

}  // namespace localization
}  // namespace senseAD
