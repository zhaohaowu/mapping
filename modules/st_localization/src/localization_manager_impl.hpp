/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * Wang Xiaofeng <wangxiaofeng@sensetime.com>
 * Zhao Ming <zhaoming@senseauto.com>
 */
#pragma once

#include <Eigen/Core>
#include <atomic>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>

#include "localization/common/log.hpp"
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

namespace msf {
class MSFFusion;
}
class LocalizationFrontend;
class LocalizationBackend;
class LocalizationDeadReckoning;
class LocalizationStateMonitor;
class LocalizationVisualizer;

class LocalizationManagerImpl {
 public:
  LocalizationManagerImpl() = default;
  ~LocalizationManagerImpl() = default;

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

  void SetNavData(uint64_t timestamp, const NavState& nav_state);

  void SetDualAntHeadingData(uint64_t timestamp,
                             const DualAntennaHeading& dual_ant_heading);

  // get localization results

  adLocStatus_t GetNavStateInfo(NavStateInfo* info) const;

  adLocStatus_t GetOdomStateInfo(OdomStateInfo* info) const;

 private:
  // restart process for relative localization
  adLocStatus_t DRRestartProc();

  // restart process for absolute localization
  adLocStatus_t LocRestartProc();

  // switch origin process for absolute localization
  adLocStatus_t LocSwitchOriginProc();

  adLocStatus_t LoadMapOrigin(const std::string& file_path);

  adLocStatus_t CreateLocalizationFrontend();

  adLocStatus_t BackendEngineFactory(LocatorType type);

  adLocStatus_t CreateLocalizationBackend();

  adLocStatus_t CreateMSF();

  adLocStatus_t CreateLocalizationDR();

  adLocStatus_t CreateLocalizationMonitor();

  adLocStatus_t CreateLocalizationVisualizer();

  adLocStatus_t SetModulePointerLink();

  void StopSetDataAndThread(bool is_abs_loc_mode);

  void ContinueSetDataAndThread(bool is_abs_loc_mode);

  adLocStatus_t ConvertLocalMapCoord(
      uint64_t timestamp, std::shared_ptr<RoadStructure> local_map_data);

  adLocStatus_t ConvertImuToBodyFrame(const Imu& raw_imu, Imu* out_imu);

  void SaveLocOriginLLA(const PointLLH_t& origin_lla);

 private:
  friend class LocalizationStateMonitor;

  // Localization config parameters
  LocalizationParam param_;

  // stop set data flag
  std::atomic<bool> is_loc_stop_data_{false};
  std::atomic<bool> is_dr_stop_data_{false};

  // localization origin set flag
  std::atomic<bool> set_origin_flag_{false};

  // latest gnss data for origin set
  std::mutex gnss_data_for_origin_mutex_;
  Gnss gnss_data_for_origin_;

  // frontend locator type
  LocatorType front_locator_type_;
  // backend locator type
  std::vector<bool> back_locator_type_;

  // system core instance
  std::shared_ptr<msf::MSFFusion> msf_fusion_ = nullptr;

  std::shared_ptr<LocalizationFrontend> loc_frontend_ = nullptr;
  std::map<LocatorType, std::shared_ptr<LocalizationBackend>> loc_backends_;

  std::shared_ptr<LocalizationDeadReckoning> loc_dead_reckoning_ = nullptr;

  std::shared_ptr<LocalizationStateMonitor> loc_monitor_ = nullptr;

  std::shared_ptr<LocalizationVisualizer> loc_visualizer_ = nullptr;
};

}  // namespace localization
}  // namespace senseAD
