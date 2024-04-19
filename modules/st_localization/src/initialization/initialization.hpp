/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 * Du Jiankui <dujiankui@senseauto.com>
 */
#pragma once

#include <list>
#include <memory>
#include <mutex>  // NOLINT
#include <utility>
#include <vector>

#include "base_locator/base_locator.hpp"
#include "common/sorted_container.hpp"
#include "localization/data_type/base.hpp"
#include "localization/data_type/dual_antenna_heading.hpp"
#include "localization/data_type/gnss.hpp"
#include "localization/data_type/imu.hpp"
#include "localization/data_type/ins.hpp"
#include "localization/data_type/vehicle_info.hpp"
#include "localization/localization_param.hpp"

namespace senseAD {
namespace localization {

class BaseLocator;
class LocalizationDeadReckoning;

enum class InitStage { NOT_INITED = 0, INIT_ING, INIT_DONE };

#define DATA_DECLARE_ADD_FUNC(TYPE, name)                           \
 public:                                                            \
  void Add##TYPE(uint64_t timestamp, const TYPE& name##_data) {     \
    std::lock_guard<std::mutex> name##_guard(raw_##name##_mutex_);  \
    raw_##name##_.emplace_back(timestamp, name##_data);             \
    while (timestamp * 1e-9 - raw_##name##_.begin()->first * 1e-9 > \
           window_secs_) {                                          \
      raw_##name##_.erase(raw_##name##_.begin());                   \
    }                                                               \
  }                                                                 \
                                                                    \
 private:                                                           \
  std::mutex raw_##name##_mutex_;                                   \
  std::list<std::pair<uint64_t, TYPE>> raw_##name##_;

class Initialization {
 public:
  Initialization() = default;
  ~Initialization() = default;

  adLocStatus_t Init(const LocalizationParam& param);

  void Initialize();

  void SetFrontendLocator(std::shared_ptr<BaseLocator> locator);

  void SetLocalizationDeadReckoning(
      std::shared_ptr<LocalizationDeadReckoning> dr);

  InitStage GetInitStage() const { return init_stage_; }

  adLocStatus_t SearchInitState(uint64_t time_ns, NavState* nav_state);

  adLocStatus_t GetLatestState(NavState* nav_state);

  DATA_DECLARE_ADD_FUNC(Ins, ins);
  DATA_DECLARE_ADD_FUNC(Imu, imu);
  DATA_DECLARE_ADD_FUNC(Gnss, gnss);
  DATA_DECLARE_ADD_FUNC(VehicleInfo, vehinfo);
  DATA_DECLARE_ADD_FUNC(DualAntennaHeading, dual_ant_heading);
  DATA_DECLARE_ADD_FUNC(NavState, mm_state);
  DATA_DECLARE_ADD_FUNC(OdomState, dr);

 private:
  adLocStatus_t InitializeCore();
  adLocStatus_t InitializeINSFrontend();
  adLocStatus_t InitializeCANFrontend();
  adLocStatus_t InitializeIMUFrontend();

  // InitializeIMUFrontend related function
  adLocStatus_t CheckDR();
  adLocStatus_t CheckGNSS();
  adLocStatus_t EstimateGNSSState();
  adLocStatus_t EstimateGNSSBias();
  adLocStatus_t InitializeInitState();
  adLocStatus_t PredictInitState();
  adLocStatus_t UpdateInitState();
  adLocStatus_t CheckInitState();
  adLocStatus_t SetInitState();
  adLocStatus_t AddFactor(uint64_t timestamp, const NavState& obs_factor);
  adLocStatus_t UpdateStateByFactor(uint64_t timestamp,
                                    const NavState& obs_factor);
  adLocStatus_t GetDRByTime(uint64_t timestamp, OdomState* odom_state);
  adLocStatus_t GetDeltaDR(uint64_t start_time, uint64_t end_time,
                           SE3d* delta_dr);
  Eigen::Vector3d LLA2DIS(const Gnss& gnss_start, const Gnss& gnss_end);
  void NavStateInterp(const NavState& s_ns, const NavState& e_ns, double factor,
                      NavState* ns);
  void OdomStateInterp(const OdomState& s_ns, const OdomState& e_ns,
                       double factor, OdomState* ns);
  adLocStatus_t IMUInterp(const uint64_t bef_ts, const Imu& bef_imu,
                          const uint64_t aft_ts, const Imu& aft_imu,
                          uint64_t search_ts, std::shared_ptr<Imu> search_imu);
  adLocStatus_t IMUExtrap(const uint64_t bef1_ts, const Imu& bef1_imu,
                          const uint64_t bef2_ts, const Imu& bef2_imu,
                          const uint64_t search_ts,
                          std::shared_ptr<Imu> search_imu);

 private:
  LocalizationParam param_;

  InitStage init_stage_ = InitStage::NOT_INITED;

  // dr moudle instance
  std::shared_ptr<LocalizationDeadReckoning> loc_dr_;

  // locator related
  LocatorType frontend_type_;
  std::shared_ptr<BaseLocator> frontend_locator_ = nullptr;
  std::vector<bool> back_locator_type_;

  // init navstate
  bool state_inited_{false};
  std::mutex init_navstate_mutex_;
  std::list<std::pair<uint64_t, NavState>> init_navstate_;

  uint64_t last_check_gnss_time_ = 0;
  adLocStatus_t last_check_gnss_status_ = adLocStatus_t::LOC_INVALID;

  std::mutex factor_mutex_;
  std::list<std::pair<uint64_t, NavState>> obs_factor_;
  uint64_t last_gnss_factor_time_ = 0;
  uint64_t last_smm_factor_time_ = 0;

  // struct of observation factor
  struct ObservationFactor {
    uint64_t t_ns;    // timestamp of this factor
    NavState factor;  // observation factor
  };

  // Type of the observation buffer to contain all the observations
  using ObservationBuffer_T = SortedContainer<ObservationFactor>;
  // all the history observations
  ObservationBuffer_T obs_buffer_;

  std::mutex gnss_navstate_mutex_;
  std::list<std::pair<uint64_t, NavState>> gnss_navstate_;

  std::mutex smm_navstate_mutex_;
  std::list<std::pair<uint64_t, NavState>> smm_navstate_;

  std::mutex gnss_bias_mutex_;
  std::list<std::pair<uint64_t, double>> gnss_bias_;
  double optimal_gnss_bias_ = 0;
  bool gnss_bias_updated_ = false;
  bool gnss_bias_estimated_ = false;

  std::list<std::pair<uint64_t, std::pair<Gnss, OdomState>>> gnss_dr_;

  Gnss last_gnss_data_;
  uint64_t last_gnss_time_in_pva_estimate_ = 0;
  uint64_t last_gnss_time_in_bias_estimate_ = 0;

  static constexpr double window_secs_ = 3.0;        // unit: second
  static constexpr double state_window_secs_ = 5.0;  // unit: second
  static constexpr double gnss_dr_consistancy_threshold_ = 5.0;
};

}  // namespace localization
}  // namespace senseAD
