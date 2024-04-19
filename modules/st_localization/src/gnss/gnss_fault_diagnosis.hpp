/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Du Jiankui <dujiankui@senseauto.com>
 * Lei Bing <leibing@senseauto.com>
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <list>
#include <memory>
#include <utility>

#include <Sophus/se3.hpp>
#include <boost/circular_buffer.hpp>

#include "common/coordinate_converter.hpp"
#include "localization/common/log.hpp"
#include "localization/data_type/gnss.hpp"
#include "localization/data_type/odomstate_info.hpp"

namespace senseAD {
namespace localization {

class GNSSFaultDiagnostor {
 public:
  GNSSFaultDiagnostor();

  adLocStatus_t SwitchOriginProc();

  adLocStatus_t Process(const uint64_t& timestamp, const Gnss& gnss_data,
                        const OdomState& odom_state, const NavState& nav_state);

  inline int GetGnssDiagnostictResult() { return gnss_fd_result_; }

  inline float64_t GetFluctuationChecksResult() { return gnss_dr_rte_; }

  inline float64_t GetConsistencyChecksResult() { return gnss_dr_consistency_; }

  inline Eigen::Vector3d GetScaleFactors() { return cov_scales_; }

 private:
  adLocStatus_t DataSequenceUpdate(const Gnss& gnss_data,
                                   const OdomState& dr_data);
  adLocStatus_t RangeChecks(const Gnss& gnss_data);
  adLocStatus_t MotionChecks(const OdomState& dr_data);
  adLocStatus_t FluctuationChecks(const Gnss& gnss_data,
                                  const OdomState& dr_data);
  adLocStatus_t ConsistencyChecks();
  float64_t LLA2DIS(const Gnss& gnss_start, const Gnss& gnss_end);

  void Position2ENU(const senseAD::localization::PointLLH_t& position,
                    Eigen::Vector3d* enu);

  // @brief: check gnss with predicted gnss data
  adLocStatus_t TrajPrecictChecks(const uint64_t& timestamp,
                                  const Gnss& gnss_data,
                                  const OdomState& dr_data,
                                  const NavState& nav_state);

  // @brief: get imu(flu) pose in enu
  adLocStatus_t GetIMUPoseInEnu(const uint64_t& timestamp,
                                const NavState& nav_state, SE3d* T_enu_imu);

  adLocStatus_t ChiSquareCheck(const Eigen::Vector3d& predict_gnss_pos,
                               const Eigen::Vector3d& obs_gnss_pos,
                               const Gnss& gnss_data);

  void Reset();

 private:
  // dr restart monitor
  uint64_t last_dr_origin_id_ = 0;
  // moving window
  const size_t window_size_ = 5;
  boost::circular_buffer<std::pair<Gnss, OdomState>> gnss_dr_windows_;
  boost::circular_buffer<std::pair<uint64_t, SE3d>> predict_window_;
  OdomState last_odom_state_;

  bool last_good_gnss_dr_valid_ = false;
  std::pair<Gnss, OdomState> last_good_gnss_dr_;

  double acc_dr_dis_th_ = 0;
  SE3d T_imu_gnss_;
  SE3d T_v_bflu_;
  // gnss fault diagnosis result
  int gnss_fd_result_ = 0;
  float64_t gnss_dr_rte_ = 0;
  float64_t gnss_dr_consistency_ = 0;

  // cov factor
  Eigen::Vector3d cov_scales_;

  // channel case
  int invalid_num_ = 0;
  double invalid_acc_dis_ = 0;

  // dr blockage time threshold, unit: s
  double odom_blockage_th_ = 1.3;

  // accumulated distance in which predict with dr
  double acc_dr_predict_dis_ = 0;

  // nav state attitude cov
  Eigen::Vector3d nav_attitude_cov_ = Eigen::Vector3d::Zero();
  // nav state attitude cov threshold,unit: deg
  Eigen::Vector3d nav_attitude_cov_th_ = Eigen::Vector3d(0.1, 0.1, 0.1);

  // fault diagnosis threshold
  const float64_t pos_lon_threshold_ = 180;
  const float64_t pos_lat_threshold_ = 90;
  const float64_t pos_height_threshold_ = 1e8;
  const float64_t pos_std_threshold_ = 10;
  const float64_t vel_threshold_ = 40;
  const float64_t vel_std_threshold_ = 10;
  const float64_t num_sats_threshold_ = 4;
  const float64_t acc_threshold_ = 5;
  const float64_t gnss_dr_rte_threshold_ = 0.05;
  const float64_t gnss_dr_consistancy_threshold_ = 10.0;
  const float64_t single_corrected_std_ = 10.0;

  double chi2_995s_[8] = {0.0, 7.88, 10.6, 12.84, 14.86, 16.75, 18.55, 20.28};

  double chi2_90s_[8] = {0.0,   2.706, 4.605,  6.251,
                         7.779, 9.236, 10.645, 12.017};
};

}  // namespace localization
}  // namespace senseAD
