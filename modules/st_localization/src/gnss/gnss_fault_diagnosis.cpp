/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Du Jiankui <dujiankui@senseauto.com>
 * Lei Bing <leibing@senseauto.com>
 */

#include "gnss/gnss_fault_diagnosis.hpp"

#include <algorithm>
#include <fstream>
#include <memory>
#include <utility>

#include "ad_time/ad_time.hpp"
#include "common/coordinate_converter.hpp"
#include "common/msf_common.hpp"
#include "common/transform_config.hpp"

namespace senseAD {
namespace localization {

GNSSFaultDiagnostor::GNSSFaultDiagnostor() {
  // Get extrinsic between imu and gnss first
  // p_imufrd_gnss
  auto larm_frd = TransformConfig::GetLeverArm();
  // p_imuflu_gnss
  Eigen::Vector3d larm_flu(larm_frd[0], -larm_frd[1], -larm_frd[2]);
  // rotate -90deg by z-axis, gnss_rfu->imu_flu
  Eigen::Vector3d ypr(-M_PI_2, 0, 0);
  // gnss(RFU) -> imu(FLU)
  // T_imu_gnss_ = SE3d(SO3(ypr), larm_flu); old Sophus
  T_imu_gnss_ = SE3d(SO3d::exp(ypr), larm_flu);

  ypr << 0.0, 0.0, 0.0;
  Eigen::Vector3d t_vb = TransformConfig::GetTvb().translation();
  // T_v_bflu_ = SE3(SO3::exp(ypr), t_vb); old Sophus
  T_v_bflu_ = SE3d(SO3d::exp(ypr), t_vb);

  gnss_dr_windows_.set_capacity(window_size_);
  predict_window_.set_capacity(window_size_);
  gnss_dr_windows_.clear();
  predict_window_.clear();
}

adLocStatus_t GNSSFaultDiagnostor::SwitchOriginProc() {
  // convert enu position to new origin
  for (auto iter = predict_window_.begin(); iter != predict_window_.end();
       ++iter) {
    CoordinateConverter::GetInstance()->ConvertENUToNewOrigin(
        iter->second.translation(), &iter->second.translation());
  }
  return LOC_SUCCESS;
}

adLocStatus_t GNSSFaultDiagnostor::Process(const uint64_t& timestamp,
                                           const Gnss& gnss_data,
                                           const OdomState& odom_state,
                                           const NavState& nav_state) {
  gnss_fd_result_ = 0;
  gnss_dr_rte_ = 0;
  gnss_dr_consistency_ = 0;
  cov_scales_ = Eigen::Vector3d::Ones();
  nav_attitude_cov_ = nav_state.pose_cov.bottomRightCorner(3, 3);

  if (odom_state.timestamp == 0) return LOC_SUCCESS;

  if (last_dr_origin_id_ != odom_state.origin_id) {
    last_dr_origin_id_ = odom_state.origin_id;
    Reset();
    LC_LWARN(GNSS) << "DR restart, reset gnss fault diagnosis module";
  }

  if (LOC_SUCCESS != RangeChecks(gnss_data)) {
    LC_LWARN(GNSS) << "GNSS Range Checks fail";
    gnss_fd_result_ = 1;
  } else if (LOC_SUCCESS != MotionChecks(odom_state)) {
    LC_LWARN(GNSS) << "GNSS Motion Checks fail";
    gnss_fd_result_ = 2;
  } else if (LOC_SUCCESS != FluctuationChecks(gnss_data, odom_state)) {
    LC_LWARN(GNSS) << "GNSS Fluctuation Checks fail";
    gnss_fd_result_ = 3;
  } else if (LOC_SUCCESS != DataSequenceUpdate(gnss_data, odom_state)) {
    LC_LWARN(GNSS) << "GNSS Data SequenceUpdate fail";
  } else if (LOC_SUCCESS != ConsistencyChecks()) {
    LC_LWARN(GNSS) << "GNSS Consistency Checks fail";
    gnss_fd_result_ = 4;
  }

  if (LOC_SUCCESS !=
      TrajPrecictChecks(timestamp, gnss_data, odom_state, nav_state)) {
    LC_LWARN(GNSS) << "GNSS Trajectory Precict Check fail";
    gnss_fd_result_ = -1;
  }

  if (gnss_fd_result_) return LOC_INVALID;

  last_good_gnss_dr_valid_ = true;
  last_good_gnss_dr_ = std::pair<Gnss, OdomState>(gnss_data, odom_state);

  return LOC_SUCCESS;
}

adLocStatus_t GNSSFaultDiagnostor::RangeChecks(const Gnss& gnss_data) {
  // check gnss status and values
  if (gnss_data.status == GnssStatus::INVALID ||
      gnss_data.measurement_time < 0 || gnss_data.position_std_dev.x <= 0 ||
      gnss_data.position_std_dev.y <= 0 || gnss_data.position_std_dev.z <= 0 ||
      gnss_data.linear_velocity_std_dev.x <= 0 ||
      gnss_data.linear_velocity_std_dev.y <= 0 ||
      gnss_data.linear_velocity_std_dev.z <= 0) {
    LC_LWARN(GNSS) << "Gnss status not valid";
    return LOC_INVALID;
  }
  if (gnss_data.status == GnssStatus::SINGLE) {
    bool cov_rescale = false;
    if (gnss_data.position_std_dev.x < 1.0) {
      double new_scale = single_corrected_std_ / gnss_data.position_std_dev.x;
      cov_scales_[0] = new_scale * new_scale / 10;
      LC_LWARN(GNSS) << "Gnss std of position x not valid: "
                     << gnss_data.position_std_dev.x << "|" << cov_scales_[0];
      cov_rescale = true;
    }
    if (gnss_data.position_std_dev.y < 1.0) {
      double new_scale = single_corrected_std_ / gnss_data.position_std_dev.y;
      cov_scales_[1] = new_scale * new_scale / 10;
      LC_LWARN(GNSS) << "Gnss std of position y not valid: "
                     << gnss_data.position_std_dev.y << "|" << cov_scales_[1];
      cov_rescale = true;
    }
    if (gnss_data.position_std_dev.z < 1.0) {
      double new_scale = single_corrected_std_ / gnss_data.position_std_dev.z;
      cov_scales_[2] = new_scale * new_scale / 10;
      LC_LWARN(GNSS) << "Gnss std of position z not valid: "
                     << gnss_data.position_std_dev.z << "|" << cov_scales_[2];
      cov_rescale = true;
    }
    if (cov_rescale) return LOC_INVALID;
  }
  if (std::fabs(gnss_data.position.lon) > pos_lon_threshold_ ||
      std::fabs(gnss_data.position.lat) > pos_lat_threshold_ ||
      std::fabs(gnss_data.position.height) > pos_height_threshold_) {
    LC_LWARN(GNSS) << "Gnss lla beyond threshold";
    return LOC_INVALID;
  }
  if (std::fabs(gnss_data.linear_velocity.x) > vel_threshold_ ||
      std::fabs(gnss_data.linear_velocity.y) > vel_threshold_ ||
      std::fabs(gnss_data.linear_velocity.z) > vel_threshold_) {
    LC_LWARN(GNSS) << "Gnss linear velocity beyond threshold";
    return LOC_INVALID;
  }
  if (gnss_data.position_std_dev.x > pos_std_threshold_ ||
      gnss_data.position_std_dev.y > pos_std_threshold_ ||
      gnss_data.position_std_dev.z > pos_std_threshold_ ||
      gnss_data.linear_velocity_std_dev.x > vel_std_threshold_ ||
      gnss_data.linear_velocity_std_dev.y > vel_std_threshold_ ||
      gnss_data.linear_velocity_std_dev.z > vel_std_threshold_) {
    LC_LWARN(GNSS) << "Gnss position or velocity std beyond threshold";
    return LOC_INVALID;
  }
  return LOC_SUCCESS;
}

adLocStatus_t GNSSFaultDiagnostor::MotionChecks(const OdomState& dr_data) {
  if ((dr_data.linear_speed.norm() > vel_threshold_) ||
      (std::fabs(dr_data.linear_acceleration.norm()) > acc_threshold_)) {
    LC_LWARN(GNSS) << "dr linear speed " << dr_data.linear_speed[0] << " "
                   << dr_data.linear_speed[1] << " " << dr_data.linear_speed[2]
                   << " m/s"
                   << ", dr linear acceleration speed "
                   << dr_data.linear_acceleration[0] << " "
                   << dr_data.linear_acceleration[1] << " "
                   << dr_data.linear_acceleration[2] << " m/s^2";
    return LOC_INVALID;
  }
  return LOC_SUCCESS;
}

adLocStatus_t GNSSFaultDiagnostor::FluctuationChecks(const Gnss& gnss_data,
                                                     const OdomState& dr_data) {
  if (!last_good_gnss_dr_valid_) return LOC_SUCCESS;

  Gnss last_good_gnss = last_good_gnss_dr_.first;
  OdomState last_good_dr = last_good_gnss_dr_.second;

  float64_t gnss_delta_time =
      gnss_data.measurement_time - last_good_gnss.measurement_time;
  if (std::fabs(gnss_delta_time) < 1e-5) return LOC_SUCCESS;

  float64_t gnss_delta_trans = LLA2DIS(last_good_gnss, gnss_data);
  float64_t gnss_delta_trans_rate = gnss_delta_trans / gnss_delta_time;
  if (std::fabs(gnss_delta_trans_rate) > vel_threshold_) {
    LC_LWARN(GNSS) << "Gnss average speed " << gnss_delta_trans_rate
                   << " m/s beyond threshold " << vel_threshold_ << " during "
                   << gnss_delta_time << "s";
    return LOC_INVALID;
  }

  float64_t dr_delta_trans =
      (dr_data.pose.translation() - last_good_dr.pose.translation()).norm();
  if (dr_delta_trans < 10) return LOC_SUCCESS;
  gnss_dr_rte_ =
      std::fabs(gnss_delta_trans - dr_delta_trans) / (dr_delta_trans);
  if ((std::fabs(gnss_dr_rte_) > gnss_dr_rte_threshold_)) {
    LC_LWARN(GNSS) << "FluctuationChecks gnss_dr_rte = " << gnss_dr_rte_;
    return LOC_INVALID;
  }

  return LOC_SUCCESS;
}

adLocStatus_t GNSSFaultDiagnostor::DataSequenceUpdate(
    const Gnss& gnss_data, const OdomState& dr_data) {
  gnss_dr_windows_.push_back(std::pair<Gnss, OdomState>(gnss_data, dr_data));
  return LOC_SUCCESS;
}

adLocStatus_t GNSSFaultDiagnostor::ConsistencyChecks() {
  if (gnss_dr_windows_.size() < window_size_) {
    LC_LWARN(GNSS) << "not full gnss and dr data window";
    return LOC_SUCCESS;
  }

  SE3d dr_now = gnss_dr_windows_.back().second.pose;
  SE3d dr_last = gnss_dr_windows_.front().second.pose;
  float64_t dr_delta_trans =
      (dr_last.translation() - dr_now.translation()).norm();
  if (dr_delta_trans < 10) return LOC_SUCCESS;

  float64_t gnss_yaw = 0;
  float64_t dr_yaw = 0;
  float64_t gnss_dr_yaw_diff = 0;
  float64_t gnss_dr_yaw_diff_sum = 0;
  for (auto iter = gnss_dr_windows_.begin();
       iter != (--(--gnss_dr_windows_.end()));) {
    const auto& gnss_cur = gnss_dr_windows_.begin()->first;
    const auto& dr_cur = gnss_dr_windows_.begin()->second;
    ++iter;
    const auto& gnss_next_1 = iter->first;
    const auto& dr_next_1 = iter->second;
    ++iter;
    const auto& gnss_next_2 = iter->first;
    const auto& dr_next_2 = iter->second;
    --iter;
    double gnss_dis_a = LLA2DIS(gnss_cur, gnss_next_1);
    double gnss_dis_b = LLA2DIS(gnss_next_1, gnss_next_2);
    double gnss_dis_c = LLA2DIS(gnss_cur, gnss_next_2);
    gnss_yaw = std::acos((gnss_dis_a * gnss_dis_a + gnss_dis_b * gnss_dis_b -
                          gnss_dis_c * gnss_dis_c) /
                         (2.0 * gnss_dis_a * gnss_dis_b));
    double dr_dis_a =
        (dr_next_1.pose.translation() - dr_cur.pose.translation()).norm();
    double dr_dis_b =
        (dr_next_2.pose.translation() - dr_next_1.pose.translation()).norm();
    double dr_dis_c =
        (dr_next_2.pose.translation() - dr_cur.pose.translation()).norm();
    dr_yaw = std::acos(
        (dr_dis_a * dr_dis_a + dr_dis_b * dr_dis_b - dr_dis_c * dr_dis_c) /
        (2.0 * dr_dis_a * dr_dis_b));
    gnss_dr_yaw_diff = gnss_yaw - dr_yaw;
    if (gnss_dr_yaw_diff > M_PI) gnss_dr_yaw_diff -= 2.0 * M_PI;
    if (gnss_dr_yaw_diff < -M_PI) gnss_dr_yaw_diff += 2.0 * M_PI;
    gnss_dr_yaw_diff_sum += gnss_dr_yaw_diff * r2d;
  }
  gnss_dr_consistency_ =
      std::fabs(gnss_dr_yaw_diff_sum / (gnss_dr_windows_.size() - 2));

  if (gnss_dr_consistency_ > gnss_dr_consistancy_threshold_) {
    LC_LWARN(GNSS) << "gnss_dr_consistency = " << gnss_dr_consistency_;
    return LOC_INVALID;
  }

  return LOC_SUCCESS;
}

adLocStatus_t GNSSFaultDiagnostor::TrajPrecictChecks(
    const uint64_t& timestamp, const Gnss& gnss_data, const OdomState& dr_data,
    const NavState& nav_state) {
  // get T_enu_gnssi
  SE3d state_enu_imuj;
  if (LOC_SUCCESS != GetIMUPoseInEnu(timestamp, nav_state, &state_enu_imuj)) {
    // if traj predict check not work, it should not influence flow
    return LOC_SUCCESS;
  }

  // gnss lla to enu
  Eigen::Vector3d obs_gnss_pos;
  Position2ENU(gnss_data.position, &obs_gnss_pos);

  SE3d state_enu_gnssj = state_enu_imuj * T_imu_gnss_;
  if (predict_window_.size() < 3) {
    // use rtk gnss to init window, attitude cov is also be considered
    if (gnss_data.status != GnssStatus::RTK_INTEGER ||
        sqrt(std::fabs(nav_attitude_cov_[0] * 57.3)) >
            nav_attitude_cov_th_[0] ||
        sqrt(std::fabs(nav_attitude_cov_[1] * 57.3)) >
            nav_attitude_cov_th_[1] ||
        sqrt(std::fabs(nav_attitude_cov_[2] * 57.3)) >
            nav_attitude_cov_th_[2]) {
      predict_window_.clear();
      LC_LDEBUG(GNSS) << "Waiting to init window: "
                      << sqrt(std::fabs(nav_attitude_cov_[0] * 57.3)) << " "
                      << sqrt(std::fabs(nav_attitude_cov_[1] * 57.3)) << " "
                      << sqrt(std::fabs(nav_attitude_cov_[0] * 57.3));
      return LOC_SUCCESS;
    }
    // use gnss position
    // state_enu_gnssj.setTranslation(obs_gnss_pos); old Sophus
    state_enu_gnssj.translation() = obs_gnss_pos;
    // add gnss to window
    predict_window_.push_back(std::make_pair(timestamp, state_enu_gnssj));
    last_odom_state_ = dr_data;
    LC_LDEBUG(GNSS) << "Initing window: "
                    << sqrt(std::fabs(nav_attitude_cov_[0] * 57.3)) << " "
                    << sqrt(std::fabs(nav_attitude_cov_[1] * 57.3)) << " "
                    << sqrt(std::fabs(nav_attitude_cov_[0] * 57.3));
    return LOC_SUCCESS;
  }

  // p_enu_gnssi
  auto last_valid_data = predict_window_.back();

  // get T_imu0_imui
  if (dr_data.timestamp == 0) {
    // traj check ok if query failed, and add gnss data
    // state_enu_gnssj.setTranslation(obs_gnss_pos); old Sophus
    state_enu_gnssj.translation() = obs_gnss_pos;
    // add gnss to window
    predict_window_.push_back(std::make_pair(timestamp, state_enu_gnssj));
    LC_LWARN(GNSS) << "Fail to query odom pose.";
    return LOC_SUCCESS;
  }
  // detect data blockage, < 300ms
  if ((dr_data.timestamp - last_odom_state_.timestamp) * kNanoSecToSec >
      odom_blockage_th_) {
    LC_LWARN(GNSS) << "Odom data blockage, reset gnss fault diagnosis module: "
                   << (dr_data.timestamp - last_odom_state_.timestamp) *
                          kNanoSecToSec;
    Reset();
    return LOC_SUCCESS;
  }

  // from veh pos(flu) to imu pos(flu)
  SE3d T_imu0_imui = T_v_bflu_.inverse() * last_odom_state_.pose * T_v_bflu_;
  SE3d T_imu0_imuj = T_v_bflu_.inverse() * dr_data.pose * T_v_bflu_;
  // use last dr to predict
  SE3d T_imui_imuj = T_imu0_imui.inverse() * T_imu0_imuj;
  SE3d T_gnssi_gnssj = T_imu_gnss_.inverse() * T_imui_imuj * T_imu_gnss_;
  SE3d predict_gnss = last_valid_data.second * T_gnssi_gnssj;
  Eigen::Vector3d predict_gnss_pos = predict_gnss.translation();
  // use nav state attitude
  // predict_gnss.setRotationMatrix(state_enu_gnssj.rotation_matrix()); old
  // Sophus
  predict_gnss.setRotationMatrix(state_enu_gnssj.rotationMatrix());

  last_odom_state_ = dr_data;

  // compute dr predict error
  Eigen::Vector3d predict_error_vec = predict_gnss.translation() - obs_gnss_pos;
  double predict_horizon_dis =
      sqrt(predict_error_vec[0] * predict_error_vec[0] +
           predict_error_vec[1] * predict_error_vec[1]);
  double predict_height_dis = std::fabs(predict_error_vec[2]);
  double dr_predict_err = predict_horizon_dis + predict_height_dis * 0.8;
  Eigen::Vector3d dr_vec =
      predict_gnss.translation() - last_valid_data.second.translation();

  double odom_distance = dr_vec.norm();
  if (odom_distance > 0.2) {
    acc_dr_dis_th_ = acc_dr_dis_th_ > 0 ? acc_dr_dis_th_ * 1.1
                                        : std::max(odom_distance * 0.05, 0.01);
  } else {
    acc_dr_dis_th_ = odom_distance * 0.05;
  }

  if (GnssStatus::INVALID == gnss_data.status) {
    // handle channel
    ++invalid_num_;
    invalid_acc_dis_ += T_imui_imuj.translation().norm();
  } else {
    invalid_num_ = 0;
    invalid_acc_dis_ = 0;
  }

  double staus_dis_th = 1.0;
  if (dr_predict_err > acc_dr_dis_th_ &&
      predict_error_vec.norm() > staus_dis_th) {  // handle gnss outliers
    gnss_fd_result_ = -1;
    predict_window_.push_back(std::make_pair(timestamp, predict_gnss));
    acc_dr_predict_dis_ += odom_distance;
  } else {  // gnss inliers
    // state_enu_gnssj.translation(obs_gnss_pos); old Sophus
    state_enu_gnssj.translation() = obs_gnss_pos;
    predict_window_.push_back(std::make_pair(timestamp, state_enu_gnssj));
    acc_dr_dis_th_ = 0;
    acc_dr_predict_dis_ = 0;
  }

  if (invalid_num_ > 6 && invalid_acc_dis_ > 50) {
    LC_LWARN(GNSS) << "Driving too long in invalid status, reset gnss "
                      "fault diagnosis module.";
    Reset();
    return LOC_SUCCESS;
  } else if (acc_dr_predict_dis_ > 100) {
    LC_LWARN(GNSS) << "Long time predict with DR, reset gnss fault "
                      "diagnosis module.";
    Reset();
    return LOC_SUCCESS;
  }

  if (gnss_fd_result_ && GnssStatus::INVALID != gnss_data.status) {
    // chi-square check
    if (LOC_SUCCESS !=
        ChiSquareCheck(predict_gnss_pos, obs_gnss_pos, gnss_data)) {
      LC_LWARN(GNSS) << "Chi-square check fail, GNSS fault detected.";
    } else {
      LC_LDEBUG(GNSS) << "Chi-square check pass, GNSS cov is ok.";
    }
    LC_LDEBUG(GNSS) << "GNSS cov scales: " << cov_scales_[1] << " "
                    << cov_scales_[0] << " " << cov_scales_[2];
  }

  if (gnss_fd_result_ == -1) return LOC_INVALID;

  return LOC_SUCCESS;
}

float64_t GNSSFaultDiagnostor::LLA2DIS(const Gnss& gnss_start,
                                       const Gnss& gnss_end) {
  PointLLH_t lla_start;
  lla_start.lat = gnss_start.position.lat;
  lla_start.lon = gnss_start.position.lon;
  lla_start.height = gnss_start.position.height;
  PointENU_t enu_start;
  CoordinateConverter::GetInstance()->LLA2ENU(lla_start, &enu_start);
  PointLLH_t lla_end;
  lla_end.lat = gnss_end.position.lat;
  lla_end.lon = gnss_end.position.lon;
  lla_end.height = gnss_end.position.height;
  PointENU_t enu_end;
  CoordinateConverter::GetInstance()->LLA2ENU(lla_end, &enu_end);
  PointENU_t enu_delta = enu_end - enu_start;
  float64_t gnss_delta_trans =
      std::sqrt(std::pow(enu_delta.x, 2) + std::pow(enu_delta.y, 2) +
                std::pow(enu_delta.z, 2));

  return gnss_delta_trans;
}

void GNSSFaultDiagnostor::Position2ENU(
    const senseAD::localization::PointLLH_t& position, Eigen::Vector3d* enu) {
  PointLLH_t gnss_lla;
  PointENU_t gnss_enu;
  gnss_lla.lat = position.lat;
  gnss_lla.lon = position.lon;
  gnss_lla.height = position.height;
  CoordinateConverter::GetInstance()->LLA2ENU(gnss_lla, &gnss_enu);
  *enu << gnss_enu.x, gnss_enu.y, gnss_enu.z;
}

adLocStatus_t GNSSFaultDiagnostor::GetIMUPoseInEnu(const uint64_t& timestamp,
                                                   const NavState& nav_state,
                                                   SE3d* T_enu_imu) {
  // get vechile pose in enu
  if (nav_state.timestamp == 0) return LOC_INVALID;

  // vehicle -> body(imu center, rfu)
  TransformConfig::FromVehicleToBody(nav_state.pose, T_enu_imu);

  // body(imu center, rfu) -> body(imu center, flu)
  Eigen::Vector3d ypr;
  ypr << -M_PI_2, 0, 0;
  // SE3d T_flu_rfu(SO3(ypr), Eigen::Vector3d::Zero()); old Sophus
  SE3d T_flu_rfu(SO3d::exp(ypr), Eigen::Vector3d::Zero());
  *T_enu_imu *= T_flu_rfu.inverse();

  return LOC_SUCCESS;
}

adLocStatus_t GNSSFaultDiagnostor::ChiSquareCheck(
    const Eigen::Vector3d& predict_gnss_pos,
    const Eigen::Vector3d& obs_gnss_pos, const Gnss& gnss_data) {
  Eigen::Matrix3d cov_mat(Eigen::Matrix3d::Identity());
  cov_mat(0, 0) =
      gnss_data.position_std_dev.x * gnss_data.position_std_dev.x * 10;
  cov_mat(1, 1) =
      gnss_data.position_std_dev.y * gnss_data.position_std_dev.y * 10;
  cov_mat(2, 2) =
      gnss_data.position_std_dev.z * gnss_data.position_std_dev.z * 10;
  double x_chi_value = (predict_gnss_pos[0] - obs_gnss_pos[0]) * 1.0 /
                       cov_mat(0, 0) * (predict_gnss_pos[0] - obs_gnss_pos[0]);
  double y_chi_value = (predict_gnss_pos[1] - obs_gnss_pos[1]) * 1.0 /
                       cov_mat(1, 1) * (predict_gnss_pos[1] - obs_gnss_pos[1]);
  double z_chi_value = (predict_gnss_pos[2] - obs_gnss_pos[2]) * 1.0 /
                       cov_mat(2, 2) * (predict_gnss_pos[2] - obs_gnss_pos[2]);
  cov_scales_[0] =
      x_chi_value > chi2_90s_[1] ? x_chi_value / chi2_90s_[1] : 1.0;
  cov_scales_[1] =
      y_chi_value > chi2_90s_[1] ? y_chi_value / chi2_90s_[1] : 1.0;
  cov_scales_[2] =
      z_chi_value > chi2_90s_[1] ? z_chi_value / chi2_90s_[1] : 1.0;
  if (cov_scales_[0] != 1.0 || cov_scales_[1] != 1.0 || cov_scales_[2] != 1.0)
    return LOC_INVALID;
  return LOC_SUCCESS;
}

void GNSSFaultDiagnostor::Reset() {
  last_good_gnss_dr_valid_ = false;
  gnss_dr_windows_.clear();
  predict_window_.clear();
  invalid_num_ = 0;
  acc_dr_dis_th_ = 0;
  invalid_acc_dis_ = 0;
  acc_dr_predict_dis_ = 0;
  gnss_fd_result_ = 0;
  cov_scales_ = Eigen::Vector3d::Ones();
  last_odom_state_ = OdomState();
}

}  // namespace localization
}  // namespace senseAD
