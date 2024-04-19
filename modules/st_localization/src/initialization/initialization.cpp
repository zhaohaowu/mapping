/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 * Du Jiankui <dujiankui@senseauto.com>
 */

#include "initialization/initialization.hpp"

#include <Eigen/Core>
#include <algorithm>

#include "can/can_locator.hpp"
#include "common/coordinate_converter.hpp"
#include "common/transform_config.hpp"
#include "imu/imu_locator.hpp"
#include "ins/ins_locator.hpp"
#include "localization/common/log.hpp"
#include "system/localization_dead_reckoning.hpp"

namespace senseAD {
namespace localization {

adLocStatus_t Initialization::Init(const LocalizationParam& param) {
  param_ = param;
  frontend_type_ = LocatorTypeFromString(param.common_param.front_locator_type);
  back_locator_type_ =
      FindLocatorTypeFromString(param_.common_param.back_locator_type);

  LC_LINFO(INITIALIZATION) << "Create Initializer done";
  return LOC_SUCCESS;
}

void Initialization::Initialize() {
  init_stage_ = InitStage::INIT_ING;

  // initialization core process
  if (InitializeCore() != LOC_SUCCESS) {
    LC_LINFO_EVERY_SEC(INITIALIZATION, 1) << "localization initializing";
    return;
  }

  init_stage_ = InitStage::INIT_DONE;
  LC_LINFO(INITIALIZATION) << "localization initialization done";
}

void Initialization::SetFrontendLocator(std::shared_ptr<BaseLocator> locator) {
  frontend_locator_ = locator;
}

void Initialization::SetLocalizationDeadReckoning(
    std::shared_ptr<LocalizationDeadReckoning> dr) {
  loc_dr_ = dr;
}

adLocStatus_t Initialization::GetLatestState(NavState* nav_state) {
  if (nullptr == nav_state) {
    LC_LERROR(INITIALIZATION) << "nullptr";
    return LOC_NULL_PTR;
  }

  std::lock_guard<std::mutex> guard(init_navstate_mutex_);
  if (init_navstate_.empty()) return LOC_LOCALIZATION_ERROR;
  *nav_state = init_navstate_.back().second;

  return LOC_SUCCESS;
}

adLocStatus_t Initialization::SearchInitState(uint64_t time_ns,
                                              NavState* nav_state) {
  if (nullptr == nav_state) return LOC_NULL_PTR;

  std::lock_guard<std::mutex> guard(init_navstate_mutex_);
  // time check
  if (init_navstate_.size() < 2) {
    LC_LDEBUG(INITIALIZATION) << "not enough valid pose window";
    return LOC_INVALID;
  }
  // search tolerance time is 1s
  static constexpr uint64_t time_tolerance = 1e9;
  if (time_ns < (init_navstate_.front().first - time_tolerance)) {
    LC_LDEBUG(INITIALIZATION) << "query time delay window too much";
    return LOC_TIME_DELAY;
  }
  if (time_ns > (init_navstate_.back().first + time_tolerance)) {
    LC_LDEBUG(INITIALIZATION) << "query time ahead window too much";
    return LOC_TIME_AHEAD;
  }

  if (time_ns == init_navstate_.front().first) {
    *nav_state = init_navstate_.front().second;
    return LOC_SUCCESS;
  }
  if (time_ns == init_navstate_.back().first) {
    *nav_state = init_navstate_.back().second;
    return LOC_SUCCESS;
  }

  NavState searched_state;
  SE3d delta_dr;
  if (time_ns < init_navstate_.front().first) {
    if (LOC_SUCCESS !=
        GetDeltaDR(time_ns, init_navstate_.front().first, &delta_dr))
      return LOC_LOCALIZATION_ERROR;
    searched_state = init_navstate_.front().second;
    searched_state.timestamp = time_ns;
    searched_state.pose *= delta_dr.inverse();
  } else if (time_ns > init_navstate_.back().first) {
    if (LOC_SUCCESS !=
        GetDeltaDR(init_navstate_.back().first, time_ns, &delta_dr))
      return LOC_LOCALIZATION_ERROR;
    searched_state = init_navstate_.back().second;
    searched_state.timestamp = time_ns;
    searched_state.pose *= delta_dr;
  } else {
    auto r_iter = init_navstate_.rbegin();
    while (r_iter != init_navstate_.rend() && (r_iter->first > time_ns)) {
      ++r_iter;
    }
    auto init_navstate_before = *r_iter;
    auto init_navstate_after = *(--r_iter);
    uint64_t time_gap_before = time_ns - init_navstate_before.first;
    uint64_t time_gap_after = init_navstate_after.first - time_ns;
    // use the most recent data to estimate
    if ((time_gap_before + time_gap_after) < 2 * time_tolerance) {
      // interpolate to searched time
      double factor = (time_ns * 1e-9 - init_navstate_before.first * 1e-9) /
                      (init_navstate_after.first * 1e-9 -
                       init_navstate_before.first * 1e-9);
      NavStateInterp(init_navstate_before.second, init_navstate_after.second,
                     factor, &searched_state);
    } else if (time_gap_after < time_tolerance) {
      if (LOC_SUCCESS !=
          GetDeltaDR(time_ns, init_navstate_after.first, &delta_dr))
        return LOC_LOCALIZATION_ERROR;
      searched_state = init_navstate_after.second;
      searched_state.timestamp = time_ns;
      searched_state.pose *= delta_dr.inverse();
    } else if (time_gap_before < time_tolerance) {
      if (LOC_SUCCESS !=
          GetDeltaDR(init_navstate_before.first, time_ns, &delta_dr))
        return LOC_LOCALIZATION_ERROR;
      searched_state = init_navstate_before.second;
      searched_state.timestamp = time_ns;
      searched_state.pose *= delta_dr;
    } else {
      LC_LDEBUG(INITIALIZATION) << "time gap is too large " << time_gap_before
                                << " " << time_gap_after;
      return LOC_LOCALIZATION_ERROR;
    }
  }

  *nav_state = searched_state;

  return LOC_SUCCESS;
}

adLocStatus_t Initialization::InitializeCore() {
  // implement for all different frontend types
  if (frontend_type_ == LocatorType::INS) {
    return InitializeINSFrontend();
  } else if (frontend_type_ == LocatorType::CAN) {
    return InitializeCANFrontend();
  } else if (frontend_type_ == LocatorType::IMU) {
    return InitializeIMUFrontend();
  } else {
    LC_LERROR(INITIALIZATION) << "frontend type error";
    return LOC_LOCALIZATION_ERROR;
  }

  return LOC_SUCCESS;
}

adLocStatus_t Initialization::InitializeINSFrontend() {
  uint64_t ins_time;
  auto ins_data = std::make_shared<Ins>();
  {
    std::lock_guard<std::mutex> guard(raw_ins_mutex_);
    if (raw_ins_.empty()) {
      LC_LERROR_EVERY_SEC(INITIALIZATION, 1) << "ins data is empty";
      return LOC_LOCALIZATION_ERROR;
    }
    ins_time = raw_ins_.back().first;
    *ins_data = raw_ins_.back().second;
  }

  // imu data is not important here but necessary for input
  auto imu_data = std::make_shared<Imu>();
  {
    std::lock_guard<std::mutex> guard(raw_imu_mutex_);
    if (raw_imu_.empty()) {
      LC_LERROR_EVERY_SEC(INITIALIZATION, 1) << "imu data is empty";
      return LOC_LOCALIZATION_ERROR;
    }
    *imu_data = raw_imu_.back().second;
  }

  auto frontend_ins = std::dynamic_pointer_cast<INSLocator>(frontend_locator_);
  if (frontend_ins->Process(ins_time, ins_data, imu_data) != LOC_SUCCESS) {
    LC_LERROR(INITIALIZATION) << "failed to initialize ins frontend";
    return LOC_LOCALIZATION_ERROR;
  }

  NavState init_navstate;
  if (frontend_ins->GetState(&init_navstate) == LOC_SUCCESS) {
    std::lock_guard<std::mutex> guard(init_navstate_mutex_);
    init_navstate_.emplace_back(ins_time, init_navstate);
    while (ins_time * 1e-9 - init_navstate_.front().first * 1e-9 >
           window_secs_) {
      init_navstate_.erase(init_navstate_.begin());
    }
  }

  return LOC_SUCCESS;
}

adLocStatus_t Initialization::InitializeCANFrontend() {
  uint64_t vehinfo_time;
  auto vehinfo_data = std::make_shared<VehicleInfo>();
  {
    std::lock_guard<std::mutex> guard(raw_vehinfo_mutex_);
    if (raw_vehinfo_.empty()) {
      LC_LERROR_EVERY_SEC(INITIALIZATION, 1) << "vehinfo data is empty";
      return LOC_LOCALIZATION_ERROR;
    }
    vehinfo_time = raw_vehinfo_.back().first;
    *vehinfo_data = raw_vehinfo_.back().second;
  }

  // search latest ins data which is before vehinfo data
  Ins ins_data;
  {
    std::lock_guard<std::mutex> guard(raw_ins_mutex_);
    if (raw_ins_.empty()) {
      LC_LERROR_EVERY_SEC(INITIALIZATION, 1) << "ins data is empty";
      return LOC_LOCALIZATION_ERROR;
    }
    if (raw_ins_.front().first > vehinfo_time) {
      LC_LDEBUG(INITIALIZATION) << "no suitable ins data to init can locator";
      return LOC_LOCALIZATION_ERROR;
    }
    auto r_iter = raw_ins_.rbegin();
    while (r_iter != raw_ins_.rend() && (r_iter->first > vehinfo_time)) {
      ++r_iter;
    }
    // time gap must be less than 10ms
    if ((vehinfo_time - r_iter->first) > 1e7) {
      LC_LDEBUG(INITIALIZATION) << "no suitable ins data to init can locator";
      return LOC_LOCALIZATION_ERROR;
    }
    ins_data = r_iter->second;
  }

  // set initial state
  PointENU_t ins_enu;
  PointLLH_t ins_lla;
  ins_lla.lat = ins_data.position.lat;
  ins_lla.lon = ins_data.position.lon;
  ins_lla.height = ins_data.position.height;
  CoordinateConverter::GetInstance()->LLA2ENU(ins_lla, &ins_enu);
  Eigen::Vector3d ins_translation(ins_enu.x, ins_enu.y, ins_enu.z);
  Eigen::Vector3d ins_rotation(ins_data.euler_angle.yaw,
                               ins_data.euler_angle.pitch,
                               ins_data.euler_angle.roll);
  SE3d Tw_v0 = INSLocator::ParsePose(ins_rotation, ins_translation,
                                     param_.common_param.ins_device, true);

  auto frontend_can = std::dynamic_pointer_cast<CANLocator>(frontend_locator_);
  frontend_can->SetInitialState(Tw_v0);
  if (frontend_can->Process(vehinfo_time, vehinfo_data) != LOC_SUCCESS) {
    LC_LERROR(INITIALIZATION) << "failed to initialize can frontend";
    return LOC_LOCALIZATION_ERROR;
  }

  NavState init_navstate;
  if (frontend_can->GetState(&init_navstate) == LOC_SUCCESS) {
    std::lock_guard<std::mutex> guard(init_navstate_mutex_);
    init_navstate_.emplace_back(vehinfo_time, init_navstate);
    while (vehinfo_time * 1e-9 - init_navstate_.front().first * 1e-9 >
           window_secs_) {
      init_navstate_.erase(init_navstate_.begin());
    }
  }

  return LOC_SUCCESS;
}

adLocStatus_t Initialization::InitializeIMUFrontend() {
  if (!loc_dr_) {
    LC_LERROR(INITIALIZATION) << "have not got dr instance";
    return LOC_LOCALIZATION_ERROR;
  }
  // add and check dr data
  OdomState odom_state;
  if (loc_dr_->GetLatestState(&odom_state) == LOC_SUCCESS) {
    if (raw_dr_.empty() || odom_state.timestamp > raw_dr_.back().first) {
      AddOdomState(odom_state.timestamp, odom_state);
      // check dr only when new odom state added
      if (CheckDR() != LOC_SUCCESS) {
        LC_LERROR_EVERY_SEC(INITIALIZATION, 1) << "failed to check dr";
        return LOC_LOCALIZATION_ERROR;
      }
    }
  }

  // check gnss data when navstate is not inited
  if (!state_inited_) {
    last_check_gnss_status_ = CheckGNSS();
    if (last_check_gnss_status_ != LOC_SUCCESS) {
      LC_LERROR_EVERY_SEC(INITIALIZATION, 1) << "failed to check gnss";
      return LOC_LOCALIZATION_ERROR;
    }
  }

  // estimate position, velocity, yaw by gnss data
  if (EstimateGNSSState() != LOC_SUCCESS) {
    LC_LERROR_EVERY_SEC(INITIALIZATION, 1)
        << "failed to estimate state by gnss";
  }

  // estimate gnss bias when using non-self-developed map
  if (back_locator_type_[SMM]) {
    if (EstimateGNSSBias() != LOC_SUCCESS) {
      LC_LERROR_EVERY_SEC(INITIALIZATION, 1) << "failed to estimate gnss bias";
    }
  } else {
    gnss_bias_estimated_ = true;
  }

  // estimate state by dr&gnss&smm
  if (!state_inited_) {
    if (InitializeInitState() != LOC_SUCCESS) {
      LC_LERROR_EVERY_SEC(INITIALIZATION, 1) << "failed to estimate init state";
      return LOC_LOCALIZATION_ERROR;
    }
    state_inited_ = true;
  }

  UpdateInitState();

  PredictInitState();

  if (CheckInitState() != LOC_SUCCESS) {
    LC_LERROR_EVERY_SEC(INITIALIZATION, 1) << "failed to check init state";
    return LOC_LOCALIZATION_ERROR;
  }

  // estimate init state
  if (SetInitState() != LOC_SUCCESS) {
    LC_LERROR_EVERY_SEC(INITIALIZATION, 1) << "failed to estimate init state";
    return LOC_LOCALIZATION_ERROR;
  }

  return LOC_SUCCESS;
}

adLocStatus_t Initialization::CheckDR() {
  std::lock_guard<std::mutex> guard(raw_dr_mutex_);
  if (raw_dr_.empty()) {
    LC_LERROR_EVERY_SEC(INITIALIZATION, 1) << "dr data is empty";
    return LOC_LOCALIZATION_ERROR;
  }
  if (GetOdomStatusEnum(raw_dr_.back().second.odom_status) !=
      OdomStatus::GOOD) {
    LC_LWARN(INITIALIZATION) << "dr status is not good, reset initializaiton";
    // reset dr related data
    raw_dr_.clear();
    gnss_dr_.clear();
    state_inited_ = false;
    std::lock_guard<std::mutex> guard(init_navstate_mutex_);
    init_navstate_.clear();
    return LOC_LOCALIZATION_ERROR;
  }

  return LOC_SUCCESS;
}

adLocStatus_t Initialization::CheckGNSS() {
  {
    std::lock_guard<std::mutex> guard(raw_gnss_mutex_);
    if (raw_gnss_.empty()) {
      LC_LERROR_EVERY_SEC(INITIALIZATION, 1) << "gnss data is empty";
      return LOC_LOCALIZATION_ERROR;
    }

    // check gnss only when new gnss data added
    if (raw_gnss_.back().first <= last_check_gnss_time_)
      return last_check_gnss_status_;
    last_check_gnss_time_ = raw_gnss_.back().first;

    // buffer gnss and dr
    uint64_t gnss_time = raw_gnss_.back().first;
    const Gnss& gnss_data = raw_gnss_.back().second;
    // query dr data
    OdomState dr_data;
    if (LOC_SUCCESS != GetDRByTime(gnss_time, &dr_data)) {
      LC_LDEBUG(INITIALIZATION)
          << "failed to query dr data, time: " << gnss_time;
    } else if (gnss_time > gnss_dr_.back().first) {
      gnss_dr_.emplace_back(gnss_time, std::make_pair(gnss_data, dr_data));
      while (gnss_time * 1e-9 - gnss_dr_.front().first * 1e-9 > window_secs_) {
        gnss_dr_.erase(gnss_dr_.begin());
      }
    }

    // check gnss data quantity
    static double gnss_rate =
        param_.common_param.ins_device == "CHNAV" ? 5.0 : 1.0;
    double time_length =
        raw_gnss_.back().first * 1e-9 - raw_gnss_.front().first * 1e-9;
    if ((raw_gnss_.size() < 3) ||
        (raw_gnss_.size() / time_length < 0.7 * gnss_rate)) {
      LC_LERROR_EVERY_SEC(INITIALIZATION, 1) << "no enough gnss data to init";
      LC_LDEBUG(INITIALIZATION) << "time length: " << time_length
                                << " gnss data size: " << raw_gnss_.size();
      return LOC_LOCALIZATION_ERROR;
    }

    // check gnss accuracy
    for (auto iter = raw_gnss_.begin(); iter != raw_gnss_.end(); ++iter) {
      const Gnss& gnss_data = iter->second;
      if ((gnss_data.position_std_dev.x > 10.0) ||
          (gnss_data.position_std_dev.y > 10.0) ||
          (gnss_data.position_std_dev.z > 20.0)) {
        LC_LDEBUG(INITIALIZATION) << "gnss is in low accuracy status";
        LC_LDEBUG(INITIALIZATION)
            << "gnss status: " << GetGnssStatusStr(gnss_data.status)
            << " pos std: " << gnss_data.position_std_dev
            << " vel std: " << gnss_data.linear_velocity_std_dev;
        return LOC_LOCALIZATION_ERROR;
      }
    }
  }

  // check consistancy between gnss and dr
  if (gnss_dr_.size() < 3) {
    LC_LDEBUG(INITIALIZATION) << "no enough valid dr data to check consistancy";
    return LOC_LOCALIZATION_ERROR;
  }

  double dr_delta_trans = (gnss_dr_.back().second.second.pose.translation() -
                           gnss_dr_.front().second.second.pose.translation())
                              .norm();
  if (dr_delta_trans < 10) {
    LC_LDEBUG(INITIALIZATION)
        << "carrier movement distance is less than 10m whthin 3s";
    return LOC_LOCALIZATION_ERROR;
  }

  double gnss_dr_yaw_diff_sum = 0;
  for (auto iter = gnss_dr_.begin(); iter != (--(--gnss_dr_.end()));) {
    const auto& gnss = iter->second.first;
    const auto& dr = iter->second.second;
    ++iter;
    const auto& gnss_next_1 = iter->second.first;
    const auto& dr_next_1 = iter->second.second;
    ++iter;
    const auto& gnss_next_2 = iter->second.first;
    const auto& dr_next_2 = iter->second.second;
    --iter;
    double gnss_dis_a = LLA2DIS(gnss, gnss_next_1).norm();
    double gnss_dis_b = LLA2DIS(gnss_next_1, gnss_next_2).norm();
    double gnss_dis_c = LLA2DIS(gnss, gnss_next_2).norm();
    double gnss_yaw =
        std::acos((gnss_dis_a * gnss_dis_a + gnss_dis_b * gnss_dis_b -
                   gnss_dis_c * gnss_dis_c) /
                  (2.0 * gnss_dis_a * gnss_dis_b));
    double dr_dis_a =
        (dr_next_1.pose.translation() - dr.pose.translation()).norm();
    double dr_dis_b =
        (dr_next_2.pose.translation() - dr_next_1.pose.translation()).norm();
    double dr_dis_c =
        (dr_next_2.pose.translation() - dr.pose.translation()).norm();
    double dr_yaw = std::acos(
        (dr_dis_a * dr_dis_a + dr_dis_b * dr_dis_b - dr_dis_c * dr_dis_c) /
        (2.0 * dr_dis_a * dr_dis_b));
    double gnss_dr_yaw_diff = gnss_yaw - dr_yaw;
    if (gnss_dr_yaw_diff > M_PI) gnss_dr_yaw_diff -= 2.0 * M_PI;
    if (gnss_dr_yaw_diff < -M_PI) gnss_dr_yaw_diff += 2.0 * M_PI;
    gnss_dr_yaw_diff_sum += gnss_dr_yaw_diff * r2d;
  }
  double gnss_dr_consistency =
      std::fabs(gnss_dr_yaw_diff_sum / (gnss_dr_.size() - 2));

  if (gnss_dr_consistency > gnss_dr_consistancy_threshold_) {
    LC_LDEBUG(INITIALIZATION) << "gnss is inconsistent with dr";
    return LOC_LOCALIZATION_ERROR;
  }

  return LOC_SUCCESS;
}

adLocStatus_t Initialization::EstimateGNSSState() {
  uint64_t gnss_time;
  Gnss gnss_data;
  {
    std::lock_guard<std::mutex> guard_gnss(raw_gnss_mutex_);
    if (raw_gnss_.empty()) {
      LC_LERROR_EVERY_SEC(INITIALIZATION, 1) << "gnss data is empty";
      return LOC_LOCALIZATION_ERROR;
    }
    gnss_time = raw_gnss_.back().first;
    gnss_data = raw_gnss_.back().second;
  }

  // velocity must be higher than 5 m/s
  Eigen::Vector3d gnss_vel(gnss_data.linear_velocity.x,
                           gnss_data.linear_velocity.y,
                           gnss_data.linear_velocity.z);
  LC_LDEBUG(INITIALIZATION)
      << "gnss time: " << gnss_time << " gnss vel: " << gnss_vel.norm();
  if (gnss_vel.norm() < 5.0) {
    LC_LERROR_EVERY_SEC(INITIALIZATION, 1)
        << "no suitable gnss data to init imu locator";
    return LOC_LOCALIZATION_ERROR;
  }

  // avoid repeated estimation
  if (gnss_time > last_gnss_time_in_pva_estimate_) {
    if (last_gnss_time_in_pva_estimate_ == 0) {
      // init last gnss data
      last_gnss_data_ = gnss_data;
      last_gnss_time_in_pva_estimate_ = gnss_time;
      LC_LDEBUG(INITIALIZATION) << "init last gnss data";
      return LOC_LOCALIZATION_ERROR;
    }

    Eigen::Vector3d gnss_enu_delta = LLA2DIS(last_gnss_data_, gnss_data);
    double delta_time =
        gnss_data.measurement_time - last_gnss_data_.measurement_time;
    static double gnss_sample_time =
        param_.common_param.ins_device == "CHNAV" ? 0.2 : 1.0;
    if ((delta_time <= 0) || (delta_time >= 2 * gnss_sample_time)) {
      LC_LDEBUG(INITIALIZATION)
          << "gnss data is discontinuous, delta_time = " << delta_time;
      last_gnss_data_ = gnss_data;
      last_gnss_time_in_pva_estimate_ = gnss_time;
      return LOC_LOCALIZATION_ERROR;
    }
    Eigen::Vector3d gnss_pos_diff_vel = gnss_enu_delta / delta_time;
    if ((gnss_pos_diff_vel - gnss_vel).norm() > 1.0) {
      LC_LDEBUG(INITIALIZATION)
          << "gnss vel is inconsistant with gnss pos diff vel,"
          << " gnss_pos_diff_vel: " << gnss_pos_diff_vel(0) << " "
          << gnss_pos_diff_vel(1) << " " << gnss_pos_diff_vel(2) << " "
          << " gnss_vel: " << gnss_vel(0) << " " << gnss_vel(1) << " "
          << gnss_vel(2);
      last_gnss_data_ = gnss_data;
      last_gnss_time_in_pva_estimate_ = gnss_time;
      return LOC_LOCALIZATION_ERROR;
    }

    // estimate velocity
    // ignore lever arm effect
    Eigen::Vector3d vehicle_vel = gnss_vel;

    // estimate yaw
    // zero at east, pi/2 at north
    double vehicle_yaw =
        std::atan2(vehicle_vel(1),
                   std::fabs(vehicle_vel(0)) < 1e-4 ? 1e-4 : vehicle_vel(0));

    // estimate position
    PointENU_t gnss_enu;
    PointLLH_t gnss_lla;
    gnss_lla.lat = gnss_data.position.lat;
    gnss_lla.lon = gnss_data.position.lon;
    gnss_lla.height = gnss_data.position.height;
    CoordinateConverter::GetInstance()->LLA2ENU(gnss_lla, &gnss_enu);
    Eigen::Vector3d gnss_pos(gnss_enu.x, gnss_enu.y, gnss_enu.z);
    Eigen::Vector3d vehicle_ypr(vehicle_yaw, 0.0, 0.0);
    Eigen::Vector3d larm_frd = TransformConfig::GetLeverArm();
    Eigen::Vector3d larm_rfu(larm_frd(1), larm_frd(0), -larm_frd(2));
    Eigen::Matrix4d Tvb = TransformConfig::GetTvb().matrix();
    Eigen::Vector3d larm_offset =
        Utility::ypr2R(vehicle_ypr) *
        (Tvb.block<3, 3>(0, 0) * larm_rfu + Tvb.block<3, 1>(0, 3));
    // position of the vehicle frame(flu) in the reference frame(enu)
    Eigen::Vector3d vehicle_pos = gnss_pos - larm_offset;

    // estimate cov of position and yaw
    double ve_with_error =
        vehicle_vel(0) +
        std::min(2.0, gnss_data.linear_velocity_std_dev.x * 100.0);
    double vn_with_error =
        vehicle_vel(1) +
        std::min(2.0, gnss_data.linear_velocity_std_dev.y * 100.0);
    double vehicle_yaw_with_error = std::atan2(
        vn_with_error, std::fabs(ve_with_error) < 1e-4 ? 1e-4 : ve_with_error);
    double vehicle_yaw_error = vehicle_yaw_with_error - vehicle_yaw;
    if (vehicle_yaw_error > M_PI) vehicle_yaw_error -= 2.0 * M_PI;
    if (vehicle_yaw_error < -M_PI) vehicle_yaw_error += 2.0 * M_PI;
    Eigen::Matrix<double, 6, 6> pose_cov;
    pose_cov.setZero();
    pose_cov(0, 0) = std::pow(gnss_data.position_std_dev.x * 10.0, 2);
    pose_cov(1, 1) = std::pow(gnss_data.position_std_dev.y * 10.0, 2);
    pose_cov(2, 2) = std::pow(gnss_data.position_std_dev.z * 10.0, 2);
    pose_cov(3, 3) = std::pow(5.0 * d2r, 2);
    pose_cov(4, 4) = std::pow(5.0 * d2r, 2);
    pose_cov(5, 5) = std::pow(vehicle_yaw_error, 2);

    // buffer gnss navstate
    NavState gnss_navstate;
    gnss_navstate.timestamp = gnss_time;
    gnss_navstate.nav_status = GetNavStatusNum(NavStatus::INITIALIZING);
    gnss_navstate.state_source = GNSS;

    // gnss_navstate.pose = SE3(SO3(vehicle_ypr), vehicle_pos); old Sophus
    gnss_navstate.pose = SE3d(SO3d::exp(vehicle_ypr), vehicle_pos);
    gnss_navstate.linear_speed = vehicle_vel;
    gnss_navstate.pose_cov = pose_cov;
    {
      std::lock_guard<std::mutex> guard(gnss_navstate_mutex_);
      gnss_navstate_.emplace_back(gnss_time, gnss_navstate);
      LC_LDEBUG(INITIALIZATION)
          << "gnss update time: " << gnss_time << ", gnss pos: " << gnss_enu
          << ", vehicle_pos: " << vehicle_pos.transpose()
          << ", vehicle_ypr: " << vehicle_ypr.transpose();
      while (gnss_time * 1e-9 - gnss_navstate_.front().first * 1e-9 >
             state_window_secs_) {
        gnss_navstate_.erase(gnss_navstate_.begin());
      }
    }

    // avoid repeated estimation
    last_gnss_data_ = gnss_data;
    last_gnss_time_in_pva_estimate_ = gnss_time;
  }

  return LOC_SUCCESS;
}

adLocStatus_t Initialization::EstimateGNSSBias() {
  {
    std::lock_guard<std::mutex> guard(gnss_navstate_mutex_);
    for (auto iter = gnss_navstate_.begin(); iter != gnss_navstate_.end();
         ++iter) {
      uint64_t gnss_navstate_time = iter->first;

      // avoid repeated estimation
      if (gnss_navstate_time > last_gnss_time_in_bias_estimate_) {
        // search latest smm data which is before gnss data
        std::pair<uint64_t, NavState> smm_navstate_before;
        std::pair<uint64_t, NavState> smm_navstate_after;
        {
          std::lock_guard<std::mutex> guard(raw_mm_state_mutex_);
          if (raw_mm_state_.size() < 2) {
            LC_LDEBUG(INITIALIZATION)
                << "no enough map matching data to estimate gnss "
                   "bias";
            return LOC_LOCALIZATION_ERROR;
          }
          if ((raw_mm_state_.front().first > gnss_navstate_time) ||
              (raw_mm_state_.back().first < gnss_navstate_time)) {
            LC_LDEBUG(INITIALIZATION)
                << "no suitable smm data for gnss navstate "
                   "data mapping, time: "
                << gnss_navstate_time << " " << raw_mm_state_.front().first
                << " " << raw_mm_state_.back().first;
            continue;
          }
          auto r_iter = raw_mm_state_.rbegin();
          while (r_iter != raw_mm_state_.rend() &&
                 (r_iter->first > gnss_navstate_time)) {
            ++r_iter;
          }
          smm_navstate_before = *r_iter;
          smm_navstate_after = *(--r_iter);
        }

        // time gap must be less than 400ms
        if ((smm_navstate_after.first - smm_navstate_before.first) > 4e8) {
          LC_LDEBUG(INITIALIZATION)
              << "time gap is too large " << smm_navstate_before.first << " "
              << smm_navstate_after.first;
          continue;
        }
        // check smm status
        double smm_lat_cov_limit = std::pow(0.5, 2);
        double smm_yaw_cov_limit = std::pow(3.0 * d2r, 2);
        if ((smm_navstate_before.second.pose_cov(1, 1) > smm_lat_cov_limit) ||
            (smm_navstate_before.second.pose_cov(5, 5) > smm_yaw_cov_limit) ||
            (smm_navstate_after.second.pose_cov(1, 1) > smm_lat_cov_limit) ||
            (smm_navstate_after.second.pose_cov(5, 5) > smm_yaw_cov_limit)) {
          LC_LDEBUG(INITIALIZATION) << "smm is in low accuracy status";
          continue;
        }

        // interpolate to gnss time
        NavState smm_navstate;
        double factor =
            (gnss_navstate_time * 1e-9 - smm_navstate_before.first * 1e-9) /
            (smm_navstate_after.first * 1e-9 -
             smm_navstate_before.first * 1e-9);
        NavStateInterp(smm_navstate_before.second, smm_navstate_after.second,
                       factor, &smm_navstate);
        {
          std::lock_guard<std::mutex> guard(smm_navstate_mutex_);
          smm_navstate_.emplace_back(gnss_navstate_time, smm_navstate);
          while (gnss_navstate_time * 1e-9 -
                     smm_navstate_.front().first * 1e-9 >
                 state_window_secs_) {
            smm_navstate_.erase(smm_navstate_.begin());
          }
        }

        // estimate bias between gnss and map
        Eigen::Vector3d gnss_bias =
            smm_navstate.pose.so3().matrix().inverse() *
            (iter->second.pose.translation() - smm_navstate.pose.translation());
        LC_LDEBUG(INITIALIZATION)
            << "current est gnss bias: " << gnss_navstate_time << ", "
            << gnss_bias(0) << " " << gnss_bias(1) << " " << gnss_bias(2);
        {
          std::lock_guard<std::mutex> guard(gnss_bias_mutex_);
          gnss_bias_.emplace_back(gnss_navstate_time, gnss_bias(1));
          while (gnss_navstate_time * 1e-9 - gnss_bias_.front().first * 1e-9 >
                 1.3 * state_window_secs_) {
            gnss_bias_.erase(gnss_bias_.begin());
          }
          gnss_bias_updated_ = true;
        }

        // avoid repeated estimation
        last_gnss_time_in_bias_estimate_ = gnss_navstate_time;
      }
    }
  }

  if (gnss_bias_updated_) {
    gnss_bias_updated_ = false;
    std::lock_guard<std::mutex> guard(gnss_bias_mutex_);

    // check whether gnss bias estimation converges
    if ((gnss_bias_.back().first - gnss_bias_.front().first) <
        state_window_secs_ * 1e9) {
      LC_LDEBUG(INITIALIZATION) << "no enough data to estimate gnss bias";
      return LOC_LOCALIZATION_ERROR;
    }

    double gnss_bias_min = gnss_bias_.begin()->second;
    double gnss_bias_max = gnss_bias_.begin()->second;
    double gnss_bias_mean = 0;
    for (auto iter = gnss_bias_.begin(); iter != gnss_bias_.end(); ++iter) {
      gnss_bias_min = std::min(gnss_bias_min, iter->second);
      gnss_bias_max = std::max(gnss_bias_max, iter->second);
      gnss_bias_mean += iter->second;
    }
    gnss_bias_mean /= gnss_bias_.size();

    // use max-min or variance to check
    if (gnss_bias_.size() < 5) {
      double gnss_bias_max_min = gnss_bias_max - gnss_bias_min;
      if (gnss_bias_max_min > 0.2) {
        LC_LDEBUG(INITIALIZATION)
            << "gnss bias max-min check failed, max-min: " << gnss_bias_max_min;
        return LOC_LOCALIZATION_ERROR;
      }
    } else {
      double gnss_bias_var = 0;
      for (auto iter = gnss_bias_.begin(); iter != gnss_bias_.end(); ++iter) {
        gnss_bias_var += std::pow((iter->second - gnss_bias_mean), 2);
      }
      gnss_bias_var /= gnss_bias_.size();
      if (gnss_bias_var > 0.04) {
        LC_LDEBUG(INITIALIZATION)
            << "gnss bias var check failed, var: " << gnss_bias_var;
        return LOC_LOCALIZATION_ERROR;
      }
    }

    optimal_gnss_bias_ = gnss_bias_mean;
    gnss_bias_estimated_ = true;
    LC_LDEBUG(INITIALIZATION) << "optimal gnss bias: " << optimal_gnss_bias_;
    return LOC_SUCCESS;
  }

  return LOC_LOCALIZATION_ERROR;
}

adLocStatus_t Initialization::InitializeInitState() {
  uint64_t init_gnss_time;
  NavState init_gnss_navstate;
  {
    std::lock_guard<std::mutex> guard(gnss_navstate_mutex_);
    if (gnss_navstate_.empty()) {
      LC_LERROR_EVERY_SEC(INITIALIZATION, 1) << "gnss navstate window is empty";
      return LOC_LOCALIZATION_ERROR;
    }
    init_gnss_time = gnss_navstate_.back().first;
    init_gnss_navstate = gnss_navstate_.back().second;
  }
  // search dr data
  OdomState searched_dr;
  if (LOC_SUCCESS != GetDRByTime(init_gnss_time, &searched_dr)) {
    LC_LERROR(INITIALIZATION)
        << "failed to query dr data, ts " << init_gnss_time;
    return LOC_LOCALIZATION_ERROR;
  }
  // reset dr data by gnss navstate
  {
    std::lock_guard<std::mutex> guard(raw_dr_mutex_);
    for (auto iter = raw_dr_.begin(); iter != raw_dr_.end(); ++iter) {
      uint64_t dr_time = iter->first;
      OdomState dr_data = iter->second;
      NavState init_navstate = init_gnss_navstate;
      init_navstate.timestamp = dr_data.timestamp;
      init_navstate.linear_acceleration = dr_data.linear_acceleration;
      init_navstate.angular_speed = dr_data.angular_speed;
      if (dr_time < init_gnss_time) {
        init_navstate.pose *=
            (dr_data.pose.inverse() * searched_dr.pose).inverse();
      } else {
        init_navstate.pose *= (searched_dr.pose.inverse() * dr_data.pose);
      }
      // reset roll and pitch of init_navstate by dr data
      double init_navstate_yaw =
          Utility::R2ypr(init_navstate.pose.so3().matrix())(0);
      Eigen::Vector3d dr_ypr = Utility::R2ypr(dr_data.pose.so3().matrix());
      // init_navstate.pose.so3() = SO3(
      //     Eigen::Vector3d(init_navstate_yaw, dr_ypr(1), dr_ypr(2))); old
      //     Sophus
      init_navstate.pose.so3() =
          SO3d::exp(Eigen::Vector3d(init_navstate_yaw, dr_ypr(1), dr_ypr(2)));
      // linear_speed of init_navstate is in ENU frame
      init_navstate.linear_speed =
          init_navstate.pose.so3().matrix() * dr_data.linear_speed;
      {
        std::lock_guard<std::mutex> guard(init_navstate_mutex_);
        init_navstate_.emplace_back(dr_time, init_navstate);
        while (dr_time * 1e-9 - init_navstate_.front().first * 1e-9 >
               window_secs_) {
          init_navstate_.erase(init_navstate_.begin());
        }
      }
    }
  }

  last_gnss_factor_time_ = init_gnss_time;
  LC_LINFO(INITIALIZATION) << "state init success";

  return LOC_SUCCESS;
}

adLocStatus_t Initialization::PredictInitState() {
  uint64_t current_time;
  NavState current_state;
  {
    std::lock_guard<std::mutex> guard(init_navstate_mutex_);
    current_time = init_navstate_.back().first;
    current_state = init_navstate_.back().second;
  }
  {
    std::lock_guard<std::mutex> guard(raw_dr_mutex_);
    // search dr data which is equal to current time
    if (raw_dr_.size() < 2) {
      LC_LDEBUG(INITIALIZATION) << "no enough dr data to state predict";
      return LOC_LOCALIZATION_ERROR;
    }
    if (current_time >= raw_dr_.back().first) return LOC_SUCCESS;
    auto r_iter = raw_dr_.rbegin();
    while (r_iter != raw_dr_.rend() && (r_iter->first > current_time)) {
      ++r_iter;
    }
    OdomState searched_dr = r_iter->second;
    // predict init state by dr data
    for (auto iter = r_iter.base(); iter != raw_dr_.end(); ++iter) {
      uint64_t dr_time = iter->first;
      OdomState dr_data = iter->second;
      NavState init_navstate = current_state;
      init_navstate.timestamp = dr_data.timestamp;
      init_navstate.linear_acceleration = dr_data.linear_acceleration;
      init_navstate.angular_speed = dr_data.angular_speed;
      init_navstate.pose *= (searched_dr.pose.inverse() * dr_data.pose);
      // reset roll and pitch of init_navstate by dr data
      double init_navstate_yaw =
          Utility::R2ypr(init_navstate.pose.so3().matrix())(0);
      Eigen::Vector3d dr_ypr = Utility::R2ypr(dr_data.pose.so3().matrix());
      // init_navstate.pose.so3() = SO3d(
      //     Eigen::Vector3d(init_navstate_yaw, dr_ypr(1), dr_ypr(2))); old
      //     Sophus
      init_navstate.pose.so3() =
          SO3d::exp(Eigen::Vector3d(init_navstate_yaw, dr_ypr(1), dr_ypr(2)));
      // linear_speed of init_navstate is in ENU frame
      init_navstate.linear_speed =
          init_navstate.pose.so3().matrix() * dr_data.linear_speed;
      // dr error in short time and short distance: 10%, 0.5deg/m
      // delta pose cov is set the same in each dimension
      // no need to transform to global
      double distance =
          (searched_dr.pose.inverse() * dr_data.pose).translation().norm();
      init_navstate.pose_cov.block<3, 3>(0, 0) +=
          Eigen::Matrix3d::Identity() * std::pow(distance * 0.1, 2);
      init_navstate.pose_cov.block<3, 3>(3, 3) +=
          Eigen::Matrix3d::Identity() * std::pow(distance * 0.5 * d2r, 2);
      {
        std::lock_guard<std::mutex> guard(init_navstate_mutex_);
        init_navstate_.emplace_back(dr_time, init_navstate);
        while (dr_time * 1e-9 - init_navstate_.front().first * 1e-9 >
               window_secs_) {
          init_navstate_.erase(init_navstate_.begin());
        }
      }
    }
  }
  return LOC_SUCCESS;
}

adLocStatus_t Initialization::UpdateInitState() {
  // add gnss factor
  {
    std::lock_guard<std::mutex> guard(gnss_navstate_mutex_);
    if (!gnss_navstate_.empty()) {
      auto iter = gnss_navstate_.rbegin();
      while (iter != gnss_navstate_.rend() &&
             (iter->first > last_gnss_factor_time_)) {
        NavState gnss_navstate = iter->second;
        if (gnss_bias_estimated_) {
          // remove gnss position bias
          SE3d gnss_pose_enu_flu = gnss_navstate.pose;
          Eigen::Vector3d gnss_t_without_bias =
              gnss_pose_enu_flu.translation() -
              gnss_pose_enu_flu.so3().matrix().middleCols(1, 1) *
                  optimal_gnss_bias_;
          // gnss_pose_enu_flu.setTranslation(gnss_t_without_bias); old Sophus
          gnss_pose_enu_flu.translation() = gnss_t_without_bias;
          gnss_navstate.pose = gnss_pose_enu_flu;
        }
        AddFactor(iter->first, gnss_navstate);
        iter++;
      }
      last_gnss_factor_time_ = gnss_navstate_.back().first;
    }
  }
  // add smm factor
  if (back_locator_type_[SMM]) {
    std::lock_guard<std::mutex> guard(raw_mm_state_mutex_);
    if (!raw_mm_state_.empty()) {
      auto iter = raw_mm_state_.rbegin();
      while (iter != raw_mm_state_.rend() &&
             (iter->first > last_smm_factor_time_)) {
        // do not check smm status, fuse all smm data
        // transform pos cov from local to global
        NavState smm_obs = iter->second;
        Eigen::Matrix3d pos_cov_local =
            iter->second.pose_cov.topLeftCorner(3, 3);
        Eigen::Matrix3d rot = iter->second.pose.so3().matrix();
        Eigen::Matrix3d pos_cov_global = rot * pos_cov_local * rot.transpose();
        smm_obs.pose_cov.topLeftCorner(3, 3) = pos_cov_global;
        AddFactor(iter->first, smm_obs);
        iter++;
      }
      last_smm_factor_time_ = raw_mm_state_.back().first;
    }
  }

  uint64_t latest_init_state_time;
  {
    std::lock_guard<std::mutex> guard(init_navstate_mutex_);
    latest_init_state_time = init_navstate_.back().first;
  }

  // update state by obs factor
  {
    std::lock_guard<std::mutex> lock(factor_mutex_);
    while (!obs_factor_.empty()) {
      // avoid factor timestamp is ahead of init_state timestamp
      if (obs_factor_.front().first > latest_init_state_time) break;
      UpdateStateByFactor(obs_factor_.front().first,
                          obs_factor_.front().second);
      obs_factor_.pop_front();
    }
  }
  return LOC_SUCCESS;
}

adLocStatus_t Initialization::CheckInitState() {
  if ((back_locator_type_[SMM]) && (!gnss_bias_estimated_))
    return LOC_LOCALIZATION_ERROR;

  NavState nav_state;
  {
    std::lock_guard<std::mutex> guard(init_navstate_mutex_);
    if (init_navstate_.empty()) return LOC_LOCALIZATION_ERROR;
    nav_state = init_navstate_.back().second;
  }
  // check nav_state status
  double pos_cov_limit = std::pow(3.0, 2);
  double att_cov_limit = std::pow(3.0 * d2r, 2);
  if ((nav_state.pose_cov(0, 0) > pos_cov_limit) ||
      (nav_state.pose_cov(1, 1) > pos_cov_limit) ||
      (nav_state.pose_cov(2, 2) > 4.0 * pos_cov_limit) ||
      (nav_state.pose_cov(3, 3) > att_cov_limit) ||
      (nav_state.pose_cov(4, 4) > att_cov_limit) ||
      (nav_state.pose_cov(5, 5) > att_cov_limit)) {
    LC_LDEBUG(INITIALIZATION) << "init navstate is in low accuracy status";
    return LOC_LOCALIZATION_ERROR;
  }

  return LOC_SUCCESS;
}

adLocStatus_t Initialization::SetInitState() {
  // get init state
  uint64_t init_time;
  NavState init_navstate;
  {
    std::lock_guard<std::mutex> guard(init_navstate_mutex_);
    if (init_navstate_.empty()) return LOC_LOCALIZATION_ERROR;
    init_time = init_navstate_.back().first;
    init_navstate = init_navstate_.back().second;
  }

  // transform to body frame
  SE3d state_enu_flu = init_navstate.pose;
  SE3d state_enu_rfu;
  TransformConfig::FromVehicleToBody(state_enu_flu, &state_enu_rfu);
  PointENU_t init_enu;
  PointLLH_t init_lla;
  init_enu.x = state_enu_rfu.translation()(0);
  init_enu.y = state_enu_rfu.translation()(1);
  init_enu.z = state_enu_rfu.translation()(2);
  CoordinateConverter::GetInstance()->ENU2LLA(init_enu, &init_lla);
  // SE3d Trfu_frd =
  //     SE3d(SO3d(Eigen::Vector3d(M_PI_2, 0.0, M_PI)), Vector3d::Zero()); old
  //     Sophus
  SE3d Trfu_frd =
      SE3d(SO3d::exp(Eigen::Vector3d(M_PI_2, 0.0, M_PI)), Vector3d::Zero());
  SE3d Tned_enu = Trfu_frd;
  SE3d state_ned_frd = Tned_enu * state_enu_rfu * Trfu_frd;
  state_ned_frd.normalize();
  Eigen::Vector3d ypr_ned_frd = Utility::R2ypr(state_ned_frd.so3().matrix());
  Eigen::Vector3d init_vel(init_navstate.linear_speed(1),
                           init_navstate.linear_speed(0),
                           -init_navstate.linear_speed(2));
  double init_gnss_bias = -optimal_gnss_bias_;
  LC_LINFO(INITIALIZATION) << "refined init lla: " << init_lla;
  LC_LINFO(INITIALIZATION) << "refined init ypr: " << ypr_ned_frd(0) * r2d
                           << " " << ypr_ned_frd(1) * r2d << " "
                           << ypr_ned_frd(2) * r2d;
  LC_LINFO(INITIALIZATION) << "refined init vel: " << init_vel(0) << " "
                           << init_vel(1) << " " << init_vel(2);
  LC_LINFO(INITIALIZATION) << "refined init_gnss_bias : " << init_gnss_bias;

  auto frontend_imu = std::dynamic_pointer_cast<IMULocator>(frontend_locator_);
  // frd-ned
  frontend_imu->SetInitialState(init_lla.lat * d2r, init_lla.lon * d2r,
                                init_lla.height, ypr_ned_frd, init_vel,
                                init_gnss_bias);
  // use init_navstate pose_cov to set covariance
  frontend_imu->GetSINS()->UpdateGeoParams();
  Eigen::Matrix3d Tpr = TransformConfig::GetTref().so3().matrix() *
                        frontend_imu->GetSINS()->GetTpr();
  Eigen::Matrix3d init_pos_cov = Tpr.inverse() *
                                 init_navstate.pose_cov.topLeftCorner(3, 3) *
                                 Tpr.inverse().transpose();
  double init_yaw_cov = init_navstate.pose_cov(5, 5);
  double sigma_init_ll =
      0.5 * (std::sqrt(init_pos_cov(0, 0)) + std::sqrt(init_pos_cov(1, 1)));
  double sigma_init_a = std::sqrt(init_yaw_cov);
  double sigma_init_v = 0.5;
  double sigma_init_acc_b = 0.01;
  double sigma_init_gyro_b = 0.01;
  double sigma_init_acc_s = 0.01;
  double sigma_init_gyro_s = 0.01;
  auto square = [](double a) { return a * a; };
  frontend_imu->GetSINS()->MakeErrorStateCov(
      square(sigma_init_ll), square(sigma_init_v), square(sigma_init_a),
      square(sigma_init_acc_b), square(sigma_init_gyro_b),
      square(sigma_init_acc_s), square(sigma_init_gyro_s));
  LC_LINFO(INITIALIZATION) << "refined init lat&lon std: "
                           << sigma_init_ll * r2d;
  LC_LINFO(INITIALIZATION) << "refined init attitude std: "
                           << sigma_init_a * r2d;

  {
    std::lock_guard<std::mutex> guard(raw_imu_mutex_);
    // search latest imu data which is before init data
    auto r_iter = raw_imu_.rbegin();
    while (r_iter != raw_imu_.rend() && (r_iter->first > init_time)) {
      ++r_iter;
    }
    // time gap must be less than 100ms
    if ((init_time < r_iter->first) || (init_time - r_iter->first) > 1e8) {
      LC_LDEBUG(INITIALIZATION) << "failed to initialize imu frontend";
      return LOC_LOCALIZATION_ERROR;
    }
    auto bef_iter = --(r_iter.base());
    uint64_t imu_time = init_time;
    auto imu_data = std::make_shared<Imu>();
    if (init_time == bef_iter->first) {
      *imu_data = bef_iter->second;
    } else {
      if (raw_imu_.size() < 2) return LOC_LOCALIZATION_ERROR;
      if (bef_iter != (--raw_imu_.end())) {
        // avoid same imu timestamp
        auto aft_iter = std::next(bef_iter);
        while ((aft_iter != raw_imu_.end()) &&
               (aft_iter->first == bef_iter->first))
          aft_iter++;
        // interpolation
        if (IMUInterp(bef_iter->first, bef_iter->second, aft_iter->first,
                      aft_iter->second, init_time, imu_data) != LOC_SUCCESS) {
          LC_LDEBUG(INITIALIZATION) << "failed to initialize imu frontend";
          return LOC_LOCALIZATION_ERROR;
        }
      } else {
        // avoid same imu timestamp
        auto bef1_iter = std::prev(bef_iter);
        while ((bef1_iter != raw_imu_.begin()) &&
               (bef1_iter->first == bef_iter->first))
          bef1_iter--;
        // extrapolation
        if (IMUExtrap(bef1_iter->first, bef1_iter->second, bef_iter->first,
                      bef_iter->second, init_time, imu_data) != LOC_SUCCESS) {
          LC_LDEBUG(INITIALIZATION) << "failed to initialize imu frontend";
          return LOC_LOCALIZATION_ERROR;
        }
      }
    }
    if (frontend_imu->Process(imu_time, imu_data) != LOC_SUCCESS) {
      LC_LERROR(INITIALIZATION) << "failed to initialize imu frontend";
      return LOC_LOCALIZATION_ERROR;
    }
    // predict to current frontend time
    for (auto iter = std::next(bef_iter); iter != raw_imu_.end(); ++iter) {
      imu_time = iter->first;
      *imu_data = iter->second;
      if (frontend_imu->Process(imu_time, imu_data) != LOC_SUCCESS) {
        LC_LERROR(INITIALIZATION) << "failed to initialize imu frontend";
        return LOC_LOCALIZATION_ERROR;
      }
    }
  }

  return LOC_SUCCESS;
}

adLocStatus_t Initialization::AddFactor(uint64_t timestamp,
                                        const NavState& obs_factor) {
  std::lock_guard<std::mutex> lock(factor_mutex_);
  if (obs_factor_.empty()) {
    obs_factor_.emplace_back(std::make_pair(timestamp, obs_factor));
    return LOC_SUCCESS;
  }

  if (timestamp > obs_factor_.back().first) {
    obs_factor_.emplace_back(std::make_pair(timestamp, obs_factor));
    return LOC_SUCCESS;
  }

  for (auto iter = obs_factor_.begin(); iter != obs_factor_.end(); ++iter) {
    if (timestamp < iter->first) {
      obs_factor_.insert(iter, std::make_pair(timestamp, obs_factor));
      break;
    } else if (timestamp == iter->first) {
      if ((*iter).second.state_source < obs_factor.state_source) {
        (*iter).second = obs_factor;
        LC_LDEBUG(INITIALIZATION) << "factor has same timestamp, replace...";
        break;
      }
    }
  }
  return LOC_SUCCESS;
}

adLocStatus_t Initialization::UpdateStateByFactor(uint64_t timestamp,
                                                  const NavState& obs_factor) {
  std::shared_ptr<ObservationFactor> obs_factor_ptr =
      std::make_shared<ObservationFactor>();
  obs_factor_ptr->t_ns = timestamp;
  obs_factor_ptr->factor = obs_factor;

  // remove very old observations if necessary
  obs_buffer_.ClearOlderThan(0.5);
  ObservationBuffer_T::iterator_T iter_obs_curr =
      obs_buffer_.insert(obs_factor_ptr);
  ObservationBuffer_T::iterator_T iter_obs_end = obs_buffer_.IteratorEnd();

  // go through all the observations and apply them one by one
  for (; iter_obs_curr != iter_obs_end; ++iter_obs_curr) {
    uint64_t curr_obs_time = iter_obs_curr->second->t_ns;
    // get state which is corresponding to this observation
    std::lock_guard<std::mutex> guard(init_navstate_mutex_);
    if (init_navstate_.size() < 2) {
      LC_LDEBUG(INITIALIZATION) << "no enough init state data";
      return LOC_LOCALIZATION_ERROR;
    }
    if ((init_navstate_.front().first > curr_obs_time) ||
        (init_navstate_.back().first < curr_obs_time)) {
      LC_LDEBUG(INITIALIZATION)
          << "no suitable init state data for update, time: " << curr_obs_time
          << " " << init_navstate_.front().first << " "
          << init_navstate_.back().first;
      continue;
    }
    auto r_iter = init_navstate_.rbegin();
    while (r_iter != init_navstate_.rend() && (r_iter->first > curr_obs_time)) {
      ++r_iter;
    }
    // get iter_state_start
    auto iter_state_start = r_iter.base();

    std::pair<uint64_t, NavState> state_before = *r_iter;
    std::pair<uint64_t, NavState> state_after = *(--r_iter);
    // interpolate to obs time
    NavState searched_state;
    double factor = (curr_obs_time * 1e-9 - state_before.first * 1e-9) /
                    (state_after.first * 1e-9 - state_before.first * 1e-9);
    NavStateInterp(state_before.second, state_after.second, factor,
                   &searched_state);

    NavState curr_obs_factor = iter_obs_curr->second->factor;

    // get cov of pos and yaw
    Eigen::Vector3d curr_pos_cov =
        searched_state.pose_cov.topLeftCorner(3, 3).diagonal();
    double curr_yaw_cov = searched_state.pose_cov(5, 5);
    Eigen::Vector3d obs_pos_cov =
        curr_obs_factor.pose_cov.topLeftCorner(3, 3).diagonal();
    double obs_yaw_cov = curr_obs_factor.pose_cov(5, 5);
    // fuse position
    Eigen::Vector3d curr_pos = searched_state.pose.translation();
    Eigen::Vector3d obs_pos = curr_obs_factor.pose.translation();
    Eigen::Vector3d pos_fusion_scale =
        curr_pos_cov.cwiseQuotient(curr_pos_cov + obs_pos_cov);
    Eigen::Vector3d optimal_pos =
        curr_pos + pos_fusion_scale.cwiseProduct(obs_pos - curr_pos);
    Eigen::Vector3d optimal_pos_cov =
        (Eigen::Vector3d(1.0, 1.0, 1.0) - pos_fusion_scale)
            .cwiseProduct(curr_pos_cov);
    // fuse yaw
    Eigen::Quaterniond curr_q(searched_state.pose.so3().matrix());
    Eigen::Quaterniond obs_q(curr_obs_factor.pose.so3().matrix());
    double yaw_fusion_scale = curr_yaw_cov / (curr_yaw_cov + obs_yaw_cov);
    Eigen::Quaterniond optimal_q = curr_q.slerp(yaw_fusion_scale, obs_q);
    double optimal_yaw =
        Utility::R2ypr(optimal_q.normalized().toRotationMatrix())(0);
    double optimal_yaw_cov = (1.0 - yaw_fusion_scale) * curr_yaw_cov;
    // set fusion navstate
    NavState fusion_navstate = curr_obs_factor;
    Eigen::Vector3d curr_ypr =
        Utility::R2ypr(searched_state.pose.so3().matrix());
    // fusion_navstate.pose = SE3d(
    //     SO3d(Eigen::Vector3d(optimal_yaw, curr_ypr(1), curr_ypr(2))),
    //     optimal_pos); old Sophus
    fusion_navstate.pose =
        SE3d(SO3d::exp(Eigen::Vector3d(optimal_yaw, curr_ypr(1), curr_ypr(2))),
             optimal_pos);

    fusion_navstate.pose_cov.setZero();
    fusion_navstate.pose_cov(0, 0) =
        std::max(std::pow(0.2, 2), optimal_pos_cov(0));
    fusion_navstate.pose_cov(1, 1) =
        std::max(std::pow(0.2, 2), optimal_pos_cov(1));
    fusion_navstate.pose_cov(2, 2) =
        std::max(std::pow(0.2, 2), optimal_pos_cov(2));
    fusion_navstate.pose_cov(3, 3) = std::pow(1.0 * d2r, 2);
    fusion_navstate.pose_cov(4, 4) = std::pow(1.0 * d2r, 2);
    fusion_navstate.pose_cov(5, 5) =
        std::max(std::pow(1.0 * d2r, 2), optimal_yaw_cov);

    // make sure to propagate to next observation or up to now if no
    // more observations, propagate from current state
    ObservationBuffer_T::iterator_T iter_obs_next = iter_obs_curr;
    iter_obs_next++;
    // get iter_state_end
    auto iter_state_end = init_navstate_.end();
    if (iter_obs_next != iter_obs_end) {
      uint64_t obs_time = iter_obs_next->second->t_ns;
      // get state that's corresponding to this observation
      if (init_navstate_.size() < 2) {
        LC_LDEBUG(INITIALIZATION) << "no enough init state data";
        return LOC_LOCALIZATION_ERROR;
      }
      if ((init_navstate_.front().first > obs_time) ||
          (init_navstate_.back().first < obs_time)) {
        LC_LDEBUG(INITIALIZATION)
            << "no suitable init state data for update, time: " << obs_time
            << " " << init_navstate_.front().first << " "
            << init_navstate_.back().first;
        continue;
      }
      auto r_iter = init_navstate_.rbegin();
      while (r_iter != init_navstate_.rend() && (r_iter->first > obs_time)) {
        ++r_iter;
      }
      iter_state_end = r_iter.base();
    }

    for (auto iter = iter_state_start; iter != iter_state_end; ++iter) {
      NavState new_state = iter->second;
      new_state.pose = fusion_navstate.pose *
                       (searched_state.pose.inverse() * iter->second.pose);
      new_state.pose_cov = fusion_navstate.pose_cov;
      iter->second = new_state;
    }
  }

  return LOC_SUCCESS;
}

adLocStatus_t Initialization::GetDRByTime(uint64_t timestamp,
                                          OdomState* odom_state) {
  if (nullptr == odom_state) return LOC_NULL_PTR;
  std::lock_guard<std::mutex> guard(raw_dr_mutex_);
  // time check
  if (raw_dr_.size() < 2) {
    LC_LERROR(INITIALIZATION) << "not enough valid pose window";
    return LOC_INVALID;
  }
  if (raw_dr_.front().first > timestamp) {
    LC_LERROR(INITIALIZATION) << "query time delay window too much";
    return LOC_TIME_DELAY;
  }
  // at most 100ms ahead
  static constexpr uint64_t ahead_gap = 1e8;
  if (raw_dr_.back().first + ahead_gap < timestamp) {
    LC_LERROR(INITIALIZATION) << "query time ahead window too much";
    return LOC_TIME_AHEAD;
  }

  if (raw_dr_.back().first >= timestamp) {
    // interpolate
    auto iter = raw_dr_.rbegin();
    while (iter != raw_dr_.rend() && iter->first > timestamp) {
      ++iter;
    }

    OdomState last_state = iter->second;
    if (last_state.timestamp == timestamp) {
      *odom_state = last_state;
      return LOC_SUCCESS;
    }
    OdomState next_state = (--iter)->second;
    if (next_state.timestamp == timestamp) {
      *odom_state = next_state;
      return LOC_SUCCESS;
    }
    double factor = 1.0 * (timestamp - last_state.timestamp) /
                    (next_state.timestamp - last_state.timestamp);
    OdomStateInterp(last_state, next_state, factor, odom_state);
  } else {
    // extrapolate
    static constexpr size_t min_extrapolate_diff = 10;
    if (raw_dr_.size() < min_extrapolate_diff) {
      LC_LERROR(INITIALIZATION)
          << "query time ahead, but not enough pose to extrapolate";
      return LOC_TIME_AHEAD;
    }

    OdomState latest_state, last_state, ahead_state;
    size_t ind = 0;
    auto iter = raw_dr_.rbegin();
    for (; iter != raw_dr_.rend(); ++iter, ++ind) {
      if (0 == ind) {
        latest_state = iter->second;
      }
      if (min_extrapolate_diff - 1 == ind) {
        last_state = iter->second;
        break;
      }
    }
    ahead_state = latest_state;
    ahead_state.timestamp = latest_state.timestamp +
                            (latest_state.timestamp - last_state.timestamp);
    ahead_state.pose =
        latest_state.pose * (last_state.pose.inverse() * latest_state.pose);

    double factor = 1.0 * (timestamp - latest_state.timestamp) /
                    (ahead_state.timestamp - latest_state.timestamp);
    OdomStateInterp(latest_state, ahead_state, factor, odom_state);
  }

  return LOC_SUCCESS;
}

adLocStatus_t Initialization::GetDeltaDR(uint64_t start_time, uint64_t end_time,
                                         SE3d* delta_dr) {
  OdomState start_time_dr;
  if (LOC_SUCCESS != GetDRByTime(start_time, &start_time_dr)) {
    LC_LERROR(INITIALIZATION) << "failed to query dr data, ts " << start_time;
    return LOC_LOCALIZATION_ERROR;
  }
  OdomState end_time_dr;
  if (LOC_SUCCESS != GetDRByTime(end_time, &end_time_dr)) {
    LC_LERROR(INITIALIZATION) << "failed to query dr data, ts " << end_time;
    return LOC_LOCALIZATION_ERROR;
  }

  *delta_dr = start_time_dr.pose.inverse() * end_time_dr.pose;

  return LOC_SUCCESS;
}

Eigen::Vector3d Initialization::LLA2DIS(const Gnss& gnss_start,
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
  Eigen::Vector3d gnss_enu_delta(enu_delta.x, enu_delta.y, enu_delta.z);
  return gnss_enu_delta;
}

void Initialization::NavStateInterp(const NavState& s_ns, const NavState& e_ns,
                                    double factor, NavState* ns) {
  ns->timestamp = s_ns.timestamp + (e_ns.timestamp - s_ns.timestamp) * factor;
  ns->nav_status = e_ns.nav_status;
  ns->origin = e_ns.origin;

  // ns->pose = SE3d::SE3Interpolation(s_ns.pose, e_ns.pose, factor); old Sophus
  ns->pose = Utility::SE3interpolate(s_ns.pose, e_ns.pose, factor);
  ns->linear_speed =
      s_ns.linear_speed + (e_ns.linear_speed - s_ns.linear_speed) * factor;
  ns->linear_acceleration =
      s_ns.linear_acceleration +
      (e_ns.linear_acceleration - s_ns.linear_acceleration) * factor;
  ns->angular_speed =
      s_ns.angular_speed + (e_ns.angular_speed - s_ns.angular_speed) * factor;
  ns->pose_cov = e_ns.pose_cov;
}

void Initialization::OdomStateInterp(const OdomState& s_ns,
                                     const OdomState& e_ns, double factor,
                                     OdomState* ns) {
  ns->timestamp = s_ns.timestamp + (e_ns.timestamp - s_ns.timestamp) * factor;
  ns->odom_status = e_ns.odom_status;
  ns->origin_id = e_ns.origin_id;
  // ns->pose = SE3::SE3Interpolation(s_ns.pose, e_ns.pose, factor); old Sophus
  ns->pose = Utility::SE3interpolate(s_ns.pose, e_ns.pose, factor);
  ns->linear_speed =
      s_ns.linear_speed + (e_ns.linear_speed - s_ns.linear_speed) * factor;
  ns->linear_acceleration =
      s_ns.linear_acceleration +
      (e_ns.linear_acceleration - s_ns.linear_acceleration) * factor;
  ns->angular_speed =
      s_ns.angular_speed + (e_ns.angular_speed - s_ns.angular_speed) * factor;
  ns->pose_cov = e_ns.pose_cov;
}

adLocStatus_t Initialization::IMUInterp(const uint64_t bef_ts,
                                        const Imu& bef_imu,
                                        const uint64_t aft_ts,
                                        const Imu& aft_imu, uint64_t search_ts,
                                        std::shared_ptr<Imu> search_imu) {
  if (bef_ts == aft_ts) {
    LC_LDEBUG(INITIALIZATION) << "IMU interpolate error, bef_ts: " << bef_ts
                              << ", aft_ts: " << aft_ts;
    return LOC_INVALID_PARAM;
  }
  double bef_ts_sec = bef_ts * 1e-9;
  double aft_ts_sec = aft_ts * 1e-9;
  double search_ts_sec = search_ts * 1e-9;
  double time_scale = (search_ts_sec - bef_ts_sec) / (aft_ts_sec - bef_ts_sec);
  Imu inter_imu;
  inter_imu.measurement_time =
      bef_imu.measurement_time + (search_ts_sec - bef_ts_sec);
  inter_imu.linear_acceleration =
      aft_imu.linear_acceleration * time_scale +
      bef_imu.linear_acceleration * (1 - time_scale);
  inter_imu.angular_velocity = aft_imu.angular_velocity * time_scale +
                               bef_imu.angular_velocity * (1 - time_scale);
  *search_imu = inter_imu;
  return LOC_SUCCESS;
}

adLocStatus_t Initialization::IMUExtrap(const uint64_t bef1_ts,
                                        const Imu& bef1_imu,
                                        const uint64_t bef2_ts,
                                        const Imu& bef2_imu,
                                        const uint64_t search_ts,
                                        std::shared_ptr<Imu> search_imu) {
  if (bef1_ts == bef2_ts) {
    LC_LDEBUG(INITIALIZATION) << "IMU extrapolate error, bef1_ts: " << bef1_ts
                              << ", bef2_ts: " << bef2_ts;
    return LOC_INVALID_PARAM;
  }
  double bef1_ts_sec = bef1_ts * 1e-9;
  double bef2_ts_sec = bef2_ts * 1e-9;
  double search_ts_sec = search_ts * 1e-9;
  double time_scale =
      (search_ts_sec - bef2_ts_sec) / (bef2_ts_sec - bef1_ts_sec);
  Imu extra_imu;
  extra_imu.measurement_time =
      bef2_imu.measurement_time + (search_ts_sec - bef2_ts_sec);
  extra_imu.linear_acceleration =
      bef2_imu.linear_acceleration * (1 + time_scale) -
      bef1_imu.linear_acceleration * time_scale;
  extra_imu.angular_velocity = bef2_imu.angular_velocity * (1 + time_scale) -
                               bef1_imu.angular_velocity * time_scale;
  *search_imu = extra_imu;
  return LOC_SUCCESS;
}

}  // namespace localization
}  // namespace senseAD
