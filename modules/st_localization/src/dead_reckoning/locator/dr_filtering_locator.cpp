/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Du Jiankui <dujiankui@senseauto.com>
 * Fang Yu <fangyu1@senseauto.com>
 */

#include "dead_reckoning/locator/dr_filtering_locator.hpp"

#include <algorithm>

#include "common/msf_serialization.hpp"
#include "common/path_util.hpp"
#include "common/transform_config.hpp"
#include "common/utility.hpp"
#include "eval/evaluator_imu_intrinsic.hpp"
#include "eval/evaluator_odometry.hpp"
#include "imu/imu_static_detect.hpp"

namespace senseAD {
namespace localization {
namespace dr {

adLocStatus_t DRFilteringLocator::Init(const LocalizationParam& param) {
  param_ = param;
  g_n_ = Eigen::Vector3d(0, 0, -gravity_norm);

  // static detector init
  size_t imu_freq = param_.common_param.ins_device == "CHNAV" ? 100 : 125;
  static_detector_.reset(new StaticDetector(imu_freq));
  if (nullptr == static_detector_) {
    LC_LERROR(DR) << "failed to create StaticDetector";
    return LOC_LOCALIZATION_ERROR;
  }

  // evaluator init
  if (param_.ci_param.enable_evaluation) {
    wio_imu_instrincs_.reset(new EvaluatorImuIntrinsic());
    wio_imu_instrincs_->Init(param_.ci_param.results_save_dir,
                             "WIO_IMU_INTRINSIC", param_.ci_param.testcase_id);
  }

  // init data buffer
  {
    std::lock_guard<std::mutex> lock(can_mutex_);
    can_data_list_.set_capacity(kDataSizeUpperLimit);
  }
  imu_deque_.set_capacity(imu_deque_size_);
  vel_list_.set_capacity(kAccVelSize);

  LC_LINFO(DR) << "Create DRFilteringLocator done";
  return LOC_SUCCESS;
}

adLocStatus_t DRFilteringLocator::Process(uint64_t timestamp,
                                          std::shared_ptr<Imu> raw_imu) {
  if (nullptr == raw_imu) {
    return LOC_LOCALIZATION_ERROR;
  }
  if (timestamp < timestamp_) {
    LC_LERROR_EVERY_SEC(DR, 1)
        << "dr process imu time disorder, time: " << timestamp << " "
        << timestamp_;
    return LOC_LOCALIZATION_ERROR;
  }

  // tranfer imu bodyframe from rfu to frd
  timestamp_ = timestamp;
  IMUMeasurement imu_frd = TransferIMURfuToFrd(raw_imu);
  // calculate imu std using moving window
  MovingImuStandardDeviation(imu_frd);

  // kalman filter init step
  if (!is_inited_) {
    if (LOC_SUCCESS != FilterInit(timestamp)) return LOC_LOCALIZATION_ERROR;
    is_inited_ = true;
  }

  // load msf params
  // UpdateMSFParams();

  // kalman filter predict step
  FilterStatePredict(timestamp, imu_frd);

  // kalman filter update
  VehicleStaticDetect(raw_imu);

  if (vehicle_static_) {
    FilterZeroVelocityAngularRateUpdate();
    FilterGyroBiasUpdate();
  } else {
    FilterVelocityUpdate(timestamp);
    FilterGravityAlign();
  }

  // for evaluator
  if (param_.ci_param.enable_evaluation) {
    ImuIntrinsicEvalData imu_data;
    imu_data.acc_bias = GetAccBias();
    imu_data.gyro_bias = GetGyroBias();
    wio_imu_instrincs_->WriteResult(timestamp_ * kNanoSecToSec, imu_data);
  }

  return LOC_SUCCESS;
}

adLocStatus_t DRFilteringLocator::GetState(OdomState* odom_state,
                                           double* confidence) {
  if (nullptr == odom_state) {
    LC_LERROR(DR) << "nullptr";
    return LOC_NULL_PTR;
  }

  odom_state->state_source = WIO;
  odom_state->timestamp = timestamp_;

  // Get PVA
  // from odom frame (FRD, imu center) to vehicle frame (FLU, car center)
  SE3d state_odom_odom(GetQuat(), GetPosition());
  Eigen::Vector3d ypr;
  ypr << 0.0, 0.0, M_PI;
  Eigen::Vector3d t_vb = TransformConfig::GetTvb().translation();
  // SE3 T_v_bfrd = SE3(SO3(ypr), t_vb);
  SE3d T_v_bfrd = SE3d(SO3d::exp(ypr), t_vb);

  SE3d state_vehicle_vehicle = T_v_bfrd * state_odom_odom * T_v_bfrd.inverse();
  odom_state->pose = state_vehicle_vehicle;

  // velocity output relative to current vehicle frame
  {
    SO3d R = T_v_bfrd.so3() * state_odom_odom.so3().inverse();
    odom_state->linear_speed = R.matrix() * GetVelocity();
  }
  // acceleration and angular output relative to current vehicle frame
  {
    SO3d R = TransformConfig::GetTvb().so3();
    Eigen::Vector3d acc_frd =
        acc_corrected_ - GetQuat().toRotationMatrix().transpose() * g_n_;
    Eigen::Vector3d acc_rfu(acc_frd[1], acc_frd[0], -acc_frd[2]);
    odom_state->linear_acceleration = R.matrix() * acc_rfu;

    Eigen::Vector3d gyro_rfu(gyro_corrected_[1], gyro_corrected_[0],
                             -gyro_corrected_[2]);
    odom_state->angular_speed = R.matrix() * gyro_rfu;
  }
  // position(meters) and attitude(rad) covariance
  odom_state->pose_cov.block<3, 3>(0, 0) = p_.block<3, 3>(0, 0);
  odom_state->pose_cov.block<3, 3>(3, 3) = p_.block<3, 3>(6, 6);

  return LOC_SUCCESS;
}

OdomStatus DRFilteringLocator::GetCurrentLocatorStatus() {
  std::lock_guard<std::mutex> lock(error_state_mutex_);
  if (max_error_state_(0) > 0.6 || max_error_state_(1) > 0.6 ||
      max_error_state_(2) > 1.0 || max_error_state_(3) > 2.0 ||
      max_error_state_(4) > 2.0 || max_error_state_(5) > 4.0) {
    LC_LERROR(DR) << "dr max error state: " << max_error_state_(0) << " "
                  << max_error_state_(1) << " " << max_error_state_(2) << " "
                  << max_error_state_(3) << " " << max_error_state_(4) << " "
                  << max_error_state_(5);
    return OdomStatus::FAILED;
  }
  return OdomStatus::GOOD;
}

adLocStatus_t DRFilteringLocator::FilterInit(uint64_t timestamp) {
  // query vehicle info for input timestamp
  can_velocity_scale_ = TransformConfig::GetOdomVelocityScale();

  VehicleInfo vehicle_info;
  if (LOC_SUCCESS != QueryCanDataByTime(timestamp, &vehicle_info)) {
    return LOC_LOCALIZATION_ERROR;
  }

  double init_vehicle_speed =
      0.5 * (vehicle_info.wheel_speed_rl + vehicle_info.wheel_speed_rr);
  init_vehicle_speed *= can_velocity_scale_;

  core_state_.setZero();
  Eigen::Quaterniond q_init(1.0, 0.0, 0.0, 0.0);
  q_init.normalize();
  core_state_.segment<4>(kiStateAttitudeDR) =
      Utility::EigenQtoHamiltonVec(q_init);
  // velocity
  core_state_(kiStateVelocity) = init_vehicle_speed;
  // acc bias
  SetAccBias(TransformConfig::GetAccelBias());
  // gyro bias
  SetGyroBias(TransformConfig::GetGyroBias());

  p_.setZero();
  // PV
  p_.block<3, 3>(kiErrorStatePositionDR, kiErrorStatePositionDR) =
      I33 * 0.01 * 0.01;
  p_.block<3, 3>(kiErrorStateVelocityDR, kiErrorStateVelocityDR) =
      I33 * 0.01 * 0.01;
  // A, increase initial roll/pitch uncertainty
  p_.block<3, 3>(kiErrorStateAttitudeDR, kiErrorStateAttitudeDR) =
      I33 * 0.01 * 0.01;
  p_(kiErrorStateAttitudeDR, kiErrorStateAttitudeDR) = 0.2 * 0.2;
  p_(kiErrorStateAttitudeDR + 1, kiErrorStateAttitudeDR + 1) = 0.2 * 0.2;
  // Ba
  p_.block<3, 3>(kiErrorStateAccBiasDR, kiErrorStateAccBiasDR) =
      I33 * 0.1 * 0.1;
  // Bg
  p_.block<3, 3>(kiErrorStateGyroBiasDR, kiErrorStateGyroBiasDR) =
      I33 * 0.001 * 0.001;

  q_.setZero();
  q_.block<3, 3>(kiNoiseAccMeaDR, kiNoiseAccMeaDR) =
      I33 * param_.msf_param.imu_sigma_acc * param_.msf_param.imu_sigma_acc;
  q_.block<3, 3>(kiNoiseGyroMeaDR, kiNoiseGyroMeaDR) =
      I33 * param_.msf_param.imu_sigma_gyro * param_.msf_param.imu_sigma_gyro;
  q_.block<3, 3>(kiNoiseAccBiasDR, kiNoiseAccBiasDR) =
      I33 * param_.msf_param.imu_sigma_acc_bias *
      param_.msf_param.imu_sigma_acc_bias;
  q_.block<3, 3>(kiNoiseGyroBiasDR, kiNoiseGyroBiasDR) =
      I33 * param_.msf_param.imu_sigma_gyro_bias *
      param_.msf_param.imu_sigma_gyro_bias;

  // init F matrix
  F_.setZero();
  F_.block<3, 3>(kiErrorStatePositionDR, kiErrorStatePositionDR) = O33;
  F_.block<3, 3>(kiErrorStatePositionDR, kiErrorStateVelocityDR) = I33;
  F_.block<3, 3>(kiErrorStatePositionDR, kiErrorStateAttitudeDR) = O33;
  F_.block<3, 3>(kiErrorStateVelocityDR, kiErrorStatePositionDR) = O33;
  F_.block<3, 3>(kiErrorStateVelocityDR, kiErrorStateVelocityDR) = O33;
  F_.block<3, 3>(kiErrorStateAttitudeDR, kiErrorStatePositionDR) = O33;
  F_.block<3, 3>(kiErrorStateAttitudeDR, kiErrorStateVelocityDR) = O33;
  F_.block<3, 3>(kiErrorStateAttitudeDR, kiErrorStateAttitudeDR) = O33;
  // model bias stochastic process as random walk
  static constexpr double CORRETIME = 1.0e10;
  F_.block<3, 3>(kiErrorStateAccBiasDR, kiErrorStateAccBiasDR) =
      -I33 / CORRETIME;
  F_.block<3, 3>(kiErrorStateGyroBiasDR, kiErrorStateGyroBiasDR) =
      -I33 / CORRETIME;

  // init G matrix
  G_.setZero();
  G_.block<3, 3>(kiErrorStateAccBiasDR, kiNoiseAccBiasDR) = I33;
  G_.block<3, 3>(kiErrorStateGyroBiasDR, kiNoiseGyroBiasDR) = I33;

  return LOC_SUCCESS;
}

adLocStatus_t DRFilteringLocator::FilterStatePredict(
    uint64_t timestamp, const IMUMeasurement& imu_frd) {
  // get the time gap
  if (last_imu_time_ <= 0) {
    imu_dt_ = 0.01;
  } else {
    imu_dt_ = timestamp * kNanoSecToSec - last_imu_time_ * kNanoSecToSec;
  }
  if (imu_dt_ == 0.0) {
    return LOC_SUCCESS;
  }
  last_imu_time_ = timestamp;

  // remove bias
  acc_corrected_ = imu_frd.segment<3>(0) - GetAccBias();
  gyro_corrected_ = imu_frd.segment<3>(3) - GetGyroBias();

  // velocity increment (m/s)
  Eigen::Vector3d dv = acc_corrected_ * imu_dt_;
  // angle increment (rad)
  Eigen::Vector3d da = gyro_corrected_ * imu_dt_;

  Eigen::Quaterniond Qnb = GetQuat();
  Qnb_posterior_ = Qnb;
  Eigen::Matrix3d Cnb = Qnb.toRotationMatrix();
  Eigen::Vector3d V_n = GetVelocity();

  // Velocity Update
  Eigen::Vector3d Vn_curr = V_n + Cnb * dv - g_n_ * imu_dt_;
  SetVelocity(0.5 * (V_n + Vn_curr));

  // Attitude Update
  Eigen::Vector3d sigma = da;
  double sigma_norm = sigma.norm();
  double Qbb_r = std::cos(sigma_norm / 2);
  Eigen::Vector3d Qbb_i = (std::fabs(sigma_norm) < 1e-5)
                              ? Vector3d(0, 0, 0)
                              : std::sin(sigma_norm / 2) / sigma_norm * sigma;
  Eigen::Quaterniond Qbb(Qbb_r, Qbb_i(0), Qbb_i(1), Qbb_i(2));
  Qbb.normalize();
  Qnb = Qnb * Qbb;
  Qnb.normalize();
  SetQuat(Qnb);

  // Position Update
  Pn_posterior_ = GetPosition();
  SetPosition(GetPosition() + GetVelocity() * imu_dt_);

  // error state covariance propagation
  FilterCovPropagation(imu_dt_);

  return LOC_SUCCESS;
}

adLocStatus_t DRFilteringLocator::FilterCovPropagation(double dt) {
  // get state
  Eigen::Matrix3d Cnb = GetQuat().toRotationMatrix();

  // update F
  F_.block<3, 3>(kiErrorStateVelocityDR, kiErrorStateAttitudeDR) =
      SO3d::hat(Cnb * acc_corrected_);
  F_.block<3, 3>(kiErrorStateVelocityDR, kiErrorStateAccBiasDR) = Cnb;
  F_.block<3, 3>(kiErrorStateAttitudeDR, kiErrorStateGyroBiasDR) = -Cnb;

  // update G
  G_.block<3, 3>(kiErrorStateVelocityDR, kiNoiseAccMeaDR) = Cnb;
  G_.block<3, 3>(kiErrorStateAttitudeDR, kiNoiseGyroMeaDR) = -Cnb;

  // local used variables
  static FMat Fdt;
  static FMat phi;
  static GMat phi_G;
  static FMat Qk;
  Fdt.noalias() = F_ * dt;
  phi.noalias() = FMat::Identity() + Fdt + 0.5 * Fdt * Fdt;
  phi_G.noalias() = phi * G_;
  Qk.noalias() = phi_G * q_ * phi_G.transpose() * dt;

  // update the a priori covariance matrix
  p_ = phi * p_ * phi.transpose() + Qk;
  p_ = (p_ + p_.transpose()) / 2;

  return LOC_SUCCESS;
}

void DRFilteringLocator::FilterVelocityUpdate(uint64_t timestamp) {
  static uint64_t last_update_ts = 0;

  // control the update fraequency
  double dt = 1.0 / param_.dr_param.dr_velocity_update_freq;
  if ((timestamp * 1e-9 - last_update_ts * 1e-9) < dt) return;
  last_update_ts = timestamp;

  // calculate vehicle speed for this timestamp
  VehicleInfo vehicle_info;
  if (LOC_SUCCESS != QueryCanDataByTime(timestamp, &vehicle_info)) {
    LC_LDEBUG(DR) << "query can data failed, timestamp: " << timestamp;
    return;
  }
  double vehicle_speed =
      0.5 * (vehicle_info.wheel_speed_rl + vehicle_info.wheel_speed_rr);
  vehicle_speed *= can_velocity_scale_;
  Eigen::Vector3d vel_obs(vehicle_speed, 0, 0);
  curr_vehicle_vel_ = vehicle_speed;

  // calculate vehicle speed covariance
  Eigen::Vector3d vel_cov(0, 0, 0);
  Eigen::Vector3d acc_std = imu_std_.segment<3>(0);
  vel_cov[0] = 1e-4;
  // relax NHC if vehicle in intense motion
  vel_cov[1] = acc_std[1] * acc_std[1];
  vel_cov[2] = acc_std[2] * acc_std[2];
  vel_cov[1] = std::max(0.01, std::min(vel_cov[1], 0.1));
  vel_cov[2] = std::max(0.01, std::min(vel_cov[2], 0.1));
  vel_cov *= 1e-4;  // tuned param for adjusting velocity response delay

  /////////////////////// kalman filter velocity update //////////////////////
  Eigen::Vector3d V_n = GetVelocity();
  Eigen::Matrix3d Cnb = GetQuat().toRotationMatrix();
  Eigen::Vector3d Z = Cnb.transpose() * V_n - vel_obs;

  Eigen::Matrix3d R = vel_cov.asDiagonal();

  Eigen::Matrix<double, 3, kiErrorStateSizeDR> H;
  H.setZero();
  // ignore the relationship between velocity error and misalignment angle
  H.block<3, 3>(0, 3) = Cnb.transpose();

  std::vector<int> dim_update(kiErrorStateSizeDR, 0);
  for (int i = 0; i < 3; ++i) dim_update[kiErrorStatePositionDR + i] = 1;
  for (int i = 0; i < 3; ++i) dim_update[kiErrorStateVelocityDR + i] = 1;
  FilterStateUpdate(Z, H, R, dim_update);

  // update position and velocity error state
  UpdateMaxErrorState(timestamp,
                      error_state_.segment<6>(kiErrorStatePositionDR));
}

void DRFilteringLocator::FilterGravityAlign() {
  // get state
  Eigen::Matrix3d Cnb = GetQuat().toRotationMatrix();
  Eigen::Vector3d Vn = GetVelocity();
  Eigen::Vector3d Vb = Cnb.transpose() * Vn;

  double veh_acc_x;
  if (!CalculateVehicleAccX(&veh_acc_x)) return;

  Eigen::Vector3d acc_motion =
      (gyro_corrected_).cross(Vb) + Eigen::Vector3d(veh_acc_x, 0, 0);
  Eigen::Vector3d g_b = acc_corrected_ - acc_motion;
  Eigen::Vector3d Z = g_n_ - Cnb * g_b;

  double g_norm_diff = std::fabs(g_b.norm() - gravity_norm);
  Eigen::Matrix3d R_g_norm = gravity_norm * std::pow(g_norm_diff, 2.0) * I33;
  Eigen::Matrix3d R_na =
      I33 * param_.msf_param.imu_sigma_acc * param_.msf_param.imu_sigma_acc;
  Eigen::Matrix3d R_acc_motion = 3.0 * I33;
  Eigen::Matrix3d R = R_g_norm + R_na + R_acc_motion;
  R = Cnb * R * Cnb.transpose();

  Eigen::Matrix<double, 3, kiErrorStateSizeDR> H;
  H.setZero();
  H.block<3, 3>(0, kiErrorStateAttitudeDR) = -SO3d::hat(Cnb * g_b);
  H.block<3, 3>(0, kiErrorStateAccBiasDR) = -Cnb;
  // TODO(xxx): converge of gyro yaw bias is not guaranteed
  // H.block<3, 3>(0, kiErrorStateGyroBiasDR) = -Cnb * SO3d::hat(Vb);

  // NOTE: not update yaw dimension of atttitude and gyro bias
  std::vector<int> dim_update(kiErrorStateSizeDR, 0);
  for (int i = 0; i < 2; ++i) dim_update[kiErrorStateAttitudeDR + i] = 1;
  for (int i = 0; i < 2; ++i) dim_update[kiErrorStateGyroBiasDR + i] = 1;
  for (int i = 0; i < 3; ++i) dim_update[kiErrorStateAccBiasDR + i] = 1;
  FilterStateUpdate(Z, H, R, dim_update);
}

void DRFilteringLocator::VehicleStaticDetect(std::shared_ptr<Imu> raw_imu) {
  vehicle_static_ = false;
  // zero velocity check
  static_detector_->addIMU(*raw_imu);

  // only detect static while buffer full and speed is very low
  if (!static_detector_->isFull()) return;
  if (GetVelocity().norm() >= 0.2) return;

  if (static_detector_->detector_AMV(
          param_.msf_param.imu_static_acc_var_thre) == LOC_SUCCESS) {
    vehicle_static_ = true;
  } else {
    // if detected static, check at each time else check once a second
    static_detector_->clearIMU();
  }
}

void DRFilteringLocator::FilterZeroVelocityUpdate() {
  static constexpr size_t obs_dim = 3;
  Eigen::VectorXd Z(obs_dim);
  Z = GetVelocity() - Eigen::Vector3d::Zero();  // zero velocity

  Eigen::Matrix<double, obs_dim, obs_dim> R;
  R.setIdentity();
  R *= 1e-20;

  Eigen::Matrix<double, obs_dim, kiErrorStateSizeDR> H;
  H.setZero();
  H.block<3, 3>(0, kiErrorStateVelocityDR).setIdentity();

  std::vector<int> dim_update(kiErrorStateSizeDR, 0);
  for (int i = 0; i < 3; ++i) dim_update[kiErrorStatePositionDR + i] = 1;
  for (int i = 0; i < 3; ++i) dim_update[kiErrorStateVelocityDR + i] = 1;
  FilterStateUpdate(Z, H, R, dim_update);
}

void DRFilteringLocator::FilterGyroBiasUpdate() {
  static constexpr size_t obs_dim = 3;
  Eigen::VectorXd Z(obs_dim);
  Z = imu_mean_.segment<3>(3) - GetGyroBias();  // equals zero angular rate

  Eigen::Matrix<double, obs_dim, obs_dim> R;
  R.setIdentity();
  R(0, 0) = 100.0 * imu_std_[3] * imu_std_[3];
  R(1, 1) = 100.0 * imu_std_[4] * imu_std_[4];
  R(2, 2) = 100.0 * imu_std_[5] * imu_std_[5];

  Eigen::Matrix<double, obs_dim, kiErrorStateSizeDR> H;
  H.setZero();
  H.block<3, 3>(0, kiErrorStateGyroBiasDR).setIdentity();

  // NOTE: not update attitude, as state large jump may occur
  std::vector<int> dim_update(kiErrorStateSizeDR, 0);
  for (int i = 0; i < 3; ++i) dim_update[kiErrorStateGyroBiasDR + i] = 1;
  FilterStateUpdate(Z, H, R, dim_update);
}

void DRFilteringLocator::FilterZeroVelocityAngularRateUpdate() {
  static constexpr size_t obs_dim = 9;
  Eigen::VectorXd Z(obs_dim);
  Z.setZero();
  Z.segment<3>(0) = GetPosition() - Pn_posterior_;  // zero displacement
  Z.segment<3>(3) = GetVelocity() - Eigen::Vector3d::Zero();  // zero velocity
  Eigen::Matrix3d Cnb = GetQuat().toRotationMatrix();
  Eigen::Vector3d ypr_predict = Utility::R2ypr(Cnb);
  Eigen::Vector3d ypr_before_update =
      Utility::R2ypr(Qnb_posterior_.toRotationMatrix());
  Eigen::Vector3d z_atti;  // roll, pitch, yaw
  z_atti(0) = ypr_before_update(2) - ypr_predict(2);
  z_atti(1) = ypr_before_update(1) - ypr_predict(1);
  z_atti(2) = ypr_before_update(0) - ypr_predict(0);
  Z.segment<3>(6) = z_atti;  // zero attitude change

  Eigen::Matrix<double, obs_dim, obs_dim> R;
  R.setIdentity();
  R.block<3, 3>(0, 0) *= 1e-20;  // position
  R.block<3, 3>(3, 3) *= 1e-8;   // zero velocity
  R.block<3, 3>(6, 6) *= 1e-15;  // zero angular rate

  Eigen::Matrix<double, obs_dim, kiErrorStateSizeDR> H;
  H.setZero();
  H.block<3, 3>(0, kiErrorStatePositionDR).setIdentity();
  H.block<3, 3>(3, kiErrorStateVelocityDR).setIdentity();
  double yaw = ypr_predict(0);
  double pitch = ypr_predict(1);
  H(6, 6) = std::cos(yaw) / std::cos(pitch);
  H(6, 7) = std::sin(yaw) / std::cos(pitch);
  H(7, 6) = -std::sin(yaw);
  H(7, 7) = std::cos(yaw);
  H(8, 6) = std::cos(yaw) * std::tan(pitch);
  H(8, 7) = std::sin(yaw) * std::tan(pitch);
  H(8, 8) = 1.0;

  std::vector<int> dim_update(kiErrorStateSizeDR, 0);
  for (int i = 0; i < 3; ++i) dim_update[kiErrorStatePositionDR + i] = 1;
  for (int i = 0; i < 3; ++i) dim_update[kiErrorStateVelocityDR + i] = 1;
  for (int i = 0; i < 3; ++i) dim_update[kiErrorStateAttitudeDR + i] = 1;
  FilterStateUpdate(Z, H, R, dim_update);
}

bool DRFilteringLocator::CalculateVehicleAccX(double* vehicle_acc_x) {
  vel_list_.push_back(curr_vehicle_vel_);
  if (vel_list_.size() < kAccVelSize) return false;

  double mean_vel = 0;
  for (const auto& vx : vel_list_) mean_vel += vx;
  mean_vel /= vel_list_.size();

  static double last_mean_vel = 0;
  static bool has_last = false;
  if (!has_last) {
    has_last = true;
    last_mean_vel = mean_vel;
    vel_list_.clear();
    return false;
  }

  *vehicle_acc_x = (mean_vel - last_mean_vel) / (kAccVelSize * imu_dt_);
  last_mean_vel = mean_vel;
  vel_list_.clear();
  return true;
}

void DRFilteringLocator::FilterStateUpdate(
    const Eigen::VectorXd& Z, const Eigen::MatrixXd& H,
    const Eigen::MatrixXd& R, const std::vector<int>& state_dim_update) {
  // full kalman gain
  Eigen::MatrixXd K_full(kiErrorStateSizeDR, Z.rows());
  Eigen::MatrixXd p_HT = p_ * H.transpose();
  K_full.noalias() = p_HT * (H * p_HT + R).inverse();

  // take kalman gain for update states
  Eigen::MatrixXd K(K_full.rows(), K_full.cols());
  K.setZero();
  for (int i = 0; i < kiErrorStateSizeDR; ++i) {
    if (state_dim_update[i]) K.row(i) = K_full.row(i);
  }

  // update error state
  error_state_.noalias() = K * Z;

  // update the a posteriori covariance matrix
  static IMUPMatDR I_KH;
  I_KH.noalias() = IMUPMatDR::Identity() - K * H;
  p_ = I_KH * p_ * I_KH.transpose() + K * R * K.transpose();
  p_ = (p_ + p_.transpose()) / 2.0;

  // error state feed back
  if (state_dim_update[kiErrorStatePositionDR]) {
    Eigen::Vector3d P_n = GetPosition();
    P_n -= error_state_.segment<3>(kiErrorStatePositionDR);
    SetPosition(P_n);
  }
  if (state_dim_update[kiErrorStateVelocityDR]) {
    Eigen::Vector3d V_n = GetVelocity();
    V_n -= error_state_.segment<3>(kiErrorStateVelocityDR);
    SetVelocity(V_n);
  }
  if (state_dim_update[kiErrorStateAttitudeDR]) {
    Eigen::Vector3d epi = error_state_.segment<3>(kiErrorStateAttitudeDR);
    Eigen::Matrix3d Cnb = GetQuat().toRotationMatrix();
    Cnb = (Eigen::Matrix3d::Identity() + SO3d::hat(epi)) * Cnb;
    Eigen::Quaterniond Qnb = Eigen::Quaterniond(Cnb).normalized();
    SetQuat(Qnb);
  }
  if (state_dim_update[kiErrorStateAccBiasDR]) {
    Eigen::Vector3d acc_b = GetAccBias();
    acc_b += error_state_.segment<3>(kiErrorStateAccBiasDR);
    SetAccBias(acc_b);
  }
  if (state_dim_update[kiErrorStateGyroBiasDR]) {
    Eigen::Vector3d gyro_b = GetGyroBias();
    gyro_b += error_state_.segment<3>(kiErrorStateGyroBiasDR);
    SetGyroBias(gyro_b);
  }
}

IMUMeasurement DRFilteringLocator::TransferIMURfuToFrd(
    std::shared_ptr<Imu> raw_imu) {
  IMUMeasurement imu_frd;
  imu_frd(0) = raw_imu->linear_acceleration.y;
  imu_frd(1) = raw_imu->linear_acceleration.x;
  imu_frd(2) = -raw_imu->linear_acceleration.z;
  imu_frd(3) = raw_imu->angular_velocity.y;
  imu_frd(4) = raw_imu->angular_velocity.x;
  imu_frd(5) = -raw_imu->angular_velocity.z;
  return imu_frd;
}

adLocStatus_t DRFilteringLocator::QueryCanDataByTime(
    uint64_t timestamp, VehicleInfo* vehicle_info) const {
  if (nullptr == vehicle_info) return LOC_NULL_PTR;
  std::lock_guard<std::mutex> lock(can_mutex_);
  // time check
  if (can_data_list_.size() < 2) {
    LC_LDEBUG(DR) << "not enough valid can data window";
    return LOC_INVALID;
  }
  if (can_data_list_.front().first > timestamp) {
    LC_LDEBUG(DR) << "query time delay window too much";
    return LOC_TIME_DELAY;
  }
  // at most 1s ahead
  static constexpr uint64_t ahead_gap = 1e9;
  if (can_data_list_.back().first + ahead_gap < timestamp) {
    LC_LDEBUG(DR) << "query time ahead window too much";
    return LOC_TIME_AHEAD;
  }

  if (can_data_list_.back().first >= timestamp) {
    // interpolate
    auto iter = can_data_list_.rbegin();
    while (iter != can_data_list_.rend() && iter->first > timestamp) {
      ++iter;
    }

    auto last_info = *iter;
    if (last_info.first == timestamp) {
      *vehicle_info = last_info.second;
      return LOC_SUCCESS;
    }
    auto next_info = *(--iter);
    if (next_info.first == timestamp) {
      *vehicle_info = next_info.second;
      return LOC_SUCCESS;
    }
    double factor = 1.0 * (timestamp - last_info.first) /
                    (next_info.first - last_info.first);
    VehicleInfoInterp(last_info.second, next_info.second, factor, vehicle_info);
  } else {
    // extrapolate
    auto& latest_info = can_data_list_.front();
    auto& ahead_info = can_data_list_.back();

    double factor = 1.0 * (timestamp - latest_info.first) /
                    (ahead_info.first - latest_info.first);
    VehicleInfoInterp(latest_info.second, ahead_info.second, factor,
                      vehicle_info);
  }

  return LOC_SUCCESS;
}

void DRFilteringLocator::VehicleInfoInterp(const VehicleInfo& s_info,
                                           const VehicleInfo& e_info,
                                           double factor,
                                           VehicleInfo* info) const {
  // yaw rate interpolate
  info->angular_yaw_rate =
      s_info.angular_yaw_rate +
      (e_info.angular_yaw_rate - s_info.angular_yaw_rate) * factor;
  // vehcile speed interpolate
  info->vehicle_speed = s_info.vehicle_speed +
                        (e_info.vehicle_speed - s_info.vehicle_speed) * factor;
  // wheel speed interpolate
  info->wheel_speed_fl =
      s_info.wheel_speed_fl +
      (e_info.wheel_speed_fl - s_info.wheel_speed_fl) * factor;
  info->wheel_speed_fr =
      s_info.wheel_speed_fr +
      (e_info.wheel_speed_fr - s_info.wheel_speed_fr) * factor;
  info->wheel_speed_rl =
      s_info.wheel_speed_rl +
      (e_info.wheel_speed_rl - s_info.wheel_speed_rl) * factor;
  info->wheel_speed_rr =
      s_info.wheel_speed_rr +
      (e_info.wheel_speed_rr - s_info.wheel_speed_rr) * factor;
}

adLocStatus_t DRFilteringLocator::MovingImuStandardDeviation(
    const IMUMeasurement& imu_frd) {
  imu_deque_.push_back(imu_frd);
  if (imu_deque_.size() < imu_deque_size_) {
    imu_mean_ = imu_frd;
    imu_std_ = imu_frd;
  } else if (imu_deque_.size() == imu_deque_size_) {
    // first mean and square sum
    imu_mean_.setZero();
    imu_square_sum_.setZero();
    for (const IMUMeasurement& imu : imu_deque_) {
      imu_mean_ += imu;
      imu_square_sum_ += imu.cwiseProduct(imu);
    }
    imu_mean_ /= imu_deque_size_;

    // first std
    imu_std_.setZero();
    for (const IMUMeasurement& imu : imu_deque_) {
      imu_std_ += (imu - imu_mean_).cwiseProduct(imu - imu_mean_);
    }
    imu_std_ /= imu_deque_size_;
    imu_std_ = imu_std_.cwiseSqrt();
  } else {
    const IMUMeasurement& imu_poped = imu_deque_.front();
    const IMUMeasurement& imu_added = imu_deque_.back();

    // update mean
    imu_mean_ =
        (imu_mean_ * imu_deque_size_ - imu_poped + imu_added) / imu_deque_size_;

    // update square sum
    imu_square_sum_ = imu_square_sum_ - imu_poped.cwiseProduct(imu_poped) +
                      imu_added.cwiseProduct(imu_added);

    // update std
    imu_std_ =
        imu_square_sum_ / imu_deque_size_ - imu_mean_.cwiseProduct(imu_mean_);
    imu_std_ = imu_std_.cwiseSqrt();
  }
  return LOC_SUCCESS;
}

void DRFilteringLocator::UpdateMaxErrorState(uint64_t timestamp,
                                             const Vector6d& error_state) {
  std::lock_guard<std::mutex> lock(error_state_mutex_);
  error_state_buffer_.emplace_back(timestamp, error_state);
  while (error_state_buffer_.back().first >
         error_state_buffer_.front().first + 2e9) {  // buffer 2s data
    error_state_buffer_.pop_front();
  }

  Vector6d max_err_state;
  max_err_state.setZero();
  for (const auto& item : error_state_buffer_) {
    Vector6d err = item.second;
    for (int i = 0; i <= 5; ++i) {  // x, y, z, vel_x, vel_y, vel_z
      max_err_state(i) = std::max(std::fabs(err(i)), max_err_state(i));
    }
  }
  max_error_state_ = max_err_state;
}

void DRFilteringLocator::UpdateMSFParams() {
  static uint64_t last_update_ts = 0;
  // control the update fraequency
  double dt = 60.0;  // 1 min
  if ((timestamp_ * 1e-9 - last_update_ts * 1e-9) < dt) return;
  last_update_ts = timestamp_;

  // load msf serial param
  MsfSerialParam msf_serial_param;
  if (LOC_SUCCESS !=
      TransformConfig::LoadMSFSerialParams(param_, &msf_serial_param)) {
    LC_LDEBUG(DR) << "Using estimated MSF param for DR";
    return;
  }

  // TODO(xxx): check param quality

  // write param into state
  // imu related
  Eigen::Vector3d acc_bias(msf_serial_param.acc_bias_x,
                           msf_serial_param.acc_bias_y,
                           msf_serial_param.acc_bias_z);
  Eigen::Vector3d gyro_bias(msf_serial_param.gyro_bias_x,
                            msf_serial_param.gyro_bias_y,
                            msf_serial_param.gyro_bias_z);
  // TODO(xxx): how about scale?
  // TODO(xxx): how about cov?
  SetAccBias(acc_bias);
  SetGyroBias(gyro_bias);

  // can related
  can_velocity_scale_ = msf_serial_param.can_velocity_scale;

  LC_LDEBUG(DR) << "load msf serial param from file path: "
                << param_.msf_param.msf_serial_save_path;
  LC_LDEBUG(DR) << "load imu intrinsic param: Ba " << acc_bias.transpose()
                << ", Bg " << gyro_bias.transpose();
  LC_LDEBUG(DR) << "load can velocity scale param: " << can_velocity_scale_;
}

}  // namespace dr
}  // namespace localization
}  // namespace senseAD
