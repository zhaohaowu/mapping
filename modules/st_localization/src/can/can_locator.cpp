/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * Wang Xiaofeng <wangxiaofeng@sensetime.com>
 */
#include "can/can_locator.hpp"

#include <list>

#include <opencv2/core/eigen.hpp>

#include "common/transform_config.hpp"
#include "common/utility.hpp"
#include "localization/common/log.hpp"

namespace senseAD {
namespace localization {

adLocStatus_t CANLocator::Init(const LocalizationParam& param) {
  can_minimum_speed_ = param.can_param.can_minimum_speed;
  // use offline calibration or not
  if (param.can_param.use_offline_calib) {
    can_yaw_rate_scale_ = TransformConfig::GetOdomYawRateScale();
    can_yaw_rate_offset_ = TransformConfig::GetOdomYawRateOffset();
    can_velocity_scale_ = TransformConfig::GetOdomVelocityScale();
    // velocity constant offset shouldn't exist while static
    can_velocity_offset_ = 0.0;
  }
  // initial default state
  vehicle_state_ = SE3d();
  core_state_.setZero();
  core_state_(kcStateQuat) = 1.0;
  // create can integrator
  can_integrator_ = std::make_shared<CANIntegrator>();
  if (can_integrator_->Init(param.msf_param) != LOC_SUCCESS) {
    LC_LERROR(LOCALIZATION) << "init can integrator failed";
    return LOC_LOCALIZATION_ERROR;
  }

  LC_LINFO(LOCALIZATION) << "Create CANLocator done";
  return LOC_SUCCESS;
}

adLocStatus_t CANLocator::Process(uint64_t timestamp,
                                  std::shared_ptr<VehicleInfo> can_data) {
  if (nullptr == can_data) {
    LC_LERROR(CAN) << "nullptr";
    return LOC_NULL_PTR;
  }
  if (!initial_state_set_) {
    LC_LERROR_EVERY_SEC(CAN, 1) << "waiting to set initial state";
    return LOC_LOCALIZATION_ERROR;
  }
  if (timestamp <= timestamp_) {
    LC_LERROR(CAN) << "can data disorder: " << timestamp << " " << timestamp_;
    return LOC_TIME_DISORDER;
  }

  timestamp_ = timestamp;
  // compensation of can raw data with offline calibration
  can_data_.angular_yaw_rate =
      (can_data->angular_yaw_rate + can_yaw_rate_offset_) * can_yaw_rate_scale_;
  can_data_.vehicle_speed =
      (can_data->vehicle_speed + can_velocity_offset_) * can_velocity_scale_;

  if (can_data->gear == VehicleInfo::GEAR_REVERSE) {
    can_data_.vehicle_speed = -can_data_.vehicle_speed;
  }

  // acceleration dirctly from can
  curr_linear_acc_ << can_data->acceleration_long, can_data->acceleration_lat,
      can_data->acceleration_vert;

  return CanIntegrator();
}

adLocStatus_t CANLocator::CanIntegrator() {
  // from nsec to sec
  double timestamp = timestamp_ * kNanoSecToSec;
  if (last_time_ < 0.0) {
    dt_ = 0.01;
    last_can_data_ = can_data_;
  } else {
    dt_ = timestamp - last_time_;
    assert(dt_ > 0.0);
    if (dt_ > 0.1) {
      LC_LWARN(LOCALIZATION) << "Large gap between can datas";
    }
  }

  // Get 3d velocity and express it in vehicle frame
  Vector3d v1 = Vector3d(last_can_data_.vehicle_speed, 0.0, 0.0);
  Vector3d w1 = Vector3d(0.0, 0.0, last_can_data_.angular_yaw_rate);
  Vector3d v2 = Vector3d(can_data_.vehicle_speed, 0.0, 0.0);
  Vector3d w2 = Vector3d(0.0, 0.0, can_data_.angular_yaw_rate);

  // compensation of can raw data with online calibration
  const Vector3d& vs = can_integrator_->GetVelocityScale();
  const Vector3d& wb = can_integrator_->GetAngularBias();
  v1 -= vs.asDiagonal() * v1;
  w1 -= wb;
  v2 -= vs.asDiagonal() * v2;
  w2 -= wb;

  // fix heading when stop
  if (fabs(can_data_.vehicle_speed) < can_minimum_speed_) {
    w1.setZero();
    w2.setZero();
  }

  CANMeasurement can_reading_1, can_reading_2;
  can_reading_1 << v1, w1;
  can_reading_2 << v2, w2;
  // nominal state propagation
  // if not high dynamic, following integration method makes no difference
  if (0) {
    can_integrator_->EulerIntegrator(can_reading_2, dt_);
  } else {
    can_integrator_->RK4Integrator(can_reading_1, can_reading_2, dt_);
  }

  core_state_ = can_integrator_->GetCoreState();
  SE3d core_state_se3;
  core_state_se3.translation() = can_integrator_->GetPosition();
  core_state_se3.so3() = SO3d(can_integrator_->GetQuat());
  core_state_se3.normalize();
  if (TransformConfig::FromCANToVehicle(core_state_se3, &vehicle_state_) !=
      LOC_SUCCESS) {
    LC_LERROR(CAN) << "Transform failed";
    return LOC_LOCALIZATION_ERROR;
  }

  // In vehicle frame (using online compensated measurements)
  curr_linear_velocity_ = v2;
  curr_angular_speed_ = w2;

  // udpate last state
  last_time_ = timestamp;
  last_can_data_ = can_data_;

  return LOC_SUCCESS;
}

adLocStatus_t CANLocator::GetState(NavState* nav_state, double* confidence) {
  if (nullptr == nav_state) {
    LC_LERROR(CAN) << "nullptr";
    return LOC_NULL_PTR;
  }

  nav_state->state_source = CAN;
  nav_state->timestamp = timestamp_;
  nav_state->pose = vehicle_state_;
  nav_state->pose.normalize();

  // linear_velocity, angular_velocity, linear_acceleration in 'vehicle frame'
  nav_state->linear_speed = curr_linear_velocity_;
  nav_state->angular_speed = curr_angular_speed_;
  nav_state->linear_acceleration = curr_linear_acc_;

  // Covariance is updated in MSF
  nav_state->pose_cov = can_integrator_->GetP().block<6, 6>(0, 0);
  return LOC_SUCCESS;
}

adLocStatus_t CANLocator::SetState(const NavState& nav_state) {
  vehicle_state_ = nav_state.pose;
  SE3d core_state_se3;
  if (TransformConfig::FromVehicleToCAN(vehicle_state_, &core_state_se3) !=
      LOC_SUCCESS) {
    LC_LERROR(CAN) << "Transform failed";
    return LOC_LOCALIZATION_ERROR;
  }

  core_state_.segment<3>(kcStatePosition) = core_state_se3.translation();
  core_state_.segment<4>(kcStateQuat) =
      Utility::EigenQtoHamiltonVec(core_state_se3.unit_quaternion());
  can_integrator_->SetCoreState(core_state_);

  return LOC_SUCCESS;
}

adLocStatus_t CANLocator::GetAccOutput(Vector3d* linear_acc) {
  while (acc_output_list_.size() > 10) {
    acc_output_list_.pop_front();
  }
  Vector3d dv = curr_linear_velocity_ - last_vehicle_speed_;
  last_vehicle_speed_ = curr_linear_velocity_;
  Vector3d curr_a = Vector3d::Zero();
  // if not static, but delta velocity is small
  // this maybe caused by repeated can velocity
  if (curr_linear_velocity_.norm() > 0.01 && dv.norm() <= 0.01) {
    if (acc_output_list_.empty()) {
      curr_a = Vector3d::Zero();
    } else {
      curr_a = acc_output_list_.back();
    }
  } else {
    curr_a = dv / dt_;
  }

  Vector3d ave_a = Vector3d::Zero();
  double factor = 0.0, total_factor = 0.0;
  for (auto iter = acc_output_list_.begin(); iter != acc_output_list_.end();
       ++iter) {
    factor += 1.0;
    total_factor += factor;
    ave_a += *iter * factor;
  }
  factor += 1.0;
  ave_a += curr_a * factor;
  ave_a /= (total_factor + factor);
  acc_output_list_.push_back(ave_a);

  (*linear_acc) = acc_output_list_.back();

  return LOC_SUCCESS;
}

adLocStatus_t CANLocator::SetInitialState(const SE3d& Tw_v0) {
  // set initial vehicle state
  vehicle_state_ = Tw_v0;
  SE3d Tw_c0;
  if (TransformConfig::FromVehicleToCAN(vehicle_state_, &Tw_c0) !=
      LOC_SUCCESS) {
    LC_LERROR(LOCALIZATION) << "set initial state failed";
    return LOC_LOCALIZATION_ERROR;
  }

  core_state_.setZero();
  core_state_.segment<3>(kcStatePosition) = Tw_c0.translation();
  core_state_.segment<4>(kcStateQuat) =
      Utility::EigenQtoHamiltonVec(Tw_c0.so3().unit_quaternion());
  can_integrator_->SetCoreState(core_state_);

  initial_state_set_ = true;
  return LOC_SUCCESS;
}

}  // namespace localization
}  // namespace senseAD
