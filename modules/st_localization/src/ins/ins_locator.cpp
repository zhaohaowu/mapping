/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * Chen Shengjie <chenshengjie@sensetime.com>
 * Wang Xiaofeng <wangxiaofeng@sensetime.com>
 */

#include "ins/ins_locator.hpp"

#include <fstream>
#include <string>

#include <opencv2/core/eigen.hpp>

#include "common/coordinate_converter.hpp"
#include "common/transform_config.hpp"
#include "localization/common/log.hpp"
#include "localization/data_type/base.hpp"

namespace senseAD {
namespace localization {

adLocStatus_t INSLocator::Init(const LocalizationParam& param) {
  const auto& fusion_param = param.msf_param;
  ins_device_ = param.common_param.ins_device;
  // gravity in current (ENU)
  gravity_ << 0.0, 0.0, 9.81;
  // ins measurement noise
  double var_ins_pos =
      fusion_param.fins_sigma_pos * fusion_param.fins_sigma_pos;  // m
  double var_ins_rot =
      fusion_param.fins_sigma_rot * fusion_param.fins_sigma_rot;  // rad
  Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
  ins_measure_cov_.block<3, 3>(0, 0) = I3 * var_ins_pos;
  ins_measure_cov_.block<3, 3>(3, 3) = I3 * var_ins_rot;

  LC_LINFO(INS) << "Create INSLocator done";
  return LOC_SUCCESS;
}

adLocStatus_t INSLocator::GetState(NavState* nav_state, double* confidence) {
  if (nullptr == nav_state) {
    LC_LERROR(INS) << "nullptr";
    return LOC_NULL_PTR;
  }
  *nav_state = pose_state_;
  // output location confidence
  if (confidence) *confidence = 1.0;
  return LOC_SUCCESS;
}

adLocStatus_t INSLocator::Process(uint64_t timestamp,
                                  std::shared_ptr<Ins> fused_ins,
                                  std::shared_ptr<Imu> raw_imu) {
  if (nullptr == fused_ins) {
    LC_LERROR(INS) << "nullptr";
    return LOC_NULL_PTR;
  }

  if (fused_ins->status == InsStatus::INVALID) {
    LC_LDEBUG_EVERY_SEC(INS, 1) << "Invalid ins for INSLocator.";
    return LOC_LOCALIZATION_ERROR;
  } else if (fused_ins->status == InsStatus::CONVERGING) {
    LC_LDEBUG_EVERY_SEC(INS, 1) << "waiting for ins converging.";
    return LOC_LOCALIZATION_ERROR;
  }

  pose_state_.state_source = INS;
  pose_state_.timestamp = timestamp;

  Eigen::Vector3d ins_attitude;
  ins_attitude << fused_ins->euler_angle.yaw, fused_ins->euler_angle.pitch,
      fused_ins->euler_angle.roll;
  Eigen::Vector3d ypr = ins_attitude;
  PointLLH_t ins_lla;
  ins_lla.lat = fused_ins->position.lat;
  ins_lla.lon = fused_ins->position.lon;
  ins_lla.height = fused_ins->position.height;
  PointENU_t ins_enu;
  CoordinateConverter::GetInstance()->LLA2ENU(ins_lla, &ins_enu);
  Eigen::Vector3d t;
  t << ins_enu.x, ins_enu.y, ins_enu.z;
  pose_state_.pose = ParsePose(ypr, t, ins_device_);

  // ins measurement noise
  pose_state_.pose_cov = ins_measure_cov_;

  Eigen::Vector3d ins_velocity;
  ins_velocity << fused_ins->linear_velocity.x, fused_ins->linear_velocity.y,
      fused_ins->linear_velocity.z;
  // velocity output relative to current vehicle frame
  {
    SO3d R = pose_state_.pose.so3().inverse();
    pose_state_.linear_speed = R.matrix() * ins_velocity;
  }

  // output angular speed and linear acceleration if imu input
  if (raw_imu != nullptr) {
    // output angular speed and linear acceleration not in gt mode from imu
    Eigen::Vector3d imu_angular_velocity, imu_linear_acceleration;
    imu_angular_velocity << raw_imu->angular_velocity.x,
        raw_imu->angular_velocity.y, raw_imu->angular_velocity.z;
    imu_linear_acceleration << raw_imu->linear_acceleration.x,
        raw_imu->linear_acceleration.y, raw_imu->linear_acceleration.z;

    // angular output relative to current vehicle frame
    {
      SO3d R = TransformConfig::GetTvb().so3();
      pose_state_.angular_speed = R.matrix() * imu_angular_velocity;
    }
    // acceleration output relative to current vehicle frame
    {
      SO3d R = pose_state_.pose.so3().inverse();
      SO3d R2 = TransformConfig::GetTvb().so3();
      Eigen::Vector3d vehicle_linear_acc =
          R2.matrix() * imu_linear_acceleration;
      vehicle_linear_acc -= R.matrix() * gravity_;
      pose_state_.linear_acceleration = vehicle_linear_acc;
    }
  }

  return LOC_SUCCESS;
}

SE3d INSLocator::ParsePose(const Eigen::Vector3d& ypr, const Eigen::Vector3d& t,
                           const std::string& ins_device, bool init) {
  // negelect the pitch and yaw angle, as the local z growing with x and y
  Eigen::Vector3d correct_ypr = ypr;
  if (init) correct_ypr(1) = correct_ypr(2) = 0.0;

  SE3d pose;
  // n frame is NED here
  // SO3d Cn_b(correct_ypr); old Sophus
  SO3d Cn_b = SO3d::exp(correct_ypr);
  // position in NED
  Eigen::Vector3d t_ned(t(1), t(0), -t(2));
  if (ins_device == "CHNAV") {
    // EP40 CHNAV INS body frame is vehicle frame(car center, FLU)
    Eigen::Vector3d to_flu;
    to_flu << 0.0, 0.0, M_PI;
    // body frame from FRD to FLU
    // SO3 R = Cn_b * SO3(to_flu); old Sophus
    SO3d R = Cn_b * SO3d::exp(to_flu);
    // set ENU frame reference
    pose = SE3d(R, t_ned);
  } else {
    Eigen::Vector3d to_rfu;
    to_rfu << M_PI_2, 0.0, M_PI;
    // body frame from FRD to RFU
    // SO3d R = Cn_b * SO3d(to_rfu); old Sophus
    SO3d R = Cn_b * SO3d::exp(to_rfu);
    // set vehicle frame centered
    TransformConfig::FromIMUToVehicle(SE3d(R, t_ned), &pose);
  }

  // set ENU frame reference
  TransformConfig::FromRefNEDToENU(pose, &pose, true);
  pose.normalize();

  return pose;
}

}  // namespace localization
}  // namespace senseAD
