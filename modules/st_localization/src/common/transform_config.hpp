/*
 * Copyright (C) 2019-2020 by SenseTime Group Limited. All rights reserved.
 * Wang Xiaofeng<wangxiaofeng@sensetime.com>
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <map>
#include <string>
#include <vector>

#include <Sophus/se3.hpp>
#include <opencv2/opencv.hpp>

#include "common/msf_serialization.hpp"
#include "localization/data_type/base.hpp"
#include "localization/localization_param.hpp"

namespace senseAD {
namespace localization {

using Sophus::SE3;
using Sophus::SO3;

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::Vector4d;

class TransformConfig {
 public:
  struct CamIntrinsic {
    uint32_t cam_width;
    uint32_t cam_height;
    cv::Mat K;
  };

  static adLocStatus_t LoadTransformConfig(const LocalizationParam& param);

  static adLocStatus_t LoadMSFSerialParams(const LocalizationParam& param,
                                           MsfSerialParam* msf_serial_param);

  static adLocStatus_t Transform(const SE3d& pose_from, SE3d* pose_to,
                                 const SE3d& T, bool left_mul);

  static adLocStatus_t FromVehicleToCAN(const SE3d& pose_v, SE3d* pose_c,
                                        bool left_mul = false);

  static adLocStatus_t FromCANToVehicle(const SE3d& pose_c, SE3d* pose_v,
                                        bool left_mul = false);

  static adLocStatus_t FromIMUToVehicle(const SE3d& pose_i, SE3d* pose_v,
                                        bool left_mul = false);

  static adLocStatus_t FromVehicleToIMU(const SE3d& pose_v, SE3d* pose_i,
                                        bool left_mul = false);

  static adLocStatus_t FromBodyToVehicle(const SE3d& pose_b, SE3d* pose_v,
                                         bool left_mul = false);

  static adLocStatus_t FromVehicleToBody(const SE3d& pose_v, SE3d* pose_b,
                                         bool left_mul = false);

  static adLocStatus_t FromRefNEDToENU(const SE3d& pose_ned, SE3d* pose_enu,
                                       bool left_mul = false);

  static adLocStatus_t FromRefENUToNED(const SE3d& pose_enu, SE3d* pose_ned,
                                       bool left_mul = false);

  static void SetTvc(const SE3d& Tvc) {
    Tvc_ = Tvc;
    Tcv_ = Tvc_.inverse();
    Tcv_valid_ = true;
  }

  static SE3d GetTvc() { return Tvc_; }

  static void SetTvb(const SE3d& Tvb) {
    Tvb_ = Tvb;
    Tbv_ = Tvb_.inverse();
    Tvb_valid_ = true;
  }

  static SE3d GetTvb() { return Tvb_; }

  static void SetTvi(const SE3d& Tvi) {
    Tvi_ = Tvi;
    Tiv_ = Tvi_.inverse();
    Tvi_valid_ = true;
  }

  static SE3d GetTvi() { return Tvi_; }

  static void SetCameraIntrinsic(const std::string& camera_name,
                                 const CamIntrinsic& intrinsic);

  static bool GetCameraIntrinsic(const std::string& camera_name,
                                 CamIntrinsic* intrinsic);

  static void SetCameraHomography(const std::string& camera_name,
                                  const cv::Mat& homo);

  static bool GetCameraHomography(const std::string& camera_name,
                                  cv::Mat* homo);

  static void SetTVehCam(const std::string& camera_name, const SE3d& Tvc);

  static bool GetTVehCam(const std::string& camera_name, SE3d* Tvc);

  static bool GetTVehGround(SE3d* T) {
    *T = Tveh_ground_;
    return Tveh_ground_valid_;
  }

  static void SetTVehGround(const SE3d& Tvg) {
    Tveh_ground_ = Tvg;
    Tveh_ground_valid_ = true;
  }

  static void SetTref(const SE3d& Tenu_ned) {
    Tenu_ned_ = Tenu_ned;
    Tned_enu_ = Tenu_ned_.inverse();
    Tenu_ned_valid_ = true;
  }

  static SE3d GetTref() { return Tenu_ned_; }

  static void SetLeverArm(const Vector3d& llarm) { llarm_ = llarm; }

  static Vector3d GetLeverArm() { return llarm_; }

  static void SetAccelBias(const Vector3d& accel_bias) {
    accel_bias_ = accel_bias;
  }

  static Vector3d GetAccelBias() { return accel_bias_; }

  static void SetGyroBias(const Vector3d& gyro_bias) { gyro_bias_ = gyro_bias; }

  static Vector3d GetGyroBias() { return gyro_bias_; }

  static void SetAccelScale(const Vector3d& accel_scale) {
    accel_scale_ = accel_scale;
  }

  static Vector3d GetAccelScale() { return accel_scale_; }

  static void SetGyroScale(const Vector3d& gyro_scale) {
    gyro_scale_ = gyro_scale;
  }

  static Vector3d GetGyroScale() { return gyro_scale_; }

  static void SetOdomVelocityScale(const double& odom_velocity_scale) {
    odom_velocity_scale_ = odom_velocity_scale;
  }

  static void SetOdomVelocityOffset(const double& odom_velocity_offset) {
    odom_velocity_offset_ = odom_velocity_offset;
  }

  static void SetOdomYawRateScale(const double& odom_yaw_rate_scale) {
    odom_yaw_rate_scale_ = odom_yaw_rate_scale;
  }

  static void SetOdomYawRateOffset(const double& odom_yaw_rate_offset) {
    odom_yaw_rate_offset_ = odom_yaw_rate_offset;
  }

  static float64_t GetOdomVelocityScale() { return odom_velocity_scale_; }

  static float64_t GetOdomVelocityOffset() { return odom_velocity_offset_; }

  static float64_t GetOdomYawRateScale() { return odom_yaw_rate_scale_; }

  static float64_t GetOdomYawRateOffset() { return odom_yaw_rate_offset_; }

  static void SetDualantOffset(const double& dual_ant_offset) {
    dual_ant_offset_ = dual_ant_offset;
  }

  static float64_t GetDualantOffset() { return dual_ant_offset_; }

  static void SetVehicleSize(const Vector3d& vehicle_size) {
    vehicle_size_ = vehicle_size;
  }

  static Vector3d GetVehicleSize() { return vehicle_size_; }

 private:
  // body frame (IMU center, RFU) <-> car center (vehicle frame)
  static bool Tvb_valid_;
  // transform from body frame to car center
  static SE3d Tvb_;
  // transform from car center to body frame
  static SE3d Tbv_;

  // imu frame <-> car center (vehicle frame)
  static bool Tvi_valid_;
  // transform from imu frame to car center
  static SE3d Tvi_;
  // transform from car center to imu frame
  static SE3d Tiv_;

  static SE3d Tveh_ground_;
  static bool Tveh_ground_valid_;

  // reference frame NED <-> ENU
  static bool Tenu_ned_valid_;
  // transform from NED to ENU
  static SE3d Tenu_ned_;
  // transform from ENU to NED
  static SE3d Tned_enu_;

  // lever arm (from imu center to left gnss antenna)
  static Vector3d llarm_;

  // car center(vehicle frame) <-> can frame (rear wheel frame)
  static bool Tcv_valid_;
  // transform from car center to can frame
  static SE3d Tcv_;
  // transform from can frame to car center
  static SE3d Tvc_;

  // camera intrinsic
  static std::map<std::string, CamIntrinsic> cam_intrinsics_;

  // camera Homography
  static std::map<std::string, cv::Mat> cam_homos_;

  // transform from camera frame to car center
  static std::map<std::string, SE3d> Tveh_cams_;

  // imu pre-calibrated intrinsic
  static Vector3d accel_bias_;
  static Vector3d gyro_bias_;
  static Vector3d accel_scale_;
  static Vector3d gyro_scale_;

  // odometer pre-calibrated intrinsic
  static float64_t odom_velocity_scale_;
  static float64_t odom_velocity_offset_;
  static float64_t odom_yaw_rate_scale_;
  static float64_t odom_yaw_rate_offset_;

  // dual ant heading offset w.r.t imu forward
  static float64_t dual_ant_offset_;

  // vehicle size (lenght, width, height)
  static Vector3d vehicle_size_;
};

}  // namespace localization
}  // namespace senseAD
