/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * Wang Xiaofeng<wangxiaofeng@sensetime.com>
 */
#include "common/transform_config.hpp"

#include <Sophus/se3.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include "ad_common/config_utils/config_utils.hpp"
#include "ad_common/config_utils/macros.hpp"
#include "ad_scm/ad_scm.hpp"
#include "ad_scm/ad_transform.hpp"
#include "ad_time/ad_time.hpp"
#include "common/path_util.hpp"
#include "common/utility.hpp"
#include "localization/common/log.hpp"
#include "localization/data_type/base.hpp"

namespace senseAD {
namespace localization {

using typename senseAD::common::utils::CoordinateTransformUtility;
using typename senseAD::sensorconfig::SensorConfigManager;

bool TransformConfig::Tvb_valid_ = false;
SE3d TransformConfig::Tvb_;
SE3d TransformConfig::Tbv_;

bool TransformConfig::Tvi_valid_ = false;
SE3d TransformConfig::Tvi_;
SE3d TransformConfig::Tiv_;

bool TransformConfig::Tcv_valid_ = false;
SE3d TransformConfig::Tcv_;
SE3d TransformConfig::Tvc_;

bool TransformConfig::Tveh_ground_valid_ = false;
SE3d TransformConfig::Tveh_ground_;

bool TransformConfig::Tenu_ned_valid_ = false;
SE3d TransformConfig::Tenu_ned_;
SE3d TransformConfig::Tned_enu_;

std::map<std::string, TransformConfig::CamIntrinsic>
    TransformConfig::cam_intrinsics_;

std::map<std::string, cv::Mat> TransformConfig::cam_homos_;

std::map<std::string, SE3d> TransformConfig::Tveh_cams_;

Vector3d TransformConfig::llarm_;
Vector3d TransformConfig::accel_bias_ = Vector3d::Zero();
Vector3d TransformConfig::gyro_bias_ = Vector3d::Zero();
Vector3d TransformConfig::accel_scale_ = Vector3d::Zero();
Vector3d TransformConfig::gyro_scale_ = Vector3d::Zero();

float64_t TransformConfig::odom_velocity_scale_ = 1.0;
float64_t TransformConfig::odom_velocity_offset_ = 0.0;
float64_t TransformConfig::odom_yaw_rate_scale_ = 1.0;
float64_t TransformConfig::odom_yaw_rate_offset_ = 0.0;
float64_t TransformConfig::dual_ant_offset_ = 0.0;

Vector3d TransformConfig::vehicle_size_ = Vector3d::Ones();

adLocStatus_t TransformConfig::LoadTransformConfig(
    const LocalizationParam& param) {
  CoordinateTransformUtility gTfUtil =
      CoordinateTransformUtility::get_shared_instance();
  if (gTfUtil.init_with_configuration() != senseAD::SCM_SUCCESS) {
    LC_LERROR(COMMON) << "Init SCM failed";
    return LOC_INIT_ERROR;
  }

  ////////////////// Load necessary intrinsic and extrinsic //////////////////

  // load msf serial param
  MsfSerialParam msf_serial_param;
  bool use_msf_serial_param =
      LOC_SUCCESS == LoadMSFSerialParams(param, &msf_serial_param);
  if (!use_msf_serial_param) {
    LC_LINFO(COMMON) << "Using offline calibration param";
  }

  // imu intrinsic
  senseAD::sensorconfig::IMUCalib imu_calib;
  if (senseAD::SCM_SUCCESS !=
      SensorConfigManager::GetInstance()->GetIMUCalibConfig(&imu_calib)) {
    LC_LERROR(COMMON) << "Failed to load imu intrinsic";
    return LOC_LOCALIZATION_ERROR;
  }
  Vector3d acc_bias, gyro_bias, acc_scale, gyro_scale;
  if (use_msf_serial_param) {
    acc_bias =
        Vector3d(msf_serial_param.acc_bias_x, msf_serial_param.acc_bias_y,
                 msf_serial_param.acc_bias_z);
    gyro_bias =
        Vector3d(msf_serial_param.gyro_bias_x, msf_serial_param.gyro_bias_y,
                 msf_serial_param.gyro_bias_z);
    acc_scale =
        Vector3d(msf_serial_param.acc_scale_x, msf_serial_param.acc_scale_y,
                 msf_serial_param.acc_scale_z);
    gyro_scale =
        Vector3d(msf_serial_param.gyro_scale_x, msf_serial_param.gyro_scale_y,
                 msf_serial_param.gyro_scale_z);
  } else {
    acc_bias = Vector3d(imu_calib.accel_bias[0], imu_calib.accel_bias[1],
                        imu_calib.accel_bias[2]);
    gyro_bias = Vector3d(imu_calib.gyro_bias[0], imu_calib.gyro_bias[1],
                         imu_calib.gyro_bias[2]);
    acc_scale = Vector3d(imu_calib.accel_scale[0], imu_calib.accel_scale[1],
                         imu_calib.accel_scale[2]);
    gyro_scale = Vector3d(imu_calib.gyro_scale[0], imu_calib.gyro_scale[1],
                          imu_calib.gyro_scale[2]);
  }
  LC_LINFO(COMMON) << "Load imu intrinsic param: " << acc_bias.transpose()
                   << " " << gyro_bias.transpose() << " "
                   << acc_scale.transpose() << " " << gyro_scale.transpose();
  TransformConfig::SetAccelBias(acc_bias);
  TransformConfig::SetGyroBias(gyro_bias);
  TransformConfig::SetAccelScale(acc_scale);
  TransformConfig::SetGyroScale(gyro_scale);

  // odometer linear speed factor
  senseAD::sensorconfig::CanCalib can_calib;
  if (senseAD::SCM_SUCCESS !=
      SensorConfigManager::GetInstance()->GetCanCalibConfig(&can_calib)) {
    LC_LERROR(COMMON) << "Failed to read odometer linear speed factor";
    return LOC_LOCALIZATION_ERROR;
  }
  double can_velocity_scale = use_msf_serial_param
                                  ? msf_serial_param.can_velocity_scale
                                  : can_calib.can_velocity_scale;
  LC_LINFO(COMMON) << "Load can velocity scale param: " << can_velocity_scale;
  TransformConfig::SetOdomVelocityScale(can_velocity_scale);
  TransformConfig::SetOdomVelocityOffset(can_calib.can_velocity_offset);
  TransformConfig::SetOdomYawRateScale(can_calib.can_yaw_rate_scale);
  TransformConfig::SetOdomYawRateOffset(can_calib.can_yaw_rate_offset);

  // extrinsic between can frame (rear wheel) and vehicle frame (car center)
  senseAD::Transform cvTvc;
  if (!gTfUtil.get_transform("can", "car_center", cvTvc)) {
    LC_LERROR(COMMON) << "Failed to read can to car center extrinsic";
    return LOC_LOCALIZATION_ERROR;
  }
  Matrix4d Tvc;
  cv::cv2eigen(cvTvc, Tvc);
  TransformConfig::SetTvc(
      SE3d(Tvc.topLeftCorner(3, 3), Tvc.topRightCorner(3, 1)));

  // extrinsic between imu frame and vehicle frame
  senseAD::Transform cvTvi;
  if (!gTfUtil.get_transform("gnss", "car_center", cvTvi)) {
    LC_LERROR(COMMON) << "Failed to read gnss to car center extrinsic";
    return LOC_LOCALIZATION_ERROR;
  }
  Eigen::Matrix4d Tvi;
  cv::cv2eigen(cvTvi, Tvi);
  TransformConfig::SetTvi(SE3d(Tvi.block<3, 3>(0, 0), Tvi.block<3, 1>(0, 3)));

  // extrinsic between body frame (IMU center, RFU) and vehicle frame (FLU)
  Eigen::Vector3d ypr_vb;
  ypr_vb << -M_PI_2, 0.0, 0.0;
  // TransformConfig::SetTvb(SE3(SO3(ypr_vb), Tvi.block<3, 1>(0, 3)));
  TransformConfig::SetTvb(SE3d(SO3d::exp(ypr_vb), Tvi.block<3, 1>(0, 3)));

  // Transform from NED to ENU
  Eigen::Vector3d ypr;
  ypr << M_PI_2, 0.0, M_PI;
  // SO3 Renu_ned(ypr);
  SO3d Renu_ned = SO3d::exp(ypr);
  SE3d Tenu_ned = SE3d(Renu_ned, Eigen::Vector3d::Zero());
  TransformConfig::SetTref(Tenu_ned);

  // vehicle parameters
  senseAD::VehicleConfig vehicle_param;
  Vector3d vehicle_size;
  if (senseAD::SCM_SUCCESS ==
      SensorConfigManager::GetInstance()->GetVehicleConfigInfo(
          "vehicle", &vehicle_param)) {
    vehicle_size = Vector3d(vehicle_param.car_length, vehicle_param.car_width,
                            vehicle_param.car_height);
  } else {
    LC_LERROR(COMMON)
        << "Failed to load vehicle parameters, set default values";
    vehicle_size = Vector3d(4.9, 1.9, 1.5);
  }
  TransformConfig::SetVehicleSize(vehicle_size);

  //////////////// Load config just for absolute localization ////////////////
  if (!param.common_param.absolute_localization) return LOC_SUCCESS;

  // dual ant heading offset w.r.t imu forward
  senseAD::sensorconfig::DualantCalib dual_ant_calib;
  if (senseAD::SCM_SUCCESS !=
      SensorConfigManager::GetInstance()->GetDUALANTCalibConfig(
          &dual_ant_calib)) {
    LC_LERROR(COMMON) << "Failed to load dual ant extrinsic";
    return LOC_LOCALIZATION_ERROR;
  }
  TransformConfig::SetDualantOffset(dual_ant_calib.heading_offset);

  // extrinsic between GNSS antenna and IMU(RFU)
  senseAD::Transform cvTig;
  if (!gTfUtil.get_transform("left_antenna", "gnss", cvTig)) {
    LC_LERROR(COMMON) << "Failed to read left antenna to gnss extrinsic";
    return LOC_LOCALIZATION_ERROR;
  }
  Eigen::Matrix4d Tig;
  cv::cv2eigen(cvTig, Tig);
  // express it in IMU(FRD) frame
  Vector3d llarm = Vector3d(Tig(1, 3), Tig(0, 3), -Tig(2, 3));
  TransformConfig::SetLeverArm(llarm);

  std::vector<bool> back_locator_type_ =
      FindLocatorTypeFromString(param.common_param.back_locator_type);
  // necessary for SMM
  if (back_locator_type_[SMM]) {
    // extrinsic between car_center and ground
    senseAD::Transform cvT_ground_veh;
    if (!gTfUtil.get_transform("car_center", "ground_projection",
                               cvT_ground_veh)) {
      LC_LERROR(COMMON)
          << "Failed to read car_center to ground_projection extrinsic";
      return LOC_LOCALIZATION_ERROR;
    }
    Eigen::Matrix4d T_ground_veh;
    cv::cv2eigen(cvT_ground_veh, T_ground_veh);
    TransformConfig::SetTVehGround(
        SE3d(T_ground_veh.block<3, 3>(0, 0), T_ground_veh.block<3, 1>(0, 3))
            .inverse());
  }

  // necessary for SMM
  if (back_locator_type_[SMM]) {
    const auto& camera_names = param.smm_param.enable_camera_names;

    // intrinsic for camera
    for (const auto& camera_name : camera_names) {
      senseAD::sensorconfig::StructCamInternalCalib cam_calib;
      if (senseAD::SCM_SUCCESS !=
          SensorConfigManager::GetInstance()->GetCameraInternalCalibConfig(
              camera_name, &cam_calib)) {
        LC_LERROR(COMMON) << "Failed to load camera intrinsic";
        return LOC_LOCALIZATION_ERROR;
      }
      TransformConfig::CamIntrinsic intrinsic;
      if (!cam_calib.cam_K_new.empty()) {
        intrinsic.cam_width = cam_calib.img_new_w;
        intrinsic.cam_height = cam_calib.img_new_h;
        intrinsic.K = cam_calib.cam_K_new;
      } else {
        intrinsic.cam_width = cam_calib.img_dist_w;
        intrinsic.cam_height = cam_calib.img_dist_h;
        intrinsic.K = cam_calib.cam_K;
      }
      TransformConfig::SetCameraIntrinsic(camera_name, intrinsic);
    }

    // homography for camera
    for (const auto& camera_name : camera_names) {
      senseAD::sensorconfig::HMatrix cam_h_matrix;
      senseAD::sensorconfig::CamHomography camera_homo;
      if (SensorConfigManager::GetInstance()->GetHMatrixConfig(
              camera_name, &cam_h_matrix) == senseAD::SCM_SUCCESS) {
        TransformConfig::SetCameraHomography(camera_name,
                                             cam_h_matrix.h_matrix);
      } else {
        LC_LERROR(COMMON) << "Failed to load camera homography";
        return LOC_LOCALIZATION_ERROR;
      }
    }

    // extrinsic between camera and car_center
    for (const auto& camera_name : camera_names) {
      senseAD::Transform cvTvc;
      if (!gTfUtil.get_transform(camera_name, "car_center", cvTvc)) {
        LC_LERROR(COMMON) << "Failed to read " << camera_name
                          << " to car center extrinsic";
        return LOC_LOCALIZATION_ERROR;
      }

      Eigen::Matrix4d Tvc;
      cv::cv2eigen(cvTvc, Tvc);
      TransformConfig::SetTVehCam(
          camera_name, SE3d(Tvc.block<3, 3>(0, 0), Tvc.block<3, 1>(0, 3)));
    }
  }

  return LOC_SUCCESS;
}

adLocStatus_t TransformConfig::LoadMSFSerialParams(
    const LocalizationParam& param, MsfSerialParam* msf_serial_param) {
  if (!param.msf_param.enable_msf_serial) {
    LC_LINFO(COMMON) << "Msf serial disable";
    return LOC_LOCALIZATION_ERROR;
  }

  std::string msf_serial_path = param.msf_param.msf_serial_save_path;
  using Reader = senseAD::common::utils::ConfigurationReader;
  if (FileSystem::IsFile(msf_serial_path) &&
      senseAD::CONF_SUCCESS ==
          Reader::LoadJSON(msf_serial_path, msf_serial_param)) {
    double param_ts = msf_serial_param->timestamp * 1e-9;
    double now_ts = senseAD::time::Now().ToSec();
    if (now_ts - param_ts < 5e6) {  // about 2 mouths
      LC_LINFO(COMMON) << "Read msf serial param successfully, path: "
                       << msf_serial_path;
      return LOC_SUCCESS;
    } else {
      LC_LERROR(COMMON) << "Msf serial file is too old, not used";
      return LOC_INVALID_PARAM;
    }
  } else {
    LC_LERROR(COMMON) << "Failed to find msf serial file, load failed";
    return LOC_FILE_NOT_FOUND;
  }
  return LOC_SUCCESS;
}

adLocStatus_t TransformConfig::Transform(const SE3d& pose_from, SE3d* pose_to,
                                         const SE3d& T, bool left_mul) {
  if (pose_to == nullptr) {
    LC_LERROR(COMMON) << "null input";
    return LOC_LOCALIZATION_ERROR;
  }

  if (left_mul) {
    (*pose_to) = T * pose_from;
  } else {
    (*pose_to) = pose_from * T;
  }

  pose_to->normalize();
  return LOC_SUCCESS;
}

adLocStatus_t TransformConfig::FromVehicleToCAN(const SE3d& pose_v,
                                                SE3d* pose_c, bool left_mul) {
  if (!Tcv_valid_) {
    LC_LERROR(COMMON) << "invalid extrinsic Tcv";
    return LOC_LOCALIZATION_ERROR;
  }

  if (left_mul) {
    // Tcw = Tcv * Tvw
    return Transform(pose_v, pose_c, Tcv_, true);
  } else {
    // Twc = Twv * Tvc
    return Transform(pose_v, pose_c, Tvc_, false);
  }

  return LOC_LOCALIZATION_ERROR;
}

adLocStatus_t TransformConfig::FromCANToVehicle(const SE3d& pose_c,
                                                SE3d* pose_v, bool left_mul) {
  if (!Tcv_valid_) {
    LC_LERROR(COMMON) << "invalid extrinsic Tcv";
    return LOC_LOCALIZATION_ERROR;
  }

  if (left_mul) {
    // Tvw = Tvc * Tcw
    return Transform(pose_c, pose_v, Tvc_, true);
  } else {
    // Twv = Twc * Tcv
    return Transform(pose_c, pose_v, Tcv_, false);
  }

  return LOC_LOCALIZATION_ERROR;
}

adLocStatus_t TransformConfig::FromIMUToVehicle(const SE3d& pose_i,
                                                SE3d* pose_v, bool left_mul) {
  if (!Tvi_valid_) {
    LC_LERROR(COMMON) << "invalid extrinsic Tvi";
    return LOC_LOCALIZATION_ERROR;
  }

  if (left_mul) {
    // Tvw = Tvi * Tiw;
    return Transform(pose_i, pose_v, Tvi_, true);
  } else {
    // Twv = Twi * Tiv;
    return Transform(pose_i, pose_v, Tiv_, false);
  }

  return LOC_LOCALIZATION_ERROR;
}

adLocStatus_t TransformConfig::FromVehicleToIMU(const SE3d& pose_v,
                                                SE3d* pose_i, bool left_mul) {
  if (!Tvi_valid_) {
    LC_LERROR(COMMON) << "invalid extrinsic Tvi";
    return LOC_LOCALIZATION_ERROR;
  }

  if (left_mul) {
    // Tiw = Tiv * Tvw
    return Transform(pose_v, pose_i, Tiv_, true);
  } else {
    // Twi = Twv * Tvi
    return Transform(pose_v, pose_i, Tvi_, false);
  }

  return LOC_LOCALIZATION_ERROR;
}

adLocStatus_t TransformConfig::FromBodyToVehicle(const SE3d& pose_b,
                                                 SE3d* pose_v, bool left_mul) {
  if (!Tvb_valid_) {
    LC_LERROR(COMMON) << "invalid extrinsic Tvb";
    return LOC_LOCALIZATION_ERROR;
  }

  if (left_mul) {
    // Tvw = Tvb * Tbw;
    return Transform(pose_b, pose_v, Tvb_, true);
  } else {
    // Twv = Twb * Tbv;
    return Transform(pose_b, pose_v, Tbv_, false);
  }

  return LOC_LOCALIZATION_ERROR;
}

adLocStatus_t TransformConfig::FromVehicleToBody(const SE3d& pose_v,
                                                 SE3d* pose_b, bool left_mul) {
  if (!Tvb_valid_) {
    LC_LERROR(COMMON) << "invalid extrinsic Tvb";
    return LOC_LOCALIZATION_ERROR;
  }

  if (left_mul) {
    // Tbw = Tbv * Tvw
    return Transform(pose_v, pose_b, Tbv_, true);
  } else {
    // Twb = Twv * Tvb
    return Transform(pose_v, pose_b, Tvb_, false);
  }

  return LOC_LOCALIZATION_ERROR;
}

void TransformConfig::SetCameraIntrinsic(const std::string& camera_name,
                                         const CamIntrinsic& intrinsic) {
  if (cam_intrinsics_.count(camera_name)) {
    LC_LWARN(COMMON) << camera_name << "'s intrinsic is exist.";
    return;
  }
  cam_intrinsics_[camera_name] = intrinsic;
}

bool TransformConfig::GetCameraIntrinsic(const std::string& camera_name,
                                         CamIntrinsic* intrinsic) {
  if (!cam_intrinsics_.count(camera_name)) return false;
  *intrinsic = cam_intrinsics_.at(camera_name);
  return true;
}

void TransformConfig::SetCameraHomography(const std::string& camera_name,
                                          const cv::Mat& homo) {
  if (cam_homos_.count(camera_name)) {
    LC_LWARN(COMMON) << camera_name << "'s homography is exist.";
    return;
  }
  cam_homos_[camera_name] = homo;
}

bool TransformConfig::GetCameraHomography(const std::string& camera_name,
                                          cv::Mat* homo) {
  if (!cam_homos_.count(camera_name)) return false;
  *homo = cam_homos_.at(camera_name);
  return true;
}

void TransformConfig::SetTVehCam(const std::string& camera_name,
                                 const SE3d& Tvc) {
  if (Tveh_cams_.count(camera_name)) {
    LC_LWARN(COMMON) << camera_name << "'s extrinsic is exist.";
    return;
  }
  Tveh_cams_[camera_name] = Tvc;
}

bool TransformConfig::GetTVehCam(const std::string& camera_name, SE3d* Tvc) {
  if (!Tveh_cams_.count(camera_name)) return false;
  *Tvc = Tveh_cams_.at(camera_name);
  return true;
}

adLocStatus_t TransformConfig::FromRefNEDToENU(const SE3d& pose_ned,
                                               SE3d* pose_enu, bool left_mul) {
  if (!Tenu_ned_valid_) {
    LC_LERROR(COMMON) << "invalid reference frame transform";
    return LOC_LOCALIZATION_ERROR;
  }

  if (left_mul) {
    // Tenu_x = Tenu_ned * Tned_x
    return Transform(pose_ned, pose_enu, Tenu_ned_, true);
  } else {
    // Tx_enu = Tx_ned * Tned_enu
    return Transform(pose_ned, pose_enu, Tned_enu_, false);
  }

  return LOC_LOCALIZATION_ERROR;
}

adLocStatus_t TransformConfig::FromRefENUToNED(const SE3d& pose_enu,
                                               SE3d* pose_ned, bool left_mul) {
  if (!Tenu_ned_valid_) {
    LC_LERROR(COMMON) << "invalid reference frame transform";
    return LOC_LOCALIZATION_ERROR;
  }

  if (left_mul) {
    // Tned_x = Tned_enu * Tenu_x
    return Transform(pose_enu, pose_ned, Tned_enu_, true);
  } else {
    // Tx_ned = Tx_enu * Tenu_ned
    return Transform(pose_enu, pose_ned, Tenu_ned_, false);
  }

  return LOC_LOCALIZATION_ERROR;
}

}  // namespace localization
}  // namespace senseAD
