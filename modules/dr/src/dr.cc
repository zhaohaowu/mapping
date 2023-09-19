/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: Hozon
 *Date: 2023-08-28
 *****************************************************************************/
#include "modules/dr/include/dr.h"

#include <ctime>

namespace hozon {
namespace mp {
namespace dr {

DRInterface::DRInterface() { dr_estimator_ = std::make_shared<WheelOdom>(); }

void DRInterface::SetLocation(
    std::shared_ptr<hozon::dead_reckoning::DeadReckoning> locationDataPtr) {
  /**********cmp pose************/
  dr_estimator_->update();
  HLOG_INFO << "------------1---------";

  /**********send pose************/
  // TODO(ZXL) 只有Chassis 得时候, 这个变量不修改
  if (dr_estimator_->initialized_) {
    OdometryData latest_odom = dr_estimator_->get_latest_odom_data();

    Eigen::Quaterniond qat(latest_odom.odometry.qw, latest_odom.odometry.qx,
                           latest_odom.odometry.qy, latest_odom.odometry.qz);
    Eigen::Vector3d eulerAngle = Qat2EulerAngle(qat) * 57.3;

    // frame->eulerAngle = eulerAngle * 57.3;

    //   frame->loc_omg = latest_odom.loc_omg * 57.3;
    SetLocationData(locationDataPtr, latest_odom, eulerAngle);

  } else {
    // HLOG_INFO << "dr is not init";
  }
}

Eigen::Vector3d DRInterface::Qat2EulerAngle(const Eigen::Quaterniond &q) {
  Eigen::Vector3d eulerangle = {0, 0, 0};
  double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
  double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
  eulerangle[0] = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
  if (fabs(sinp) >= 1) {
    eulerangle[1] = copysign(M_PI / 2, sinp);
  } else {
    eulerangle[1] = asin(sinp);
  }
  // yaw (z-axis rotation)
  double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
  double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  eulerangle[2] = atan2(siny_cosp, cosy_cosp);
  if (eulerangle[2] < 0) eulerangle[2] += 2 * M_PI;
  return eulerangle;
}

void DRInterface::SetInsData2Location(
    std::shared_ptr<hozon::dead_reckoning::DeadReckoning> locationDataPtr,
    const OdometryData &odom_data) {
  // 位置的欧拉角
  locationDataPtr->mutable_pose()
      ->mutable_pose_local()
      ->mutable_euler_angle()
      ->set_x(odom_data.attitude[0]);
  locationDataPtr->mutable_pose()
      ->mutable_pose_local()
      ->mutable_euler_angle()
      ->set_y(odom_data.attitude[1]);

  //   locationDataPtr->pose.poseLOCAL.eulerAngle.x = ins_data_.attitude[0];
  //   locationDataPtr->pose.poseLOCAL.eulerAngle.y = ins_data_.attitude[1];

  // 原始的线性加速度
  // locationDataPtr->acceleration.linearVRF.linearRawVRF.x =
  //     imu_data.linearAcceleration[0];
  // locationDataPtr->acceleration.linearVRF.linearRawVRF.y =
  //     imu_data.linearAcceleration[1];
  // locationDataPtr->acceleration.linearVRF.linearRawVRF.z =
  //     imu_data.linearAcceleration[2];

  //   locationDataPtr->mutable_acceleration()
  //       ->mutable_linear_vrf()
  //       ->mutable_linear_raw_vrf()
  //       ->set_x(ins_data_.linearAcceleration[0]);
  //   locationDataPtr->mutable_acceleration()
  //       ->mutable_linear_vrf()
  //       ->mutable_linear_raw_vrf()
  //       ->set_y(ins_data_.linearAcceleration[1]);
  //   locationDataPtr->mutable_acceleration()
  //       ->mutable_linear_vrf()
  //       ->mutable_linear_raw_vrf()
  //       ->set_z(ins_data_.linearAcceleration[2]);

  // 角速度
  // locationDataPtr->mutable_velocity()->mutable_twist_vrf()->mutable_angular_vrf()->set_x(odom_data.augularVelocity[0]);
  // locationDataPtr->mutable_velocity()->mutable_twist_vrf()->mutable_angular_vrf()->set_y(odom_data.augularVelocity[1]);
  // locationDataPtr->mutable_velocity()->mutable_twist_vrf()->mutable_angular_vrf()->set_z(odom_data.augularVelocity[2]);

  //   locationDataPtr->velocity.twistVRF.angularVRF.x =
  //       ins_data_.augularVelocity[0];
  //   locationDataPtr->velocity.twistVRF.angularVRF.y =
  //       ins_data_.augularVelocity[1];
  //   locationDataPtr->velocity.twistVRF.angularVRF.z =
  //       ins_data_.augularVelocity[2];
}

void DRInterface::SetLocationData(
    std::shared_ptr<hozon::dead_reckoning::DeadReckoning> locationDataPtr,
    OdometryData &latest_odom, Eigen::Vector3d &eulerAngle) {
  if (locationDataPtr == nullptr) {
    // HLOG_ERROR << " send localization input frame is nullptr";
    return;
  }
  // 位置
  locationDataPtr->mutable_pose()
      ->mutable_pose_local()
      ->mutable_position()
      ->set_x(latest_odom.odometry.x);
  locationDataPtr->mutable_pose()
      ->mutable_pose_local()
      ->mutable_position()
      ->set_y(latest_odom.odometry.y);
  locationDataPtr->mutable_pose()
      ->mutable_pose_local()
      ->mutable_position()
      ->set_z(0.0);

  // 四元数角度
  locationDataPtr->mutable_pose()
      ->mutable_pose_local()
      ->mutable_quaternion()
      ->set_w(latest_odom.odometry.qw);
  locationDataPtr->mutable_pose()
      ->mutable_pose_local()
      ->mutable_quaternion()
      ->set_x(latest_odom.odometry.qx);
  locationDataPtr->mutable_pose()
      ->mutable_pose_local()
      ->mutable_quaternion()
      ->set_y(latest_odom.odometry.qy);
  locationDataPtr->mutable_pose()
      ->mutable_pose_local()
      ->mutable_quaternion()
      ->set_z(latest_odom.odometry.qz);

  SetInsData2Location(locationDataPtr, latest_odom);

  locationDataPtr->mutable_pose()
      ->mutable_pose_local()
      ->mutable_euler_angle()
      ->set_z(eulerAngle[2]);

  locationDataPtr->mutable_pose()->mutable_pose_local()->set_heading(
      eulerAngle[2]);

  HLOG_INFO << "==== init ===="
            << " output---vel " << latest_odom.loc_vel[1] << " acc  "
            << latest_odom.loc_acc[0] << " qx " << latest_odom.odometry.qx;

  // 速度
  locationDataPtr->mutable_velocity()
      ->mutable_twist_vrf()
      ->mutable_linear_vrf()
      ->set_x(latest_odom.loc_vel[0]);
  locationDataPtr->mutable_velocity()
      ->mutable_twist_vrf()
      ->mutable_linear_vrf()
      ->set_y(latest_odom.loc_vel[1]);
  locationDataPtr->mutable_velocity()
      ->mutable_twist_vrf()
      ->mutable_linear_vrf()
      ->set_z(latest_odom.loc_vel[2]);

  // 角度读
  locationDataPtr->mutable_velocity()
      ->mutable_twist_vrf()
      ->mutable_angular_raw_vrf()
      ->set_x(latest_odom.loc_omg[0]);
  locationDataPtr->mutable_velocity()
      ->mutable_twist_vrf()
      ->mutable_angular_raw_vrf()
      ->set_y(latest_odom.loc_omg[1]);
  locationDataPtr->mutable_velocity()
      ->mutable_twist_vrf()
      ->mutable_angular_raw_vrf()
      ->set_z(latest_odom.loc_omg[2]);

  // 加速度
  locationDataPtr->mutable_acceleration()
      ->mutable_linear_vrf()
      ->mutable_linear_raw_vrf()
      ->set_x(latest_odom.loc_acc[0] / 9.80645);
  locationDataPtr->mutable_acceleration()
      ->mutable_linear_vrf()
      ->mutable_linear_raw_vrf()
      ->set_y(latest_odom.loc_acc[1] / 9.80645);
  locationDataPtr->mutable_acceleration()
      ->mutable_linear_vrf()
      ->mutable_linear_raw_vrf()
      ->set_z(latest_odom.loc_acc[2] / 9.80645);

  // locationDataPtr->mutable_acceleration()
  //     ->mutable_linear_vrf()
  //     ->mutable_angular_vrf()
  //     ->set_x(0);
  // locationDataPtr->mutable_acceleration()
  //     ->mutable_linear_vrf()
  //     ->mutable_angular_vrf()
  //     ->set_y(0);
  // locationDataPtr->mutable_acceleration()
  //     ->mutable_linear_vrf()
  //     ->mutable_angular_vrf()
  //     ->set_z(0);

  locationDataPtr->mutable_header()->set_timestamp_sec(latest_odom.timestamp);

  double sys_timestamp = GetCurrentNsecTime();

  locationDataPtr->mutable_header()->set_frame_id("HZ_DR");
  static int32_t count_ = 0;
  locationDataPtr->mutable_header()->set_sequence_num(count_);
  count_++;
  locationDataPtr->set_gnss_timestamp(sys_timestamp);
  //   locationDataPtr->set_location_state(21);

  //   locationDataPtr->set_is_valid(true);
  //   locationDataPtr->set_coord_type(
  //       hozon::perception::datacollection::CoordType::SLAM_COORD);

  HLOG_INFO
      << "==== init ==== **************************************DR pose: x: "
      << locationDataPtr->pose().pose_local().position().x()
      << " ,y: " << locationDataPtr->pose().pose_local().position().y()
      << ",z:" << locationDataPtr->pose().pose_local().position().z();
  HLOG_INFO << "==== init ==== DR quat: x: "
            << locationDataPtr->pose().pose_local().quaternion().x()
            << " ,y: " << locationDataPtr->pose().pose_local().quaternion().y()
            << ",z:" << locationDataPtr->pose().pose_local().quaternion().z()
            << ",w:" << locationDataPtr->pose().pose_local().quaternion().w()
            << " ";
  HLOG_INFO << "==== init ====  DR vel: x: "
            << locationDataPtr->velocity().twist_vrf().linear_vrf().x()
            << ",y: "
            << locationDataPtr->velocity().twist_vrf().linear_vrf().y()
            << ",z:" << locationDataPtr->velocity().twist_vrf().linear_vrf().z()
            << " ";

  HLOG_INFO << "==== init ====  DR acc: x: "
            << locationDataPtr->acceleration().linear_vrf().linear_raw_vrf().x()
            << ",y: "
            << locationDataPtr->acceleration().linear_vrf().linear_raw_vrf().y()
            << ",z:"
            << locationDataPtr->acceleration().linear_vrf().linear_raw_vrf().z()
            << " ";
}

void DRInterface::AddImuData(
    std::shared_ptr<hozon::drivers::imuIns::ImuIns> imu_proto) {
  ImuDataHozon imu_data;
  ConvertImuData(imu_proto, imu_data);
  dr_estimator_->add_imu_data(imu_data);
}

void DRInterface::AddChassisData(
    std::shared_ptr<hozon::canbus::Chassis> chassis_proto) {
  WheelDataHozon wheel_data;
  ConvertChassisData(chassis_proto, wheel_data);
  dr_estimator_->add_wheel_data(wheel_data);
}

void DRInterface::ConvertImuData(
    std::shared_ptr<hozon::drivers::imuIns::ImuIns> imu_proto,
    ImuDataHozon &imu_data) {
  if (!imu_proto) {
    HLOG_ERROR << "Parking_SLAM: no imu data !";
    // fm_.ReportFault(LOC_IMU_DATA_ERROR);
    return;
  }

  imu_data.timestamp = imu_proto->header().timestamp_sec();
  //   imu_proto->header.timestamp.sec + imu_proto->header.timestamp.nsec / 1e9;

  imu_data.gyr_measurement = Eigen::Vector3d(
      imu_proto->imu_info().angular_velocity().x() * M_PI / 180.0,
      imu_proto->imu_info().angular_velocity().y() * M_PI / 180.0,
      imu_proto->imu_info().angular_velocity().z() * M_PI / 180.0);

  // this imu is based in 9.8 per value
  imu_data.acc_measurement = Eigen::Vector3d(
      imu_proto->imu_info().linear_acceleration().x() * 9.80645,
      imu_proto->imu_info().linear_acceleration().y() * 9.80645,
      imu_proto->imu_info().linear_acceleration().z() * 9.80645);

  //   imu_data.ins_data = Eigen::Vector3d(imu_proto->ins_info().latitude(),
  //                                       imu_proto->ins_info().longitude(),
  //                                       imu_proto->ins_info().attitude().y());
  // imu_data.ins_data.gpsStatus = imu_proto->ins_info.gpsStatus;

  imu_data.gpsStatus = imu_proto->ins_info().gps_status();
  // imu_proto->ins_info.gpsStatus;
  imu_data.latitude = imu_proto->ins_info().latitude();
  // imu_proto->ins_info.latitude;
  imu_data.longitude = imu_proto->ins_info().longitude();
  // imu_proto->ins_info.longitude;
  imu_data.attitude << imu_proto->ins_info().attitude().x(),
      imu_proto->ins_info().attitude().y(),
      imu_proto->ins_info().attitude().z();

  imu_data.gyo_bias << imu_proto->offset_info().gyo_bias().x(),
      imu_proto->offset_info().gyo_bias().y(),
      imu_proto->offset_info().gyo_bias().z();
  imu_data.acc_bias << imu_proto->offset_info().acc_bias().x(),
      imu_proto->offset_info().acc_bias().y(),
      imu_proto->offset_info().acc_bias().z();
  imu_data.mounting_error << imu_proto->ins_info().mounting_error().x(),
      imu_proto->ins_info().mounting_error().y(),
      imu_proto->ins_info().mounting_error().z();

  HLOG_INFO << "==imu data: " << imu_data.gpsStatus
            << " ,latitude: " << imu_data.latitude
            << ", longitude:" << imu_data.longitude << " ";
}

void DRInterface::ConvertChassisData(
    std::shared_ptr<hozon::canbus::Chassis> chassis_proto,
    WheelDataHozon &wheel_data) {
  if (!chassis_proto) {
    // fm_.ReportFault(LOC_IMU_DATA_ERROR);
    return;
  }
  double temp_timestamp = chassis_proto->header().timestamp_sec();

  wheel_data.timestamp = temp_timestamp;
  // 左前轮脉冲计数
  wheel_data.front_left_wheel =
      chassis_proto->wheel_counter().wheel_counter_fl();
  // chassis_proto->wheel_info.ESC_FL_WhlPulCnt;
  // 左前轮方向(0,1,2,3:静止,前进,后退,未知)
  wheel_data.front_left_dir =
      static_cast<int>(chassis_proto->wheel_speed().wheel_direction_fl());
  // chassis_proto->wheel_info.ESC_FLWheelDirection;

  wheel_data.front_right_wheel =
      chassis_proto->wheel_counter().wheel_counter_fr();
  wheel_data.front_right_dir =
      chassis_proto->wheel_speed().wheel_direction_fr();

  wheel_data.rear_left_wheel =
      chassis_proto->wheel_counter().wheel_counter_rl();
  wheel_data.rear_left_dir = chassis_proto->wheel_speed().wheel_direction_rl();

  wheel_data.rear_right_wheel =
      chassis_proto->wheel_counter().wheel_counter_rr();
  wheel_data.rear_right_dir = chassis_proto->wheel_speed().wheel_direction_rr();
  // 后左轮速度(km/h)
  if (wheel_data.rear_left_dir == 2) {
    wheel_data.rear_left_speed = -chassis_proto->wheel_speed().wheel_spd_rl();
    // -chassis_proto->wheel_info.ESC_RLWheelSpeed;
    wheel_data.rear_right_speed = -chassis_proto->wheel_speed().wheel_spd_rr();
  } else {
    wheel_data.rear_left_speed = chassis_proto->wheel_speed().wheel_spd_rl();
    wheel_data.rear_right_speed = chassis_proto->wheel_speed().wheel_spd_rr();
  }
  // 档位信息(0-7:默认,park,reverse,neutral,drive,s,invalid)
  wheel_data.gear = chassis_proto->gear_location();

  // chassis_proto->vcu_info.VCU_ActGearPosition;

  HLOG_INFO << "============== wheel gear: " << wheel_data.gear
            << ",fr wheel:" << wheel_data.front_left_wheel
            << " ,fr: " << chassis_proto->wheel_counter().wheel_counter_fl()
            << " ,";
}

}  // namespace dr
}  // namespace mp
}  // namespace hozon
