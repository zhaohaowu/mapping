/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: Hozon
 *Date: 2023-08-28
 *****************************************************************************/
#include "modules/dr/include/dr.h"

#include <ctime>
#include <thread>

namespace hozon {
namespace mp {
namespace dr {

DRInterface::DRInterface(const std::string& conf_path) {
  dr_estimator_ = std::make_shared<Odometry2D>(conf_path);
}

bool DRInterface::Process() {
  dr_estimator_->update();
  return true;
}

Eigen::Vector3d DRInterface::Qat2EulerAngle(const Eigen::Quaterniond& q) {
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
  if (eulerangle[2] < 0) {
    eulerangle[2] += 2 * M_PI;
  }
  return eulerangle;
}

bool DRInterface::GetLatestPose(
    double timestamp,
    std::shared_ptr<hozon::dead_reckoning::DeadReckoning> locationDataPtr,
    double sync_time) {
  if (dr_estimator_->initialized_) {
    OdometryData latest_odom = dr_estimator_->get_latest_odom_data();
    double time_diff = timestamp - latest_odom.timestamp;
    if (time_diff > 0.015) {
      HLOG_WARN << "ins_time - dr_time:" << time_diff;
    }

    double time_delay = timestamp - sync_time;
    HLOG_DEBUG << "time_delay:" << time_delay;

    static bool has_sync_time = sync_time > 1e-2;

    // 预测
    latest_odom.timestamp = timestamp;
    Eigen::Vector3d latest_pose = {
        latest_odom.odometry.x, latest_odom.odometry.y, latest_odom.odometry.z};
    Eigen::Quaterniond latest_qua(
        latest_odom.odometry.qw, latest_odom.odometry.qx,
        latest_odom.odometry.qy, latest_odom.odometry.qz);

    HLOG_DEBUG << "latest_pose:" << latest_pose.x() << "," << latest_pose.y()
               << "," << latest_pose.z();
    HLOG_DEBUG << "latest_odom.loc_vel:" << latest_odom.loc_vel.x() << ","
               << latest_odom.loc_vel.y() << "," << latest_odom.loc_vel.z();
    HLOG_DEBUG << "latest_qua:" << latest_qua.w() << "," << latest_qua.x()
               << "," << latest_qua.y() << "," << latest_qua.z();

    Eigen::Vector3d predict_pos =
        latest_pose + latest_qua * (latest_odom.loc_vel * time_diff);
    Eigen::Vector3d delta_ang = latest_odom.loc_omg * time_diff;
    Eigen::Quaterniond predict_qat = latest_qua;
    if (delta_ang.norm() > 1e-12) {
      predict_qat = latest_qua * Eigen::Quaterniond(Eigen::AngleAxisd(
                                     delta_ang.norm(), delta_ang.normalized()));
    }

    // 角度再预测
    Eigen::Vector3d delta_ang_again = latest_odom.loc_omg * time_delay;
    if (delta_ang_again.norm() > 1e-12 && has_sync_time) {
      predict_qat = predict_qat *
                    Eigen::Quaterniond(Eigen::AngleAxisd(
                        delta_ang_again.norm(), delta_ang_again.normalized()));
    }

    // 更新
    latest_odom.odometry.x = predict_pos[0];
    latest_odom.odometry.y = predict_pos[1];
    latest_odom.odometry.z = predict_pos[2];
    latest_odom.odometry.qx = predict_qat.x();
    latest_odom.odometry.qy = predict_qat.y();
    latest_odom.odometry.qz = predict_qat.z();
    latest_odom.odometry.qw = predict_qat.w();

    HLOG_DEBUG << "wsj_pos_veh_start" << latest_odom.odometry.x << ","
               << latest_odom.odometry.y << "," << latest_odom.odometry.z
               << "wsj_pos_veh_end";

    HLOG_DEBUG << "wsj_theta_q_start" << latest_odom.odometry.qw << ","
               << latest_odom.odometry.qx << "," << latest_odom.odometry.qy
               << "," << latest_odom.odometry.qz << "wsj_theta_q_end";

    HLOG_DEBUG << "wsj_dr_time_start" << latest_odom.timestamp
               << "wsj_dr_time_end";

    HLOG_DEBUG << "wsj_filter_vel_start" << (latest_odom.loc_vel.x())
               << "wsj_filter_vel_end";

    SetLocationData(std::move(locationDataPtr), latest_odom);
    return true;
  } else {
    // HLOG_WARN << "DR: is not init";
    return false;
  }
}

void DRInterface::SetLocationData(
    std::shared_ptr<hozon::dead_reckoning::DeadReckoning> locationDataPtr,
    const OdometryData& latest_odom) {
  static int dr_seq_cnt = 0;
  if (locationDataPtr == nullptr) {
    // HLOG_ERROR << "DR: send localization input frame is nullptr";
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

  //   // 四元数角度
  locationDataPtr->mutable_pose()
      ->mutable_pose_local()
      ->mutable_quaternion()
      ->set_w(static_cast<float>(latest_odom.odometry.qw));
  locationDataPtr->mutable_pose()
      ->mutable_pose_local()
      ->mutable_quaternion()
      ->set_x(static_cast<float>(latest_odom.odometry.qx));
  locationDataPtr->mutable_pose()
      ->mutable_pose_local()
      ->mutable_quaternion()
      ->set_y(static_cast<float>(latest_odom.odometry.qy));
  locationDataPtr->mutable_pose()
      ->mutable_pose_local()
      ->mutable_quaternion()
      ->set_z(static_cast<float>(latest_odom.odometry.qz));

  // 欧拉角
  locationDataPtr->mutable_pose()
      ->mutable_pose_local()
      ->mutable_euler_angle()
      ->set_x(latest_odom.attitude[0]);
  locationDataPtr->mutable_pose()
      ->mutable_pose_local()
      ->mutable_euler_angle()
      ->set_y(latest_odom.attitude[1]);
  Eigen::Quaterniond qat(latest_odom.odometry.qw, latest_odom.odometry.qx,
                         latest_odom.odometry.qy, latest_odom.odometry.qz);
  Eigen::Vector3d eulerAngle = Qat2EulerAngle(qat) * 57.3;
  locationDataPtr->mutable_pose()
      ->mutable_pose_local()
      ->mutable_euler_angle()
      ->set_z(eulerAngle[2]);

  // heading
  locationDataPtr->mutable_pose()->mutable_pose_local()->set_heading(
      static_cast<float>(eulerAngle[2]));

  HLOG_DEBUG << "wsj_dr_heading_start" << eulerAngle[2] << "wsj_dr_heading_end";

  // 速度
  locationDataPtr->mutable_velocity()
      ->mutable_twist_vrf()
      ->mutable_linear_vrf()
      ->set_x(static_cast<float>(latest_odom.loc_vel[0]));
  locationDataPtr->mutable_velocity()
      ->mutable_twist_vrf()
      ->mutable_linear_vrf()
      ->set_y(static_cast<float>(latest_odom.loc_vel[1]));
  locationDataPtr->mutable_velocity()
      ->mutable_twist_vrf()
      ->mutable_linear_vrf()
      ->set_z(static_cast<float>(latest_odom.loc_vel[2]));

  // 原始角速度
  locationDataPtr->mutable_velocity()
      ->mutable_twist_vrf()
      ->mutable_angular_raw_vrf()
      ->set_x(static_cast<float>(latest_odom.ins_gyr[0]));
  locationDataPtr->mutable_velocity()
      ->mutable_twist_vrf()
      ->mutable_angular_raw_vrf()
      ->set_y(static_cast<float>(latest_odom.ins_gyr[1]));
  locationDataPtr->mutable_velocity()
      ->mutable_twist_vrf()
      ->mutable_angular_raw_vrf()
      ->set_z(static_cast<float>(latest_odom.ins_gyr[2]));

  // 计算角速度
  locationDataPtr->mutable_velocity()
      ->mutable_twist_vrf()
      ->mutable_angular_vrf()
      ->set_x(static_cast<float>(latest_odom.loc_omg[0]));
  locationDataPtr->mutable_velocity()
      ->mutable_twist_vrf()
      ->mutable_angular_vrf()
      ->set_y(static_cast<float>(latest_odom.loc_omg[1]));
  locationDataPtr->mutable_velocity()
      ->mutable_twist_vrf()
      ->mutable_angular_vrf()
      ->set_z(static_cast<float>(latest_odom.loc_omg[2]));

  // 加速度
  locationDataPtr->mutable_acceleration()
      ->mutable_linear_vrf()
      ->mutable_linear_raw_vrf()
      ->set_x(static_cast<float>(latest_odom.ins_acc[0]));
  locationDataPtr->mutable_acceleration()
      ->mutable_linear_vrf()
      ->mutable_linear_raw_vrf()
      ->set_y(static_cast<float>(latest_odom.ins_acc[1]));
  locationDataPtr->mutable_acceleration()
      ->mutable_linear_vrf()
      ->mutable_linear_raw_vrf()
      ->set_z(static_cast<float>(latest_odom.ins_acc[2]));

  locationDataPtr->mutable_acceleration()
      ->mutable_linear_vrf()
      ->mutable_angular_vrf()
      ->set_x(0);
  locationDataPtr->mutable_acceleration()
      ->mutable_linear_vrf()
      ->mutable_angular_vrf()
      ->set_y(0);
  locationDataPtr->mutable_acceleration()
      ->mutable_linear_vrf()
      ->mutable_angular_vrf()
      ->set_z(0);

  locationDataPtr->mutable_header()->set_seq(dr_seq_cnt);
  locationDataPtr->mutable_header()->set_gnss_stamp(latest_odom.timestamp);
  locationDataPtr->mutable_header()->set_data_stamp(latest_odom.timestamp);

  locationDataPtr->mutable_header()->mutable_sensor_stamp()->set_chassis_stamp(
      latest_odom.chassis_stamp);
  locationDataPtr->mutable_header()->mutable_sensor_stamp()->set_imuins_stamp(
      latest_odom.imu_stamp);

  double sys_timestamp = GetCurrentNsecTime();
  locationDataPtr->mutable_header()->set_publish_stamp(sys_timestamp);

  locationDataPtr->mutable_header()->set_frame_id("HZ_DR");
  dr_seq_cnt++;

  //   std::cout << std::setprecision(15)
  //             << "MDR: " << locationDataPtr->mutable_header()->data_stamp()
  //             << ","
  //             << latest_odom.chassis_seq << "," << latest_odom.odometry.x <<
  //             ","
  //             << latest_odom.odometry.y << "," << eulerAngle[2] << std::endl;
}

void DRInterface::AddImuData(
    const std::shared_ptr<const hozon::soc::ImuIns>& imu_proto) {
  ImuDataHozon imu_data;
  ConvertImuData(imu_proto, imu_data);
  dr_estimator_->add_imu_data(imu_data);
}

void DRInterface::AddChassisData(
    const std::shared_ptr<const hozon::soc::Chassis>& chassis_proto) {
  WheelDataHozon wheel_data;
  ConvertChassisData(chassis_proto, wheel_data);
  dr_estimator_->add_wheel_data(wheel_data);
}

void DRInterface::ConvertImuData(
    const std::shared_ptr<const hozon::soc::ImuIns>& imu_proto,
    ImuDataHozon& imu_data) {
  if (!imu_proto) {
    // HLOG_ERROR << "DR: no imu data !";
    return;
  }

  // imu_data.timestamp = imu_proto->header().data_stamp();
  imu_data.timestamp = imu_proto->header().sensor_stamp().imuins_stamp();
  imu_data.sync_stamp = imu_proto->sync_domain_time_s() * 1e-9;
  // imu_data.seq =
  // std::cout << std::setprecision(15) << "Imu datastamp: " <<
  // imu_data.timestamp
  //           << std::endl;
  // HLOG_ERROR << "Imu datastamp: " << imu_data.timestamp;
  //   if (imu_data.timestamp == 0) {
  //     imu_data.timestamp = imu_proto->header().publish_stamp();
  //   }

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

  imu_data.gpsStatus = static_cast<int>(imu_proto->ins_info().gps_status());
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
  imu_data.sdPosition << imu_proto->ins_info().sd_position().x(),
      imu_proto->ins_info().sd_position().y(),
      imu_proto->ins_info().sd_position().z();
  /*
   * 接收ins的加速度　角速度
   */
  imu_data.ins_acc << imu_proto->ins_info().linear_acceleration().x(),
      imu_proto->ins_info().linear_acceleration().y(),
      imu_proto->ins_info().linear_acceleration().z();
  imu_data.ins_gyr << imu_proto->ins_info().augular_velocity().x(),
      imu_proto->ins_info().augular_velocity().y(),
      imu_proto->ins_info().augular_velocity().z();
}

void DRInterface::ConvertChassisData(
    const std::shared_ptr<const hozon::soc::Chassis>& chassis_proto,
    WheelDataHozon& wheel_data) {
  if (!chassis_proto) {
    // HLOG_ERROR << "DR: no chassis data !";
    return;
  }
  // wheel_data.timestamp = chassis_proto->header().data_stamp();
  wheel_data.timestamp = chassis_proto->header().sensor_stamp().chassis_stamp();
  //   if (wheel_data.timestamp == 0) {
  //     wheel_data.timestamp = chassis_proto->header().publish_stamp();
  //   }

  wheel_data.seq = chassis_proto->header().seq();

  // 左前轮脉冲计数
  wheel_data.front_left_wheel =
      chassis_proto->wheel_counter().wheel_counter_fl();
  // 左前轮方向(0,1,2,3:静止,前进,后退,未知)
  wheel_data.front_left_dir =
      static_cast<int>(chassis_proto->wheel_speed().wheel_direction_fl());

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
  if (wheel_data.rear_left_dir == hozon::soc::WheelSpeed_WheelSpeedType::
                                      WheelSpeed_WheelSpeedType_BACKWARD) {
    wheel_data.rear_left_speed = -chassis_proto->wheel_speed().wheel_spd_rl();
    // -chassis_proto->wheel_info.ESC_RLWheelSpeed;
    wheel_data.rear_right_speed = -chassis_proto->wheel_speed().wheel_spd_rr();
  } else {
    wheel_data.rear_left_speed = chassis_proto->wheel_speed().wheel_spd_rl();
    wheel_data.rear_right_speed = chassis_proto->wheel_speed().wheel_spd_rr();
  }
  // 档位信息(0-7:默认,park,reverse,neutral,drive,s,invalid)
  wheel_data.gear = chassis_proto->gear_location();

  // rolling counter
  wheel_data.rolling_counter = chassis_proto->idb4_msgcounter();

  // chassis_proto->vcu_info.VCU_ActGearPosition;

  //   HLOG_DEBUG << "============== wheel gear: " << wheel_data.gear
  //             << ",fr wheel:" << wheel_data.front_left_wheel
  //             << " ,fr: " <<
  //             chassis_proto->wheel_counter().wheel_counter_fl()
  //             << " ,";
}

}  // namespace dr
}  // namespace mp
}  // namespace hozon
