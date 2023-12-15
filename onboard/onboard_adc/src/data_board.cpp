/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: shenlianchen
 *Date: 2023-10-08
 *****************************************************************************/

#include "onboard/onboard_adc/include/data_board.h"

namespace hozon {
namespace mp {

DataBoard::DataBoard() {
  adsfi_lane_proto = std::make_shared<hozon::perception::TransportElement>();
  imu_proto = std::make_shared<hozon::soc::ImuIns>();
  chassis_proto = std::make_shared<hozon::soc::Chassis>();
  plugin_proto = std::make_shared<hozon::localization::HafNodeInfo>();
}

void DataBoard::Adsfi2Proto(const hz_Adsfi::AlgLaneDetectionOutArray& stu,
                            hozon::perception::TransportElement* proto) {
  if (proto == nullptr) {
    return;
  }

  const double tick =
      stu.header.timestamp.sec + stu.header.timestamp.nsec * 1e-9;
  const double gnssstamp =
      stu.header.gnssStamp.sec + stu.header.gnssStamp.nsec * 1e-9;
  proto->mutable_header()->set_publish_stamp(tick);
  proto->mutable_header()->set_gnss_stamp(gnssstamp);
  proto->mutable_header()->set_data_stamp(gnssstamp);
  proto->mutable_header()->set_seq(static_cast<int32_t>(stu.header.seq));
  proto->mutable_header()->set_frame_id(stu.header.frameID);

  auto all_lanes = stu.laneDetectionFrontOut;
  all_lanes.insert(all_lanes.end(), stu.laneDetectionRearOut.begin(),
                   stu.laneDetectionRearOut.end());
  for (const auto& it : all_lanes) {
    for (const auto& itt : it) {
      auto* proto_line = proto->add_lane();
      // proto_line->set_track_id(itt.track_id);
      switch (static_cast<int>(itt.cls)) {
        case 0:  // 单实线
          proto_line->set_lanetype(hozon::perception::SolidLine);
          break;
        case 1:  // 单虚线
          proto_line->set_lanetype(hozon::perception::DashedLine);
          break;
        case 2:  // 短粗虚线
          proto_line->set_lanetype(hozon::perception::ShortDashedLine);
          break;
        case 3:  // 双实线
          proto_line->set_lanetype(hozon::perception::DoubleSolidLine);
          break;
        case 4:  // 双虚线
          proto_line->set_lanetype(hozon::perception::DoubleDashedLine);
          break;
        case 5:  // 左实右虚
          proto_line->set_lanetype(hozon::perception::LeftSolidRightDashed);
          break;
        case 6:  // 左虚右实
          proto_line->set_lanetype(hozon::perception::RightSolidLeftDashed);
          break;
        case 7:  // 栅栏
          // 注意：proto消息类别里没有栅栏，这里赋值为Other
          proto_line->set_lanetype(hozon::perception::Other);
          break;
        case 8:  // 路沿
          proto_line->set_lanetype(hozon::perception::RoadEdge);
          break;
        case 9:  // 其它
          proto_line->set_lanetype(hozon::perception::Other);
          break;
        default:
          proto_line->set_lanetype(hozon::perception::Unknown);
          break;
      }

      switch (itt.lanelineSeq) {
        // rtfevent消息的顺序是：当前车道左侧从0向左递减，右侧从1向右递增，
        // 并且左侧路沿为-100，右侧路沿为100
        case 0:
          proto_line->set_lanepos(hozon::perception::EGO_LEFT);
          break;
        case -1:
          proto_line->set_lanepos(hozon::perception::ADJACENT_LEFT);
          break;
        case -2:
          proto_line->set_lanepos(hozon::perception::THIRD_LEFT);
          break;
        case -3:
          proto_line->set_lanepos(hozon::perception::FOURTH_LEFT);
          break;
        case -100:
          proto_line->set_lanepos(hozon::perception::BOLLARD_LEFT);
          break;
        case 1:
          proto_line->set_lanepos(hozon::perception::EGO_RIGHT);
          break;
        case 2:
          proto_line->set_lanepos(hozon::perception::ADJACENT_RIGHT);
          break;
        case 3:
          proto_line->set_lanepos(hozon::perception::THIRD_RIGHT);
          break;
        case 4:
          proto_line->set_lanepos(hozon::perception::FOURTH_RIGHT);
          break;
        case 100:
          proto_line->set_lanepos(hozon::perception::BOLLARD_RIGHT);
          break;
        default:
          proto_line->set_lanepos(hozon::perception::OTHER);
          break;
      }

      for (const auto& rpt : itt.pointVehicleCoord) {
        auto* ppt = proto_line->add_points();
        ppt->set_x(rpt.x);
        ppt->set_y(rpt.y);
        ppt->set_z(rpt.z);
      }

      auto* curve = proto_line->mutable_lane_param()->add_cubic_curve_set();
      curve->set_start_point_x(itt.laneFit.xStartVRF);
      curve->set_end_point_x(itt.laneFit.xEndVRF);
      curve->set_c0(itt.laneFit.coefficients.d);  // x^0
      curve->set_c1(itt.laneFit.coefficients.c);  // x^1
      curve->set_c2(itt.laneFit.coefficients.b);  // x^2
      curve->set_c3(itt.laneFit.coefficients.a);  // x^3

      // 注意：这里默认用geoConfidence
      proto_line->set_confidence(itt.geometryConfidence);

      // 注意：这里默认都是检测得到的，不是脑补的
      proto_line->set_use_type(hozon::perception::REAL);

      switch (static_cast<int>(itt.color)) {
        case 0:  // WHITE
          proto_line->set_color(hozon::perception::WHITE);
          break;
        case 1:  // YELLOW
          proto_line->set_color(hozon::perception::YELLOW);
          break;
        case 4:  // GREEN
          proto_line->set_color(hozon::perception::GREEN);
          break;
        // 注意：所有proto类型里没有的颜色，都设为了UNKNOWN
        case 2:  // ORANGE
        case 3:  // BLUE
        case 5:  // GRAY
        case 6:  // LEFT_WHITE_RIGHT_YELLOW
        case 7:  // LEFT_YELLOW_RIGHT_WHITE
        case 8:  // YELLOW_GRAY_FUSION
        case 9:  // OTHER
        default:
          proto_line->set_color(hozon::perception::UNKNOWN);
          break;
      }
    }
  }
}

void DataBoard::Adsfi2Proto(
    const std::shared_ptr<hz_Adsfi::AlgImuIns>& imuinsDataPtr_,
    std::shared_ptr<hozon::soc::ImuIns>& imu_proto) {
  /*********receive imu**********/
  if (!imuinsDataPtr_) {
    return;
  }

  imu_proto->mutable_header()->set_seq(
      static_cast<int32_t>(imuinsDataPtr_->header.seq));
  const double tick = imuinsDataPtr_->header.timestamp.sec +
                      imuinsDataPtr_->header.timestamp.nsec * 1e-9;
  const double gnssstamp = imuinsDataPtr_->header.gnssStamp.sec +
                           imuinsDataPtr_->header.gnssStamp.nsec * 1e-9;
  imu_proto->mutable_header()->set_publish_stamp(tick);
  imu_proto->mutable_header()->set_gnss_stamp(gnssstamp);
  imu_proto->mutable_header()->mutable_sensor_stamp()->set_imuins_stamp(
      gnssstamp);
  imu_proto->mutable_header()->set_data_stamp(tick);
  imu_proto->mutable_header()->set_frame_id(imuinsDataPtr_->header.frameID);
  // imu
  imu_proto->mutable_imu_info()->mutable_angular_velocity()->set_x(
      imuinsDataPtr_->imu_info.angularVelocity.x);
  imu_proto->mutable_imu_info()->mutable_angular_velocity()->set_y(
      imuinsDataPtr_->imu_info.angularVelocity.y);
  imu_proto->mutable_imu_info()->mutable_angular_velocity()->set_z(
      imuinsDataPtr_->imu_info.angularVelocity.z);

  // 单位: g
  imu_proto->mutable_imu_info()->mutable_linear_acceleration()->set_x(
      imuinsDataPtr_->imu_info.linearAcceleration.x);
  imu_proto->mutable_imu_info()->mutable_linear_acceleration()->set_y(
      imuinsDataPtr_->imu_info.linearAcceleration.y);
  imu_proto->mutable_imu_info()->mutable_linear_acceleration()->set_z(
      imuinsDataPtr_->imu_info.linearAcceleration.z);

  imu_proto->mutable_imu_info()->set_imuyaw(imuinsDataPtr_->imu_info.imuyaw);

  // ins
  imu_proto->mutable_ins_info()->mutable_attitude()->set_x(
      imuinsDataPtr_->ins_info.attitude.x);
  imu_proto->mutable_ins_info()->mutable_attitude()->set_y(
      imuinsDataPtr_->ins_info.attitude.y);
  imu_proto->mutable_ins_info()->mutable_attitude()->set_z(
      imuinsDataPtr_->ins_info.attitude.z);

  imu_proto->mutable_ins_info()->set_latitude(
      imuinsDataPtr_->ins_info.latitude);
  imu_proto->mutable_ins_info()->set_longitude(
      imuinsDataPtr_->ins_info.longitude);

  imu_proto->mutable_ins_info()->set_gps_status(
      imuinsDataPtr_->ins_info.gpsStatus);
  imu_proto->mutable_ins_info()->set_sys_status(
      imuinsDataPtr_->ins_info.sysStatus);
  imu_proto->mutable_ins_info()->set_heading(imuinsDataPtr_->ins_info.heading);

  imu_proto->mutable_ins_info()->mutable_augular_velocity()->set_x(
      imuinsDataPtr_->ins_info.augularVelocity.x);
  imu_proto->mutable_ins_info()->mutable_augular_velocity()->set_y(
      imuinsDataPtr_->ins_info.augularVelocity.y);
  imu_proto->mutable_ins_info()->mutable_augular_velocity()->set_z(
      imuinsDataPtr_->ins_info.augularVelocity.z);

  imu_proto->mutable_ins_info()->mutable_linear_velocity()->set_x(
      imuinsDataPtr_->ins_info.linearVelocity.x);
  imu_proto->mutable_ins_info()->mutable_linear_velocity()->set_y(
      imuinsDataPtr_->ins_info.linearVelocity.y);
  imu_proto->mutable_ins_info()->mutable_linear_velocity()->set_z(
      imuinsDataPtr_->ins_info.linearVelocity.z);

  imu_proto->mutable_ins_info()->mutable_linear_acceleration()->set_x(
      imuinsDataPtr_->imu_info.linearAcceleration.x);
  imu_proto->mutable_ins_info()->mutable_linear_acceleration()->set_y(
      imuinsDataPtr_->imu_info.linearAcceleration.y);
  imu_proto->mutable_ins_info()->mutable_linear_acceleration()->set_z(
      imuinsDataPtr_->imu_info.linearAcceleration.z);

  imu_proto->mutable_ins_info()->mutable_mounting_error()->set_x(
      imuinsDataPtr_->ins_info.mountingError.x);
  imu_proto->mutable_ins_info()->mutable_mounting_error()->set_y(
      imuinsDataPtr_->ins_info.mountingError.y);
  imu_proto->mutable_ins_info()->mutable_mounting_error()->set_z(
      imuinsDataPtr_->ins_info.mountingError.z);

  imu_proto->set_gps_sec(imuinsDataPtr_->gpsSec);
  imu_proto->set_gps_week(imuinsDataPtr_->gpsWeek);
}

void DataBoard::Adsfi2Proto(
    const std::shared_ptr<hz_Adsfi::AlgChassisInfo>& chassisDataPtr_,
    std::shared_ptr<hozon::soc::Chassis>& chassis_proto) {
  if (!chassisDataPtr_) {
    return;
  }
  const double tick = chassisDataPtr_->header.timestamp.sec +
                      chassisDataPtr_->header.timestamp.nsec * 1e-9;
  const double gnssstamp = chassisDataPtr_->header.gnssStamp.sec +
                           chassisDataPtr_->header.gnssStamp.nsec * 1e-9;
  chassis_proto->mutable_header()->set_publish_stamp(tick);
  chassis_proto->mutable_header()->set_gnss_stamp(gnssstamp);
  chassis_proto->mutable_header()->set_data_stamp(tick);
  chassis_proto->mutable_header()->mutable_sensor_stamp()->set_chassis_stamp(
      gnssstamp);
  // 档位
  auto gear_pos = chassisDataPtr_->vcu_info.VCU_ActGearPosition;
  switch (gear_pos) {
    case 0:
      chassis_proto->set_gear_location(
          hozon::soc::Chassis::GearPosition::Chassis_GearPosition_GEAR_NONE);
      break;
    case 1:
      chassis_proto->set_gear_location(
          hozon::soc::Chassis::GearPosition::Chassis_GearPosition_GEAR_PARKING);
      break;
    case 2:
      chassis_proto->set_gear_location(
          hozon::soc::Chassis::GearPosition::Chassis_GearPosition_GEAR_REVERSE);
      break;
    case 3:
      chassis_proto->set_gear_location(
          hozon::soc::Chassis::GearPosition::Chassis_GearPosition_GEAR_NEUTRAL);
      break;
    case 4:
      chassis_proto->set_gear_location(
          hozon::soc::Chassis::GearPosition::Chassis_GearPosition_GEAR_DRIVE);
      break;
    case 5:
      chassis_proto->set_gear_location(
          hozon::soc::Chassis::GearPosition::Chassis_GearPosition_GEAR_LOW);
      break;
    case 7:
    default:
      chassis_proto->set_gear_location(
          hozon::soc::Chassis::GearPosition::Chassis_GearPosition_GEAR_INVALID);
      break;
  }

  // 轮胎方向
  auto direction = chassisDataPtr_->wheel_info.ESC_FLWheelDirection;
  switch (direction) {
    case 0:
      chassis_proto->mutable_wheel_speed()->set_wheel_direction_fl(
          hozon::soc::WheelSpeed::WheelSpeedType::
              WheelSpeed_WheelSpeedType_STANDSTILL);
      break;
    case 1:
      chassis_proto->mutable_wheel_speed()->set_wheel_direction_fl(
          hozon::soc::WheelSpeed::WheelSpeedType::
              WheelSpeed_WheelSpeedType_FORWARD);
      break;
    case 2:
      chassis_proto->mutable_wheel_speed()->set_wheel_direction_fl(
          hozon::soc::WheelSpeed::WheelSpeedType::
              WheelSpeed_WheelSpeedType_BACKWARD);
      break;
    case 3:
      chassis_proto->mutable_wheel_speed()->set_wheel_direction_fl(
          hozon::soc::WheelSpeed::WheelSpeedType::
              WheelSpeed_WheelSpeedType_INVALID);
      break;
    default:
      chassis_proto->mutable_wheel_speed()->set_wheel_direction_fl(
          hozon::soc::WheelSpeed::WheelSpeedType::
              WheelSpeed_WheelSpeedType_BACKWARD);
      break;
  }

  direction = chassisDataPtr_->wheel_info.ESC_FRWheelDirection;
  switch (direction) {
    case 0:
      chassis_proto->mutable_wheel_speed()->set_wheel_direction_fr(
          hozon::soc::WheelSpeed::WheelSpeedType::
              WheelSpeed_WheelSpeedType_STANDSTILL);
      break;
    case 1:
      chassis_proto->mutable_wheel_speed()->set_wheel_direction_fr(
          hozon::soc::WheelSpeed::WheelSpeedType::
              WheelSpeed_WheelSpeedType_FORWARD);
      break;
    case 2:
      chassis_proto->mutable_wheel_speed()->set_wheel_direction_fr(
          hozon::soc::WheelSpeed::WheelSpeedType::
              WheelSpeed_WheelSpeedType_BACKWARD);
      break;
    case 3:
      chassis_proto->mutable_wheel_speed()->set_wheel_direction_fr(
          hozon::soc::WheelSpeed::WheelSpeedType::
              WheelSpeed_WheelSpeedType_INVALID);
      break;
    default:
      chassis_proto->mutable_wheel_speed()->set_wheel_direction_fr(
          hozon::soc::WheelSpeed::WheelSpeedType::
              WheelSpeed_WheelSpeedType_BACKWARD);
      break;
  }

  direction = chassisDataPtr_->wheel_info.ESC_RRWheelDirection;
  switch (direction) {
    case 0:
      chassis_proto->mutable_wheel_speed()->set_wheel_direction_rr(
          hozon::soc::WheelSpeed::WheelSpeedType::
              WheelSpeed_WheelSpeedType_STANDSTILL);
      break;
    case 1:
      chassis_proto->mutable_wheel_speed()->set_wheel_direction_rr(
          hozon::soc::WheelSpeed::WheelSpeedType::
              WheelSpeed_WheelSpeedType_FORWARD);
      break;
    case 2:
      chassis_proto->mutable_wheel_speed()->set_wheel_direction_rr(
          hozon::soc::WheelSpeed::WheelSpeedType::
              WheelSpeed_WheelSpeedType_BACKWARD);
      break;
    case 3:
      chassis_proto->mutable_wheel_speed()->set_wheel_direction_rr(
          hozon::soc::WheelSpeed::WheelSpeedType::
              WheelSpeed_WheelSpeedType_INVALID);
      break;
    default:
      chassis_proto->mutable_wheel_speed()->set_wheel_direction_rr(
          hozon::soc::WheelSpeed::WheelSpeedType::
              WheelSpeed_WheelSpeedType_BACKWARD);
      break;
  }

  direction = chassisDataPtr_->wheel_info.ESC_RLWheelDirection;
  switch (direction) {
    case 0:
      chassis_proto->mutable_wheel_speed()->set_wheel_direction_rl(
          hozon::soc::WheelSpeed::WheelSpeedType::
              WheelSpeed_WheelSpeedType_STANDSTILL);
      break;
    case 1:
      chassis_proto->mutable_wheel_speed()->set_wheel_direction_rl(
          hozon::soc::WheelSpeed::WheelSpeedType::
              WheelSpeed_WheelSpeedType_FORWARD);
      break;
    case 2:
      chassis_proto->mutable_wheel_speed()->set_wheel_direction_rl(
          hozon::soc::WheelSpeed::WheelSpeedType::
              WheelSpeed_WheelSpeedType_BACKWARD);
      break;
    case 3:
      chassis_proto->mutable_wheel_speed()->set_wheel_direction_rl(
          hozon::soc::WheelSpeed::WheelSpeedType::
              WheelSpeed_WheelSpeedType_INVALID);
      break;
    default:
      chassis_proto->mutable_wheel_speed()->set_wheel_direction_rl(
          hozon::soc::WheelSpeed::WheelSpeedType::
              WheelSpeed_WheelSpeedType_BACKWARD);
      break;
  }

  // 脉冲
  auto counter = chassisDataPtr_->wheel_info.ESC_FL_WhlPulCnt;
  chassis_proto->mutable_wheel_counter()->set_wheel_counter_fl(counter);
  counter = chassisDataPtr_->wheel_info.ESC_FR_WhlPulCnt;
  chassis_proto->mutable_wheel_counter()->set_wheel_counter_fr(counter);
  counter = chassisDataPtr_->wheel_info.ESC_RR_WhlPulCnt;
  chassis_proto->mutable_wheel_counter()->set_wheel_counter_rr(counter);
  counter = chassisDataPtr_->wheel_info.ESC_RL_WhlPulCnt;
  chassis_proto->mutable_wheel_counter()->set_wheel_counter_rl(counter);

  // 方向盘角度
  auto steer_angle = chassisDataPtr_->steering_info.SteeringAngle;
  chassis_proto->set_steering_angle(steer_angle);

  // 轮速
  auto vel = chassisDataPtr_->wheel_info.ESC_FLWheelSpeed;
  chassis_proto->mutable_wheel_speed()->set_wheel_spd_fl(vel);
  vel = chassisDataPtr_->wheel_info.ESC_FRWheelSpeed;
  chassis_proto->mutable_wheel_speed()->set_wheel_spd_fr(vel);
  vel = chassisDataPtr_->wheel_info.ESC_RRWheelSpeed;
  chassis_proto->mutable_wheel_speed()->set_wheel_spd_rr(vel);
  vel = chassisDataPtr_->wheel_info.ESC_RLWheelSpeed;
  chassis_proto->mutable_wheel_speed()->set_wheel_spd_rl(vel);
}

void DataBoard::Adsfi2Proto(const hz_Adsfi::AlgLocationNodeInfo& stu,
                            hozon::localization::HafNodeInfo* const proto) {
  if (proto == nullptr) {
    return;
  }

  const auto& node_info = stu.location_node_info;
  proto->Clear();
  proto->mutable_header()->set_seq(static_cast<int32_t>(node_info.header.seq));
  proto->mutable_header()->set_frame_id(node_info.header.frameId);

  const double tick =
      node_info.header.stamp.sec + node_info.header.stamp.nsec * 1e-9;
  const double gnssstamp =
      node_info.header.gnssStamp.sec + node_info.header.gnssStamp.nsec * 1e-9;
  proto->mutable_header()->set_publish_stamp(tick);
  proto->mutable_header()->set_gnss_stamp(gnssstamp);

  proto->set_is_valid(true);
  proto->set_type(
      static_cast<hozon::localization::HafNodeInfo_NodeType>(node_info.type));
  proto->set_gps_week(node_info.gpsWeek);
  proto->set_gps_sec(node_info.gpsSec);

  proto->mutable_pos_wgs()->set_x(node_info.posSmooth.x);
  proto->mutable_pos_wgs()->set_y(node_info.posSmooth.y);
  proto->mutable_pos_wgs()->set_z(node_info.posSmooth.z);
  proto->mutable_pos_gcj02()->set_x(node_info.posGCJ02.x);
  proto->mutable_pos_gcj02()->set_y(node_info.posGCJ02.y);
  proto->mutable_pos_gcj02()->set_z(node_info.posGCJ02.z);

  proto->mutable_attitude()->set_x(node_info.attitude.x);
  proto->mutable_attitude()->set_y(node_info.attitude.y);
  proto->mutable_attitude()->set_z(node_info.attitude.z);

  proto->mutable_quaternion()->set_x(node_info.quaternion.x);
  proto->mutable_quaternion()->set_y(node_info.quaternion.y);
  proto->mutable_quaternion()->set_z(node_info.quaternion.z);
  proto->mutable_quaternion()->set_w(node_info.quaternion.w);

  proto->mutable_linear_velocity()->set_x(node_info.linearVelocity.x);
  proto->mutable_linear_velocity()->set_y(node_info.linearVelocity.y);
  proto->mutable_linear_velocity()->set_z(node_info.linearVelocity.z);

  proto->mutable_angular_velocity()->set_x(node_info.angularVelocity.x);
  proto->mutable_angular_velocity()->set_y(node_info.angularVelocity.y);
  proto->mutable_angular_velocity()->set_z(node_info.angularVelocity.z);

  proto->mutable_linear_acceleration()->set_x(node_info.linearAcceleration.x);
  proto->mutable_linear_acceleration()->set_y(node_info.linearAcceleration.y);
  proto->mutable_linear_acceleration()->set_z(node_info.linearAcceleration.z);

  proto->mutable_gyro_bias()->set_x(node_info.gyroBias.x);
  proto->mutable_gyro_bias()->set_y(node_info.gyroBias.y);
  proto->mutable_gyro_bias()->set_z(node_info.gyroBias.z);

  proto->mutable_accel_bias()->set_x(node_info.accelBias.x);
  proto->mutable_accel_bias()->set_y(node_info.accelBias.y);
  proto->mutable_accel_bias()->set_z(node_info.accelBias.z);

  proto->mutable_sd_position()->set_x(node_info.sdPosition.x);
  proto->mutable_sd_position()->set_y(node_info.sdPosition.y);
  proto->mutable_sd_position()->set_z(node_info.sdPosition.z);

  proto->mutable_sd_velocity()->set_x(node_info.sdVelocity.x);
  proto->mutable_sd_velocity()->set_y(node_info.sdVelocity.y);
  proto->mutable_sd_velocity()->set_z(node_info.sdVelocity.z);

  proto->mutable_sd_attitude()->set_x(node_info.sdAttitude.x);
  proto->mutable_sd_attitude()->set_y(node_info.sdAttitude.y);
  proto->mutable_sd_attitude()->set_z(node_info.sdAttitude.z);

  for (const auto& c : node_info.covariance) {
    proto->add_covariance(c);
  }

  proto->set_sys_status(node_info.sysStatus);
  proto->set_gps_status(node_info.gpsStatus);
  proto->set_heading(node_info.heading);
  proto->set_warn_info(node_info.warn_info);

  proto->mutable_mounting_error()->set_x(node_info.mountingError.x);
  proto->mutable_mounting_error()->set_y(node_info.mountingError.y);
  proto->mutable_mounting_error()->set_z(node_info.mountingError.z);

  proto->set_sensor_used(node_info.sensorUsed);
  proto->set_wheel_velocity(node_info.wheelVelocity);
  proto->set_odo_sf(node_info.odoSF);
  proto->set_valid_estimate(node_info.validEstimate);
}

}  // namespace mp
}  // namespace hozon
