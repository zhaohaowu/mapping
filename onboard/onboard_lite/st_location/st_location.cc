/******************************************************************************
 *   Copyright (C) 2024 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： st_location.cc
 *   author     ： wangmeng
 *   date       ： 2024.04
 ******************************************************************************/
#include <base/utils/log.h>
#include <gflags/gflags.h>
#include <perception-lib/lib/environment/environment.h>

#include "modules/util/include/util/rviz_agent/rviz_agent.h"
#include "perception-base/base/state_machine/state_machine_info.h"
#include "yaml-cpp/yaml.h"

// ==st
#include "onboard/onboard_lite/st_location/st_location.h"

using hozon::netaos::adf_lite::Bundle;

namespace hozon {
namespace perception {
namespace common_onboard {

constexpr char* const kImuInsTopic = "imu_ins";
constexpr char* const kChassisTopic = "chassis";
constexpr char* const kGnssTopic = "gnssinfo";
// todo:感知topic后续换成"local_map",只关注地面，反正percep_transport没有空中信息;感知需求
constexpr char* const kPerceptionTopic = "percep_transport";
constexpr char* const kRunningModeTopic = "running_mode";
constexpr char* const kStOutputTopic = "/location/st_location";

int32_t STLocation::AlgInit() {
  std::cout << " =====================================wangmeng init0" << std::endl;
  std::cout << " =====================================wangmeng init1" << std::endl;
  std::cout << " =====================================wangmeng init2" << std::endl;
  std::cout << " =====================================wangmeng init3" << std::endl;
  //  pose_estimation_ = std::make_unique<MapMatching>();
  //  const std::string adflite_root_path =
  //      hozon::perception::lib::GetEnv("ADFLITE_ROOT_PATH", ".");
  //  const std::string config_yaml = adflite_root_path + "/" +
  //  FLAGS_config_yaml; const std::string config_cam_yaml =
  //      adflite_root_path + "/" + FLAGS_config_cam_yaml;
  //  const std::string pose_estimation_lite_config_yaml =
  //      adflite_root_path + "/" + FLAGS_pose_estimation_lite_config_yaml;
  //  if (!pose_estimation_->Init(config_yaml, config_cam_yaml)) {
  //    return -1;
  //  }

  // RegistMessageType();
  // RegistAlgProcessFunc(
  //     "receive_chassis",
  //     std::bind(&STLocation::OnChassis, this, std::placeholders::_1));
  // RegistAlgProcessFunc("receive_imu_ins", std::bind(&STLocation::OnImuIns, this,
  //                                                   std::placeholders::_1));
  // RegistAlgProcessFunc("receive_gnss", std::bind(&STLocation::OnGnss, this,
  //                                                std::placeholders::_1));
  // RegistAlgProcessFunc(
  //     "recv_perception",
  //     std::bind(&STLocation::OnPerception, this, std::placeholders::_1));
  // RegistAlgProcessFunc(
  //     "recv_running_mode",
  //     std::bind(&STLocation::OnRunningMode, this, std::placeholders::_1));
  // RegistAlgProcessFunc(
  //     "send_st_location_result",
  //     std::bind(&STLocation::OnLocOutput, this, std::placeholders::_1));

  //  YAML::Node config = YAML:  // latest_local_map_data_ =
  //     std::make_shared<senseAD::localization::RoadStructure>();:LoadFile(pose_estimation_lite_config_yaml);
  //  auto use_rviz_bridge = config["use_rviz_bridge"].as<bool>();
  //  if (use_rviz_bridge) {
  //    auto viz_addr = config["viz_addr"].as<std::string>();
  //    int ret = mp::util::RvizAgent::Instance().Init(viz_addr);
  //    if (ret < 0) {
  //      HLOG_WARN << "RvizAgent init failed:" << viz_addr;
  //    }
  //  }
  // latest_local_map_data_ =
  //     std::make_shared<senseAD::localization::RoadStructure>();
  return 0;
}

void STLocation::AlgRelease() {
  if (mp::util::RvizAgent::Instance().Ok()) {
    mp::util::RvizAgent::Instance().Term();
  }
}

// void STLocation::RegistMessageType() const {
//   REGISTER_PROTO_MESSAGE_TYPE(kImuInsTopic, hozon::soc::ImuIns);
//   REGISTER_PROTO_MESSAGE_TYPE(kChassisTopic, hozon::soc::Chassis);
//   REGISTER_PROTO_MESSAGE_TYPE(kGnssTopic, hozon::soc::gnss::GnssInfo);
//   REGISTER_PROTO_MESSAGE_TYPE(kPerceptionTopic,
//                               hozon::perception::TransportElement);
//   REGISTER_PROTO_MESSAGE_TYPE(kRunningModeTopic,
//                               hozon::perception::common_onboard::running_mode);
//   //  REGISTER_PROTO_MESSAGE_TYPE(kStOutputTopic,
//   //                              hozon::localization::Localization);
// }

// int32_t STLocation::OnChassis(Bundle* input) {
//   static double last_chassis_time = -1.0;
//   auto ptr_rec_chassis = input->GetOne(kChassisTopic);

//   auto dr_fault = hozon::perception::lib::FaultManager::Instance();
//   static bool chassis_input_data_error_flag = false;
//   static bool chassis_input_time_error_flag = false;
//   static bool chassis_input_data_nan_flag = false;
//   if (!ptr_rec_chassis) {
//     dr_fault->Report(MAKE_FM_TUPLE(
//         hozon::perception::base::FmModuleId::MAPPING,
//         hozon::perception::base::FaultType::CHASSIS_INPUT_SIGNAL_ERROR_MUL_FPS,
//         hozon::perception::base::FaultStatus::OCCUR,
//         hozon::perception::base::SensorOrientation::UNKNOWN, 3, 50));
//     chassis_input_data_error_flag = true;
//     HLOG_ERROR << "DR: receive chassis is null";
//     return -1;
//   } else {
//     if (chassis_input_data_error_flag) {
//       dr_fault->Report(MAKE_FM_TUPLE(
//           hozon::perception::base::FmModuleId::MAPPING,
//           hozon::perception::base::FaultType::
//               CHASSIS_INPUT_SIGNAL_ERROR_MUL_FPS,
//           hozon::perception::base::FaultStatus::RESET,
//           hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
//       chassis_input_data_error_flag = false;
//     }
//   }
//   std::shared_ptr<hozon::soc::Chassis> chassis_proto =
//       std::static_pointer_cast<hozon::soc::Chassis>(ptr_rec_chassis->proto_msg);
//   double cur_chassis_time =
//       chassis_proto->header().sensor_stamp().chassis_stamp();
//   if (last_chassis_time > 0) {
//     if (cur_chassis_time - last_chassis_time < 0) {
//       dr_fault->Report(MAKE_FM_TUPLE(
//           hozon::perception::base::FmModuleId::MAPPING,
//           hozon::perception::base::FaultType::CHASSIS_INPUT_TIME_ERROR_MUL_FPS,
//           hozon::perception::base::FaultStatus::OCCUR,
//           hozon::perception::base::SensorOrientation::UNKNOWN, 3, 50));
//       chassis_input_time_error_flag = true;
//       HLOG_ERROR << "DR: receive chassis data stamp is error";
//     } else {
//       if (chassis_input_time_error_flag) {
//         dr_fault->Report(MAKE_FM_TUPLE(
//             hozon::perception::base::FmModuleId::MAPPING,
//             hozon::perception::base::FaultType::
//                 CHASSIS_INPUT_TIME_ERROR_MUL_FPS,
//             hozon::perception::base::FaultStatus::RESET,
//             hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
//         chassis_input_time_error_flag = false;
//       }
//     }
//     if (cur_chassis_time - last_chassis_time >= 0.03) {
//       dr_fault->Report(MAKE_FM_TUPLE(
//           hozon::perception::base::FmModuleId::MAPPING,
//           hozon::perception::base::FaultType::CHASSIS_INPUT_TIME_ERROR_MUL_FPS,
//           hozon::perception::base::FaultStatus::OCCUR,
//           hozon::perception::base::SensorOrientation::UNKNOWN, 3, 50));
//       chassis_input_time_error_flag = true;
//       HLOG_ERROR << "DR: receive chassis data stamp is delay";
//     } else {
//       if (chassis_input_time_error_flag) {
//         dr_fault->Report(MAKE_FM_TUPLE(
//             hozon::perception::base::FmModuleId::MAPPING,
//             hozon::perception::base::FaultType::
//                 CHASSIS_INPUT_TIME_ERROR_MUL_FPS,
//             hozon::perception::base::FaultStatus::RESET,
//             hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
//         chassis_input_time_error_flag = false;
//       }
//     }
//   }
//   double count_rl = chassis_proto->wheel_counter().wheel_counter_rl();
//   double count_rr = chassis_proto->wheel_counter().wheel_counter_rr();
//   if (std::isnan(count_rl) || std::isnan(count_rr)) {
//     dr_fault->Report(MAKE_FM_TUPLE(
//         hozon::perception::base::FmModuleId::MAPPING,
//         hozon::perception::base::FaultType::CHASSIS_INPUT_VALUE_ERROR_MUL_FPS,
//         hozon::perception::base::FaultStatus::OCCUR,
//         hozon::perception::base::SensorOrientation::UNKNOWN, 3, 50));
//     chassis_input_data_nan_flag = true;
//     HLOG_ERROR << "DR: receive chassis data Nan";
//   } else {
//     if (chassis_input_data_nan_flag) {
//       dr_fault->Report(MAKE_FM_TUPLE(
//           hozon::perception::base::FmModuleId::MAPPING,
//           hozon::perception::base::FaultType::CHASSIS_INPUT_VALUE_ERROR_MUL_FPS,
//           hozon::perception::base::FaultStatus::RESET,
//           hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
//       chassis_input_data_nan_flag = false;
//     }
//   }
//   //===========
//   //==st内部只用了四轮轮速，车速用于数据检查，m/s
//   senseAD::localization::VehicleInfo vehicle_info;
//   // todo:时间戳单位不确定，需要纳秒
//   uint64_t nano_sec =
//       chassis_proto->header().data_stamp();  // todo:proto为s，需要改为ns
//   // 档位信息(0-7:默认,park,reverse,neutral,drive,s,invalid)
//   int gear_type = chassis_proto->gear_location();
//   switch (gear_type) {
//     case 0:
//       vehicle_info.gear = senseAD::localization::VehicleInfo::GEAR_NONE;
//       break;
//     case 1:
//       vehicle_info.gear = senseAD::localization::VehicleInfo::GEAR_PARK;
//       break;
//     case 2:
//       vehicle_info.gear = senseAD::localization::VehicleInfo::GEAR_REVERSE;
//       break;
//     case 3:
//       vehicle_info.gear = senseAD::localization::VehicleInfo::GEAR_NEUTRAL;
//       break;
//     case 4:
//       vehicle_info.gear = senseAD::localization::VehicleInfo::GEAR_DRIVE;
//       break;
//     case 5:
//       // todo:此处有问题，s档和low的含义分别是什么？
//       vehicle_info.gear = senseAD::localization::VehicleInfo::
//           GEAR_LOW;  // st不用档位信息，由外部给轮速正负
//       break;
//     default:
//       vehicle_info.gear = senseAD::localization::VehicleInfo::GEAR_NONE;
//       break;
//   }
//   // todo:好像没有turn_signal
//   if (chassis_proto->left_turn_signal() == true) {
//     vehicle_info.turn_signal =
//         senseAD::localization::VehicleInfo::TURN_SIGNAL_LEFT;
//   } else if (chassis_proto->right_turn_signal() == true) {
//     vehicle_info.turn_signal =
//         senseAD::localization::VehicleInfo::TURN_SIGNAL_RIGHT;
//   } else {
//     vehicle_info.turn_signal =
//         senseAD::localization::VehicleInfo::TURN_SIGNAL_NONE;
//   }

//   vehicle_info.throttle = chassis_proto->throttle_percentage();
//   vehicle_info.brake = chassis_proto->brake_percentage();
//   vehicle_info.steering_angle =
//       chassis_proto->steering_angle();  // todo:确认单位；不用
//   vehicle_info.steering_torque = chassis_proto->steering_torque_nm();

//   vehicle_info.vehicle_speed = chassis_proto->speed_mps();
//   vehicle_info.wheel_speed_fl = chassis_proto->wheel_speed().wheel_spd_fl();
//   vehicle_info.wheel_speed_fr = chassis_proto->wheel_speed().wheel_spd_fr();
//   vehicle_info.wheel_speed_rl = chassis_proto->wheel_speed().wheel_spd_rl();
//   vehicle_info.wheel_speed_rr = chassis_proto->wheel_speed().wheel_spd_rr();
//   // no vehicle speed in pilot mdc vehicle, so mock it
//   vehicle_info.vehicle_speed =
//       0.5 * (vehicle_info.wheel_speed_rr + vehicle_info.wheel_speed_rl);
//   // modify wheel speed when reverse//todo:下面655.35这段还需要吗;不需要，废除
//   if (vehicle_info.wheel_speed_fl > 100)
//     vehicle_info.wheel_speed_fl -= 655.35 * 0.335;
//   if (vehicle_info.wheel_speed_fr > 100)
//     vehicle_info.wheel_speed_fr -= 655.35 * 0.335;
//   if (vehicle_info.wheel_speed_rl > 100)
//     vehicle_info.wheel_speed_rl -= 655.35 * 0.335;
//   if (vehicle_info.wheel_speed_rr > 100)
//     vehicle_info.wheel_speed_rr -= 655.35 * 0.335;

//   vehicle_info.acceleration_lat = chassis_proto->imu_acc().y();  // m/s2
//   vehicle_info.acceleration_long = chassis_proto->imu_acc().x();
//   vehicle_info.acceleration_vert = chassis_proto->imu_acc().z();
//   vehicle_info.angular_roll_rate = chassis_proto->imu_ang_rate().x();  // rad
//   vehicle_info.angular_yaw_rate = chassis_proto->imu_ang_rate().z();
//   // todo:好像没有以下几项；不用
//   //  vehicle_info.suspension_fl = msg.getChassis().getSuspensionFl();
//   //  vehicle_info.suspension_fr = msg.getChassis().getSuspensionFr();
//   //  vehicle_info.suspension_rl = msg.getChassis().getSuspensionRl();
//   //  vehicle_info.suspension_rr = msg.getChassis().getSuspensionRr();
//   //  vehicle_info.tire_pressure_fl = msg.getChassis().getTirePressureFl();
//   //  vehicle_info.tire_pressure_fr = msg.getChassis().getTirePressureFr();
//   //  vehicle_info.tire_pressure_rl = msg.getChassis().getTirePressureRl();
//   //  vehicle_info.tire_pressure_rr = msg.getChassis().getTirePressureRr();
//   // todo:有Fuel range in meters但是没有fuel_level；不用
//   //  vehicle_info.fuel_level = msg.getChassis().getFuelLevel();
//   localization_manager_->SetCanData(nano_sec, vehicle_info);
//   return 0;
// }

// int32_t STLocation::OnImuIns(Bundle* input) {
//   static double last_imu_time = -1.0;

//   if (!input) {
//     return -1;
//   }
//   auto ptr_rec_imu = input->GetOne(kImuInsTopic);
//   // diagnose
//   auto dr_fault = hozon::perception::lib::FaultManager::Instance();
//   auto dr_health = hozon::perception::lib::HealthManager::Instance();
//   static bool input_data_error_flag = false;
//   static bool input_time_error_flag = false;
//   static bool input_data_nan_flag = false;
//   static bool output_data_nan_flag = false;

//   dr_health->HealthReport(
//       MAKE_HM_TUPLE(hozon::perception::base::HmModuleId::MAPPING,
//                     hozon::perception::base::HealthId::
//                         CPID_IMU_FPS_AFTER_DETECT_DATA_ABSTRACT));
//   if (!ptr_rec_imu) {
//     dr_fault->Report(MAKE_FM_TUPLE(
//         hozon::perception::base::FmModuleId::MAPPING,
//         hozon::perception::base::FaultType::IMU_DATA_ERROR_MUL_FPS,
//         hozon::perception::base::FaultStatus::OCCUR,
//         hozon::perception::base::SensorOrientation::UNKNOWN, 3, 50));
//     input_data_error_flag = true;
//     HLOG_ERROR << "DR: receive imu data is null";
//     return -1;
//   } else {
//     if (input_data_error_flag) {
//       dr_fault->Report(MAKE_FM_TUPLE(
//           hozon::perception::base::FmModuleId::MAPPING,
//           hozon::perception::base::FaultType::IMU_DATA_ERROR_MUL_FPS,
//           hozon::perception::base::FaultStatus::RESET,
//           hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
//       input_data_error_flag = false;
//     }
//   }

//   std::shared_ptr<hozon::soc::ImuIns> imu_proto =
//       std::static_pointer_cast<hozon::soc::ImuIns>(ptr_rec_imu->proto_msg);
//   double cur_imu_time = imu_proto->header().sensor_stamp().imuins_stamp();
//   if (last_imu_time > 0) {
//     if (cur_imu_time - last_imu_time < 0) {
//       dr_fault->Report(MAKE_FM_TUPLE(
//           hozon::perception::base::FmModuleId::MAPPING,
//           hozon::perception::base::FaultType::IMU_DATA_TIME_ERROR_MUL_FPS,
//           hozon::perception::base::FaultStatus::OCCUR,
//           hozon::perception::base::SensorOrientation::UNKNOWN, 3, 50));
//       input_time_error_flag = true;
//       HLOG_ERROR << "DR: receive imu data stamp is error";
//     } else {
//       if (input_time_error_flag) {
//         dr_fault->Report(MAKE_FM_TUPLE(
//             hozon::perception::base::FmModuleId::MAPPING,
//             hozon::perception::base::FaultType::IMU_DATA_TIME_ERROR_MUL_FPS,
//             hozon::perception::base::FaultStatus::RESET,
//             hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
//         input_time_error_flag = false;
//       }
//     }

//     if (cur_imu_time - last_imu_time >= 0.03) {
//       dr_fault->Report(MAKE_FM_TUPLE(
//           hozon::perception::base::FmModuleId::MAPPING,
//           hozon::perception::base::FaultType::IMU_DATA_TIME_ERROR_MUL_FPS,
//           hozon::perception::base::FaultStatus::OCCUR,
//           hozon::perception::base::SensorOrientation::UNKNOWN, 3, 50));
//       input_time_error_flag = true;
//       HLOG_ERROR << "DR: receive imu data stamp is delay";
//     } else {
//       if (input_time_error_flag) {
//         dr_fault->Report(MAKE_FM_TUPLE(
//             hozon::perception::base::FmModuleId::MAPPING,
//             hozon::perception::base::FaultType::IMU_DATA_TIME_ERROR_MUL_FPS,
//             hozon::perception::base::FaultStatus::RESET,
//             hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
//         input_time_error_flag = false;
//       }
//     }
//   }
//   //=============================
//   senseAD::localization::Imu raw_imu;
//   raw_imu.measurement_time = imu_proto->header().data_stamp();  // todo:proto为s
//   raw_imu.angular_velocity.x = imu_proto->imu_info().angular_velocity().x();
//   raw_imu.angular_velocity.y = imu_proto->imu_info().angular_velocity().y();
//   raw_imu.angular_velocity.z = imu_proto->imu_info().angular_velocity().z();
//   raw_imu.linear_acceleration.x =
//       imu_proto->imu_info().linear_acceleration().x();
//   raw_imu.linear_acceleration.y =
//       imu_proto->imu_info().linear_acceleration().y();
//   raw_imu.linear_acceleration.z =
//       imu_proto->imu_info().linear_acceleration().z();
//   //  raw_imu.status =
//   //  imu_proto->imu_info().imu_status();//todo:status怎么判定?当前无状态，st全当good
//   raw_imu.status = senseAD::localization::ImuStatus::GOOD;
//   uint64_t corrected_imu_time =
//       static_cast<uint64_t>(raw_imu.measurement_time * 1e9);
//   // fill input data
//   localization_manager_->SetImuData(corrected_imu_time, raw_imu);

//   return 0;
// }
// int32_t STLocation::OnGnss(Bundle* input) {
//   static double last_gnss_time = -1.0;
//   auto ptr_rec_gnss = input->GetOne(kGnssTopic);
//   // diagnose
//   auto phm_fault = hozon::perception::lib::FaultManager::Instance();
//   if (!ptr_rec_gnss) {
//     phm_fault->Report(MAKE_FM_TUPLE(
//         hozon::perception::base::FmModuleId::MAPPING,
//         hozon::perception::base::FaultType::GNSS_DATA_ERROR_MUL_FPS,
//         hozon::perception::base::FaultStatus::OCCUR,
//         hozon::perception::base::SensorOrientation::UNKNOWN, 3, 500));
//     HLOG_ERROR << "INS: receive gnss is null";
//     return -1;
//   } else {
//     phm_fault->Report(MAKE_FM_TUPLE(
//         hozon::perception::base::FmModuleId::MAPPING,
//         hozon::perception::base::FaultType::GNSS_DATA_ERROR_MUL_FPS,
//         hozon::perception::base::FaultStatus::RESET,
//         hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
//   }
//   std::shared_ptr<hozon::soc::gnss::GnssInfo> gnss_proto =
//       std::static_pointer_cast<hozon::soc::gnss::GnssInfo>(
//           ptr_rec_gnss->proto_msg);
//   if (!gnss_proto) {
//     return -1;
//   }
//   double cur_gnss_time = gnss_proto->header().sensor_stamp().gnss_stamp();
//   if (last_gnss_time > 0) {
//     if (cur_gnss_time - last_gnss_time < 0) {
//       phm_fault->Report(MAKE_FM_TUPLE(
//           hozon::perception::base::FmModuleId::MAPPING,
//           hozon::perception::base::FaultType::GNSS_DATA_TIME_ERROR_MUL_FPS,
//           hozon::perception::base::FaultStatus::OCCUR,
//           hozon::perception::base::SensorOrientation::UNKNOWN, 3, 500));
//       HLOG_ERROR << "INS: receive gnss data stamp is error";
//     } else {
//       phm_fault->Report(MAKE_FM_TUPLE(
//           hozon::perception::base::FmModuleId::MAPPING,
//           hozon::perception::base::FaultType::GNSS_DATA_TIME_ERROR_MUL_FPS,
//           hozon::perception::base::FaultStatus::RESET,
//           hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
//     }
//     if (cur_gnss_time - last_gnss_time >= 0.3) {
//       phm_fault->Report(MAKE_FM_TUPLE(
//           hozon::perception::base::FmModuleId::MAPPING,
//           hozon::perception::base::FaultType::GNSS_DATA_TIME_ERROR_MUL_FPS,
//           hozon::perception::base::FaultStatus::OCCUR,
//           hozon::perception::base::SensorOrientation::UNKNOWN, 3, 500));
//       HLOG_ERROR << "INS: receive gnss data stamp is delay";
//     } else {
//       phm_fault->Report(MAKE_FM_TUPLE(
//           hozon::perception::base::FmModuleId::MAPPING,
//           hozon::perception::base::FaultType::GNSS_DATA_TIME_ERROR_MUL_FPS,
//           hozon::perception::base::FaultStatus::RESET,
//           hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
//     }
//   }
//   double longi = gnss_proto->gnss_pos().longitude();
//   double lati = gnss_proto->gnss_pos().latitude();
//   if (std::isnan(longi) || std::isnan(lati)) {
//     phm_fault->Report(MAKE_FM_TUPLE(
//         hozon::perception::base::FmModuleId::MAPPING,
//         hozon::perception::base::FaultType::GNSS_DATA_VALUE_ERROR_MUL_FPS,
//         hozon::perception::base::FaultStatus::OCCUR,
//         hozon::perception::base::SensorOrientation::UNKNOWN, 3, 500));
//     HLOG_ERROR << "GNSS: receive gnss data Nan";
//   } else {
//     phm_fault->Report(MAKE_FM_TUPLE(
//         hozon::perception::base::FmModuleId::MAPPING,
//         hozon::perception::base::FaultType::GNSS_DATA_VALUE_ERROR_MUL_FPS,
//         hozon::perception::base::FaultStatus::RESET,
//         hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
//   }
//   last_gnss_time = cur_gnss_time;
//   double diff_age = gnss_proto->gnss_pos().diff_age();
//   if (std::isnan(diff_age)) {
//     phm_fault->Report(MAKE_FM_TUPLE(
//         hozon::perception::base::FmModuleId::MAPPING,
//         hozon::perception::base::FaultType::
//             CORRECTION_SERVICE_ABNORMAL_FOR_LONG_PERIOD,
//         hozon::perception::base::FaultStatus::OCCUR,
//         hozon::perception::base::SensorOrientation::UNKNOWN, 3, 500));
//     HLOG_ERROR << "GNSS: receive diff age Nan";
//   } else {
//     phm_fault->Report(MAKE_FM_TUPLE(
//         hozon::perception::base::FmModuleId::MAPPING,
//         hozon::perception::base::FaultType::
//             CORRECTION_SERVICE_ABNORMAL_FOR_LONG_PERIOD,
//         hozon::perception::base::FaultStatus::RESET,
//         hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
//   }
//   //===========================================
//   senseAD::localization::Gnss raw_gnss;
//   raw_gnss.measurement_time = gnss_proto->header().sensor_stamp().gnss_stamp();
//   raw_gnss.linear_velocity.x = gnss_proto->gnss_vel().vel_x();
//   raw_gnss.linear_velocity.y = gnss_proto->gnss_vel().vel_y();
//   raw_gnss.linear_velocity.z = gnss_proto->gnss_vel().vel_z();
//   raw_gnss.linear_velocity_std_dev.x = gnss_proto->gnss_vel().vel_x_std();
//   raw_gnss.linear_velocity_std_dev.y = gnss_proto->gnss_vel().vel_y_std();
//   raw_gnss.linear_velocity_std_dev.z = gnss_proto->gnss_vel().vel_z_std();
//   raw_gnss.position.lon = gnss_proto->gnss_pos().longitude();
//   raw_gnss.position.lat = gnss_proto->gnss_pos().latitude();
//   raw_gnss.position.height = gnss_proto->gnss_pos().altitude();
//   // todo:不确认lon，lat,hight和x标准差是否对应；st是东北天顺序，需要把经纬高转成东北
//   // 天，在msf_factor_imu_gnss.hpp中，
//   // Tpr_是东北天转经纬高(弧度转m)的雅可比
//   raw_gnss.position_std_dev.x = gnss_proto->gnss_pos().lon_std();
//   raw_gnss.position_std_dev.y = gnss_proto->gnss_pos().lat_std();
//   raw_gnss.position_std_dev.z = gnss_proto->gnss_pos().hgt_std();
//   //  raw_gnss.status = gnss_proto->gnss_pos().pos_type();//todo:不确定pos映射
//   // todo:为了和加偏地图对齐，要么使用加偏topic,要么自己加偏；找彭伟把gnss数据也加偏
//   //  if (param_.common_param.use_ehr_map == true) {
//   //    gcj02_converter_.wgs84toGCJ02(msg.getLatitude(), msg.getLongitude(),
//   //                                  &raw_gnss.position.lat,
//   //                                  &raw_gnss.position.lon);
//   //  }

//   uint64_t corrected_gnss_time = static_cast<uint64_t>(
//       gnss_proto->header().sensor_stamp().gnss_stamp() * 1e9);
//   //  if (ins_use_device_time_) {
//   //    corrected_gnss_time =
//   //        static_cast<uint64_t>(raw_gnss.measurement_time * 1e9);
//   //  } else if (ins_using_utc_time_) {
//   //    corrected_gnss_time =
//   //        static_cast<uint64_t>(raw_gnss.measurement_time * 1e9) +
//   //        system_time_offset_;
//   //  }

//   //  if (param_.common_param.ins_device == "CHNAV") {
//   //    // raw value of CHNAV's pos std is too small
//   //    // increase observation noise
//   //    raw_gnss.position_std_dev.x = 2.0 * msg.getPositionStdDev().getX();
//   //    raw_gnss.position_std_dev.y = 2.0 * msg.getPositionStdDev().getY();
//   //    raw_gnss.position_std_dev.z = 2.0 * msg.getPositionStdDev().getZ();
//   //    // Status of CHNAV is different from NovAtel/BYNAV(SenseAuto baseline)
//   //    // TODO(dujiankui): Unify input in sensor processing layer
//   //    raw_gnss.status = senseAD::localization::GnssStatus::RTK_INTEGER;
//   //  } else if (param_.common_param.ins_device == "HIRAIN" ||
//   //             param_.common_param.ins_device == "ASENSING") {
//   //    double heading = msg.getLinearVelocity().getY() * M_PI / 180;
//   //    double horizon_v = msg.getLinearVelocity().getX();
//   //    raw_gnss.linear_velocity.x = std::sin(heading) * horizon_v;
//   //    raw_gnss.linear_velocity.y = std::cos(heading) * horizon_v;
//   //    raw_gnss.linear_velocity_std_dev.x = 0.02;
//   //    raw_gnss.linear_velocity_std_dev.y = 0.02;
//   //    raw_gnss.linear_velocity_std_dev.z = 0.1;
//   //    if (raw_gnss.status == senseAD::localization::GnssStatus::RTK_INTEGER) {
//   //      raw_gnss.position_std_dev.x = 0.4 * msg.getPositionStdDev().getX();
//   //      raw_gnss.position_std_dev.y = 0.4 * msg.getPositionStdDev().getY();
//   //      raw_gnss.position_std_dev.z = 0.4 * msg.getPositionStdDev().getZ();
//   //    } else if (raw_gnss.status ==
//   //               senseAD::localization::GnssStatus::RTK_FLOAT) {
//   //      raw_gnss.position_std_dev.x =
//   //          std::max(msg.getPositionStdDev().getX(), 1.0);
//   //      raw_gnss.position_std_dev.y =
//   //          std::max(msg.getPositionStdDev().getY(), 1.0);
//   //      raw_gnss.position_std_dev.z =
//   //          std::max(msg.getPositionStdDev().getZ(), 1.0);
//   //    }
//   //  }

//   // fill input data
//   localization_manager_->SetGnssData(corrected_gnss_time, raw_gnss);
//   return 0;
// }

// int32_t STLocation::OnPerception(Bundle* input) {
//   if (!input) {
//     return -1;
//   }
//   static double last_percep_time = -1.0;
//   auto phm_fault = hozon::perception::lib::FaultManager::Instance();
//   auto p_perception = input->GetOne(kPerceptionTopic);
//   if (p_perception == nullptr) {
//     phm_fault->Report(MAKE_FM_TUPLE(
//         hozon::perception::base::FmModuleId::MAPPING,
//         hozon::perception::base::FaultType::
//             MULTI_FRAME_PERCEPTION_INPUT_DATA_LOSS,
//         hozon::perception::base::FaultStatus::OCCUR,
//         hozon::perception::base::SensorOrientation::UNKNOWN, 3, 500));
//     HLOG_ERROR << "Location:perception input data loss";
//     return -1;
//   }
//   phm_fault->Report(
//       MAKE_FM_TUPLE(hozon::perception::base::FmModuleId::MAPPING,
//                     hozon::perception::base::FaultType::
//                         MULTI_FRAME_PERCEPTION_INPUT_DATA_LOSS,
//                     hozon::perception::base::FaultStatus::RESET,
//                     hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));

//   const auto percep_proto =
//       std::static_pointer_cast<hozon::perception::TransportElement>(
//           p_perception->proto_msg);
//   if (!percep_proto) {
//     return -1;
//   }
//   double cur_percep_time = percep_proto->header().data_stamp();
//   if (last_percep_time > 0) {
//     if (last_percep_time - cur_percep_time > 0) {
//       phm_fault->Report(MAKE_FM_TUPLE(
//           hozon::perception::base::FmModuleId::MAPPING,
//           hozon::perception::base::FaultType::
//               MULTI_FRAME_PERCEPTION_INPUT_TIME_ERROR,
//           hozon::perception::base::FaultStatus::OCCUR,
//           hozon::perception::base::SensorOrientation::UNKNOWN, 3, 500));
//       HLOG_ERROR << "Location:receieve perception time error";
//     } else {
//       phm_fault->Report(MAKE_FM_TUPLE(
//           hozon::perception::base::FmModuleId::MAPPING,
//           hozon::perception::base::FaultType::
//               MULTI_FRAME_PERCEPTION_INPUT_TIME_ERROR,
//           hozon::perception::base::FaultStatus::RESET,
//           hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
//     }
//   }
//   last_percep_time = cur_percep_time;
//   //==========================================
//   senseAD::localization::PerceptData percep_data;
//   // senmantic:traffic_signs
//   for (auto& one_sign :
//        percep_proto
//            ->traffic_signs()) {  // todo:合众原始感知有路牌数据，但是没有跟踪
//     int id =
//         0;  // todo:st算法多帧默认跟踪id准确,没有跟踪id，需要hz补上;单帧不需要
//     senseAD::localization::PerceptTrafficSign& st_traffic_sign =
//         percep_data.traffic_signs[id];
//     st_traffic_sign.confidence = one_sign.confidence();
//     double width = static_cast<double>(one_sign.bbox2d().size().x());
//     double hight = static_cast<double>(one_sign.bbox2d().size().y());
//     double x = static_cast<double>(one_sign.bbox2d().topleft().x());
//     double y = static_cast<double>(one_sign.bbox2d().topleft().y());
//     st_traffic_sign.rect.center =
//         senseAD::localization::Point2D_t((x + width) / 2, (y + hight) / 2);
//     st_traffic_sign.rect.length = hight;
//     st_traffic_sign.rect.width = width;
//     // todo:st路牌类型和合众路牌类型不同，无法对应；st只用trackid做数据关联，不用类型
//     //    int type = static_cast<int>(one_sign.type());
//     //    switch (type) {
//     //      case 1://禁止行驶方向
//     //        st_traffic_sign.type =
//     //            senseAD::localization::TrafficSignType::Others;
//     //        break;
//     //      case 2://禁止行驶行为
//     //        st_traffic_sign.type =
//     //            senseAD::localization::TrafficSignType::Others;
//     //        break;
//     //      case 3://限速
//     //        st_traffic_sign.type =
//     //            senseAD::localization::TrafficSignType::MaxSpeedLimit;
//     //        break;
//     //      case 4://取消限速
//     //        st_traffic_sign.type =
//     //            senseAD::localization::TrafficSignType::AllSpeedLimitCancel;
//     //        break;
//     //      default:
//     //        st_traffic_sign.type =
//     //        senseAD::localization::TrafficSignType::Others; break;
//     //    }
//   }
//   // senmantic:light_poles
//   for (auto& one_pole :
//        percep_proto
//            ->light_poles()) {  // todo:合众原始感知没有灯杆数据；需要提需求
//     int id = 0;                // todo:没有跟踪id需要补上;
//     senseAD::localization::PerceptPole& st_traffic_pole = percep_data.poles[id];
//     st_traffic_pole.confidence =
//         1.0;  // todo:合众的pole没有confidence；st的confidence也不用
//     st_traffic_pole.type = senseAD::localization::PoleType::Unknown;
//     st_traffic_pole.processed_center.x = one_pole.points_3d().x();
//     st_traffic_pole.processed_center.y = one_pole.points_3d().y();
//     st_traffic_pole.processed_center.z = one_pole.points_3d().z();
//     // todo:st的pole没有up_pt和down_pt字段，但是合众有；st的tracking方案需要3d点，单帧方案2d框即可
//   }

//   // geometry
//   for (auto& one_line : percep_proto->lane()) {
//     auto id =
//         one_line
//             .track_id();  // todo:合众的track_id单帧唯一，但是逻辑为++,不判定车道号；st需要跟踪过的id
//     auto& lane_line = percep_data.lane_lines[id];
//     lane_line.confidence = 1.0;
//     for (auto& param : one_line.lane_param().cubic_curve_set()) {
//       lane_line.start_point.x = param.start_point_x();
//       lane_line.start_point
//           .y;  // todo:合众没有y；内部没有使用y,算力够就自己用三次曲线算一下y
//       lane_line.end_point.x = param.end_point_x();
//       lane_line.end_point.y;  // todo:合众没有y
//       lane_line.poly_coef.emplace_back(
//           param.c3());  // todo:st的poly_coef顺序是c3,c2,c1,c0倒着来
//       lane_line.poly_coef.emplace_back(param.c2());
//       lane_line.poly_coef.emplace_back(param.c1());
//       lane_line.poly_coef.emplace_back(param.c0());
//     }
//      // st内部如果用图像就用img_pts字段，如果用bev就用三次曲线计算点
//     for (auto& pt : one_line.points()) {
//       senseAD::localization::Point2D_t st_pt{pt.x(), pt.y()};
//       lane_line.img_pts.emplace_back(
//           st_pt);
//     }
//     int color = static_cast<int>(one_line.color());
//     switch (color) {
//       case 0:
//         lane_line.color = senseAD::localization::Color::Other;
//         break;
//       case 1:
//         lane_line.color = senseAD::localization::Color::White;
//         break;
//       case 2:
//         lane_line.color = senseAD::localization::Color::Yellow;
//         break;
//       case 3:
//         lane_line.color = senseAD::localization::Color::Green;
//         break;
//       default:
//         lane_line.color = senseAD::localization::Color::
//             Other;  // todo:st中没有RED,BLCAK颜色;hz是为了兼容其他模块，可以不管
//         break;
//     }

//     lane_line.line_type = senseAD::localization::LineType::LaneMarking;
//     int line_type = static_cast<int>(one_line.lanetype());
//     switch (line_type) {
//       case 0:
//         lane_line.line_type = senseAD::localization::LineType::Unknown;
//         lane_line.line_style = senseAD::localization::LineStyle::Unknown;
//         break;
//       case 1:
//         lane_line.line_style = senseAD::localization::LineStyle::SingleSolid;
//         break;
//       case 2:
//         lane_line.line_style = senseAD::localization::LineStyle::SingleDashed;
//         break;
//       case 3:
//         lane_line.line_style = senseAD::localization::LineStyle::ShortDashed;
//         break;
//       case 4:
//         lane_line.line_style = senseAD::localization::LineStyle::DoubleSolid;
//         break;
//       case 5:
//         lane_line.line_style = senseAD::localization::LineStyle::DoubleDashed;
//         break;
//       case 6:
//         lane_line.line_style =
//             senseAD::localization::LineStyle::LeftSolidRightDashed;
//         break;
//       case 7:
//         lane_line.line_style =
//             senseAD::localization::LineStyle::RightSolidLeftDashed;
//         break;
//       case 8:
//         lane_line.line_style = senseAD::localization::LineStyle::ShadedArea;
//         break;
//       case 9:
//         lane_line.line_style = senseAD::localization::LineStyle::
//             LaneVirtualMarking;  // todo:和road虚拟线的区别；st内部只区分curb和marking，以及虚线和实线
//         break;
//       case 10:
//         lane_line.line_style =
//             senseAD::localization::LineStyle::IntersectionVirualMarking;
//         break;
//       case 11:
//         lane_line.line_style =
//             senseAD::localization::LineStyle::CurbVirtualMarking;
//         break;
//       case 12:
//         lane_line.line_style = senseAD::localization::LineStyle::UnclosedRoad;
//         break;
//       case 13:
//         lane_line.line_style =
//             senseAD::localization::LineStyle::RoadVirtualLine;
//         break;
//       case 14:
//         lane_line.line_style = senseAD::localization::LineStyle::
//             Other;  // todo:st没有LaneChangeVirtualLine
//         break;
//       case 15:
//         lane_line.line_style = senseAD::localization::LineStyle::FishBone;
//         break;
//       default:
//         lane_line.line_style = senseAD::localization::LineStyle::Other;
//         break;
//     }
//   }
//   // curb
//   for (auto& one_curb : percep_proto->road_edges()) {
//     auto id = one_curb.id();
//     auto& curb_line =
//         percep_data.lane_lines
//             [id];  // todo:合众的路沿id和车道线id会重复,如何处理；st内部数据关联使用，不能重复
//     curb_line.confidence = 1.0;

//     curb_line.start_point.x = one_curb.vehicle_curve().start_point_x();
//     curb_line.start_point.y;
//     curb_line.end_point.x = one_curb.vehicle_curve().end_point_x();
//     curb_line.end_point.y;
//     curb_line.poly_coef.emplace_back(one_curb.vehicle_curve().c0());
//     curb_line.poly_coef.emplace_back(one_curb.vehicle_curve().c1());
//     curb_line.poly_coef.emplace_back(one_curb.vehicle_curve().c2());
//     curb_line.poly_coef.emplace_back(one_curb.vehicle_curve().c3());
//     for (auto& pt : one_curb.points()) {
//       senseAD::localization::Point2D_t st_pt{pt.x(), pt.y()};
//       curb_line.img_pts.emplace_back(st_pt);
//     }

//     curb_line.line_style = senseAD::localization::LineStyle::Other;
//     curb_line.color = senseAD::localization::Color::Other;
//     int curb_type = static_cast<int>(one_curb.type());
//     switch (curb_type) {
//       case 0:
//         curb_line.line_type = senseAD::localization::LineType::Curb;
//         break;
//       case 1:
//         curb_line.line_type = senseAD::localization::LineType::
//             Nature;  // todo:地面是nature吗？st感知只给了curb和makring，对于定位其他不重要
//         break;
//       case 2:
//         curb_line.line_type =
//             senseAD::localization::LineType::Other;  // todo:st没有锥桶；不用管
//         break;
//       case 3:
//         curb_line.line_type = senseAD::localization::LineType::
//             Concrete;  // todo:3是水马,st的对吗；不用管
//         break;
//       case 4:
//         curb_line.line_type = senseAD::localization::LineType::Fence;
//         break;
//       default:
//         curb_line.line_style = senseAD::localization::LineStyle::Unknown;
//         break;
//     }
//   }
//   // visual
//   //  percep_data.raw_image;//todo:合众的transport_perception没有原图，需要从别的topic引入："lm_image"
//   senseAD::localization::SyncPerceptData sync_data;
//   auto percept_data_ptr =
//       std::make_shared<senseAD::localization::PerceptData>();
//   percept_data_ptr->traffic_signs = std::move(percep_data.traffic_signs);
//   percept_data_ptr->poles = std::move(percep_data.poles);
//   percept_data_ptr->lane_lines = std::move(percep_data.lane_lines);
//   sync_data.multi_cam_data_.push_back(percept_data_ptr);

//   localization_manager_->SetPerceptData(
//       sync_data.multi_cam_data_[0]->timestamp_ns, sync_data);
//   return 0;
// }

// bool STLocation::GetLocalMapData(
//     const Eigen::Vector3d& vehicle_position,
//     const Eigen::Matrix3d& vehicle_rotation, const Eigen::Vector3d& ref_point,
//     std::shared_ptr<senseAD::localization::RoadStructure>) {
//   // st滤波器状态：其中位置是经纬高，观测是lla转的enu
//   // todo:合众引擎用Utm查地图，但是st传入的是enu定位点，后续要把enu转utm作为地图引擎查询点
//   double x = vehicle_position.x();
//   double y = vehicle_position.y();

//   hozon::common::coordinate_convertor::GCS2UTM(51, &y, &x);
//   hozon::common::PointENU utm_position;
//   utm_position.set_x(y);
//   utm_position.set_y(x);
//   utm_position.set_z(0);
//   // 计算车辆在enu系下的航向角
//   double heading = vehicle_rotation.eulerAngles(2, 1, 0)[0];
//   if (heading > M_PI) {
//     heading -= 2 * M_PI;
//   } else if (heading < -M_PI) {
//     heading += 2 * M_PI;
//   }
//   std::vector<hdmap::LaneInfoConstPtr> hdmap_lanes;
//   GLOBAL_HD_MAP->GetLanesWithHeading(utm_position, 100, heading, M_PI / 4,
//                                      &hdmap_lanes);
//   if (hdmap_lanes.empty()) {
//     HLOG_ERROR << "Get HdMap lanes Failed";
//     return false;
//   }
//   // todo:高精地图有元素，但是杆牌相关接口要和hz要；st的杆牌使用方式是高精地图重投影到image
//   //====================
//   auto SwitchToStMapLine = [&](const hozon::hdmap::LaneBoundary& lane_boundary,
//                                senseAD::localization::LineData& one_line) {
//     for (const auto& curve_segment : lane_boundary.curve().segment()) {
//       senseAD::localization::LineSegmentData one_seg;
//       one_seg.id;  // todo:hz的seg有id且唯一,等浩武截图;
//       one_seg.line_style = senseAD::localization::LineStyle::
//           Unknown;  // todo:hz的seg没有style;st不同的seg类型会链接成不同的线，并且只有重定位用type
//       if (lane_boundary.boundary_type()[0].types()[0] ==
//           hozon::hdmap::LaneBoundaryType::Type::LaneBoundaryType_Type_CURB) {
//         //有实线虚线，curb等属性
//         one_seg.line_type = senseAD::localization::LineType::
//             Curb;  // todo:hz的seg没有type;但是boundary_type有
//       } else {
//         one_seg.line_type = senseAD::localization::LineType::
//             LaneMarking;  // todo:补足其他车道类型;不需要，只用curb和marking
//       }
//       //      curve_segment.line_segment().point();//这个里面装的是utm点，但是不确定是否有数据
//       for (const auto& p : curve_segment.line_segment().original_point()) {
//         // todo:st里面的点是gcj还是enu？st的地图点是经纬度，至于wgs还是gcj只要和定位一致即可
//         senseAD::localization::Point3D_t p_gcj(p.y(), p.x(), 0);
//         one_seg.points.emplace_back(p_gcj);
//       }
//       one_line.line_segments.emplace_back(one_seg);
//     }
//   };
//   latest_local_map_data_->header.timestamp_ns;  // todo:没有地图时间
//   latest_local_map_data_->header.coord_type;  // todo:需要合众地图坐标系//gcj
//   //  latest_local_map_data_->semantic_map_data.lines.resize();//todo:没有地图线数量，无法resize；和hz要

//   for (const auto& lane_ptr :
//        hdmap_lanes) {  // todo:一个lane两条线，相邻车道会重复,需要自己组织去重
//     auto lane = lane_ptr->lane();
//     senseAD::localization::LineData st_line;
//     if (lane.has_extra_left_boundary()) {
//       // todo:此字段为实际存在但是图商会给虚拟线，如果不调用会造成少一条线；st只用感知和地图一致的线
//       //      SwitchToStMapLine(lane.extra_left_boundary());
//     }
//     if (lane.has_extra_right_boundary()) {
//       //      SwitchToStMapLine(lane.extra_right_boundary());
//     }
//     if (lane.has_left_boundary()) {
//       SwitchToStMapLine(lane.left_boundary(), st_line);
//     }
//     if (lane.has_right_boundary()) {
//       SwitchToStMapLine(lane.right_boundary(), st_line);
//     }
//   }

//   return true;
// }

// int32_t STLocation::OnRunningMode(Bundle* input) {
//   auto rm_msg = input->GetOne(kRunningModeTopic);
//   //  if (rm_msg == nullptr) {
//   //    HLOG_ERROR << "nullptr rm_msg plugin";
//   //    return -1;
//   //  }
//   //  auto msg =
//   //      std::static_pointer_cast<hozon::perception::common_onboard::running_mode>(
//   //          rm_msg->proto_msg);
//   //  if (msg == nullptr) {
//   //    HLOG_ERROR << "nullptr rm_msg->proto_msg";
//   //    return -1;
//   //  }
//   //  int runmode = msg->mode();
//   //  if (runmode ==
//   //      static_cast<int>(hozon::perception::base::RunningMode::PARKING)) {
//   //    PauseTrigger("recv_ins_fusion");
//   //    PauseTrigger("recv_perception");
//   //    PauseTrigger("send_pose_estimation_result");
//   //  } else if (runmode == static_cast<int>(
//   //                            hozon::perception::base::RunningMode::DRIVING)
//   //                            ||
//   //             runmode ==
//   //                 static_cast<int>(hozon::perception::base::RunningMode::ALL))
//   //                 {
//   //    ResumeTrigger("recv_ins_fusion");
//   //    ResumeTrigger("recv_perception");
//   //    ResumeTrigger("send_pose_estimation_result");
//   //  }
//   return 0;
// }

// int32_t STLocation::OnLocOutput(Bundle* input) {
//   if (!input) {
//     return -1;
//   }

//   senseAD::localization::NavStateInfo nav_state_info;
//   if (localization_manager_->GetNavStateInfo(&nav_state_info) ==
//       senseAD::localization::LOC_SUCCESS) {
//   } else {
//     HLOG_ERROR << "获取nav结果失败";
//     return -1;
//   }
//   senseAD::localization::OdomStateInfo odom_state_info;
//   if (localization_manager_->GetOdomStateInfo(&odom_state_info) ==
//       senseAD::localization::LOC_SUCCESS) {
//   } else {
//     HLOG_ERROR << "获取odo结果失败";
//   }

//   auto loc_result = std::make_shared<hozon::localization::Localization>();
//   TransStOutput2LocOutput(nav_state_info, odom_state_info, loc_result);
//   auto localization_pack =
//       std::make_shared<hozon::netaos::adf_lite::BaseData>();
//   localization_pack->proto_msg = loc_result;
//   SendOutput(kStOutputTopic, localization_pack);

//   return 0;
// }

// bool STLocation::TransStOutput2LocOutput(
//     senseAD::localization::NavStateInfo& nav_output,
//     senseAD::localization::OdomStateInfo& odo_output,
//     std::shared_ptr<hozon::localization::Localization>& loc_output) {
//   return true;
// }

}  // namespace common_onboard
}  // namespace perception
}  // namespace hozon
