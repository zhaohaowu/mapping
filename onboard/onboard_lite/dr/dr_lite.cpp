/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: Hozon
 *Date: 2023-08-30
 *****************************************************************************/

#include "adf-lite/include/base.h"
#include "base/utils/log.h"
#include "depend/nos/x86_2004/include/adf-lite/include/executor.h"
#include "depend/nos/x86_2004/include/adf/include/node_proto_register.h"
#include "depend/perception-lib/lib/fault_manager/fault_manager.h"
#include "depend/perception-lib/lib/health_manager/health_manager.h"
#include "gtest/gtest.h"
#include "modules/dr/include/dr.h"
#include "modules/util/include/util/orin_trigger_manager.h"
#include "onboard/onboard_lite/map_fusion/map_fusion_config_lite.h"
#include "perception-lib/lib/environment/environment.h"
#include "proto/dead_reckoning/dr.pb.h"
#include "proto/soc/chassis.pb.h"
#include "proto/soc/sensor_imu_ins.pb.h"
#include "yaml-cpp/yaml.h"

namespace hozon {
namespace perception {
namespace common_onboard {

using hozon::netaos::adf_lite::Bundle;

class DeadReckoning : public hozon::netaos::adf_lite::Executor {
 public:
  DeadReckoning() = default;
  ~DeadReckoning() = default;
  int32_t AlgInit() override {
    REGISTER_PROTO_MESSAGE_TYPE("imu_ins", hozon::soc::ImuIns);
    REGISTER_PROTO_MESSAGE_TYPE("chassis", hozon::soc::Chassis);
    REGISTER_PROTO_MESSAGE_TYPE("dr", hozon::dead_reckoning::DeadReckoning);

    std::string default_work_root = "/app/";
    std::string work_root =
        hozon::perception::lib::GetEnv("ADFLITE_ROOT_PATH", default_work_root);
    if (work_root.empty()) {
      HLOG_ERROR << "ENV: ADFLITE_ROOT_PATH is not set.";
      return -1;
    }
    std::string config_file = work_root +
                              "/runtime_service/mapping/conf/mapping/"
                              "dr/dr_config.yaml";
    dr_interface_ = std::make_shared<hozon::mp::dr::DRInterface>(config_file);

    // 接收数据线程
    RegistAlgProcessFunc("receive_chassis",
                         std::bind(&DeadReckoning::receive_chassis, this,
                                   std::placeholders::_1));
    RegistAlgProcessFunc("receive_imu", std::bind(&DeadReckoning::receive_imu,
                                                  this, std::placeholders::_1));

    // 读取速度故障阈值
    YAML::Node config = YAML::LoadFile(config_file);
    vel_error_threshold_ = config["vel_error_threshold"].as<double>();
    HLOG_INFO << "vel_error_threshold_:" << vel_error_threshold_;

    HLOG_INFO << "DR: AlgInit successfully ";
    return 0;
  }

  int32_t receive_chassis(Bundle* input);
  int32_t receive_imu(Bundle* input);

  void AlgRelease() override {}
  void CheckTriggerDrDrift(double vel_x, double p_x, double p_y,
                           double cur_time);
  bool IsDrDrift(double cur_time, double vel_x, double p_x, double p_y);

  template <typename T1, typename T2>
  bool isVelError(const T1& t1, const T2& t2);

 private:
  std::shared_ptr<hozon::mp::dr::DRInterface> dr_interface_ = nullptr;
  std::deque<Eigen::Vector3d> deque_dr_;
  std::deque<Eigen::Vector3d> deque_ins_;
  double vel_error_threshold_;
};

REGISTER_ADF_CLASS(DeadReckoning, DeadReckoning);

// recieve in-process data and interprocess data
int32_t DeadReckoning::receive_chassis(Bundle* input) {
  static double last_chassis_time = -1.0;
  // get one chassis data
  auto ptr_rec_chassis = input->GetOne("chassis");
  auto dr_fault = hozon::perception::lib::FaultManager::Instance();
  static bool chassis_input_data_error_flag = false;
  static bool chassis_input_time_error_flag = false;
  static bool chassis_input_time_error_delay_flag = false;
  static bool chassis_input_data_nan_flag = false;

  static int count_chassis = 0;
  count_chassis++;
  if (count_chassis > 100) {
    count_chassis = 0;
    HLOG_INFO << "receive chassis heartbeat";
  }

  if (!ptr_rec_chassis) {
    dr_fault->ReportDebounceTime(MAKE_TIME_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::CHASSIS_INPUT_SIGNAL_ERROR_MUL_FPS,
        hozon::perception::base::FaultStatus::OCCUR,
        hozon::perception::base::SensorOrientation::UNKNOWN, 400,
        base::DebounceType::DEBOUNCE_TYPE_TIME));
    chassis_input_data_error_flag = true;
    HLOG_ERROR << "DR: receive chassis is null";
    return -1;
  } else {
    if (chassis_input_data_error_flag) {
      dr_fault->ReportDebounceTime(MAKE_TIME_FM_TUPLE(
          hozon::perception::base::FmModuleId::MAPPING,
          hozon::perception::base::FaultType::
              CHASSIS_INPUT_SIGNAL_ERROR_MUL_FPS,
          hozon::perception::base::FaultStatus::RESET,
          hozon::perception::base::SensorOrientation::UNKNOWN, 400,
          base::DebounceType::DEBOUNCE_TYPE_TIME));
      chassis_input_data_error_flag = false;
    }
  }
  std::shared_ptr<hozon::soc::Chassis> chassis_proto =
      std::static_pointer_cast<hozon::soc::Chassis>(ptr_rec_chassis->proto_msg);
  double cur_chassis_time =
      chassis_proto->header().sensor_stamp().chassis_stamp();

  if (last_chassis_time > 0) {
    if (cur_chassis_time - last_chassis_time < 0) {
      dr_fault->Report(MAKE_FM_TUPLE(
          hozon::perception::base::FmModuleId::MAPPING,
          hozon::perception::base::FaultType::CHASSIS_INPUT_TIME_ERROR_MUL_FPS,
          hozon::perception::base::FaultStatus::OCCUR,
          hozon::perception::base::SensorOrientation::UNKNOWN, 6, 100));
      chassis_input_time_error_flag = true;
      HLOG_ERROR << "DR: receive chassis data stamp is error:"
                 << "cur_chassis_time:" << cur_chassis_time << ","
                 << "last_chassis_time:" << last_chassis_time;
    } else {
      if (chassis_input_time_error_flag) {
        dr_fault->Report(MAKE_FM_TUPLE(
            hozon::perception::base::FmModuleId::MAPPING,
            hozon::perception::base::FaultType::
                CHASSIS_INPUT_TIME_ERROR_MUL_FPS,
            hozon::perception::base::FaultStatus::RESET,
            hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
        chassis_input_time_error_flag = false;
      }
    }
    if (cur_chassis_time - last_chassis_time >= 0.03) {
      dr_fault->Report(MAKE_FM_TUPLE(
          hozon::perception::base::FmModuleId::MAPPING,
          hozon::perception::base::FaultType::CHASSIS_INPUT_TIME_ERROR_MUL_FPS,
          hozon::perception::base::FaultStatus::OCCUR,
          hozon::perception::base::SensorOrientation::UNKNOWN, 6, 100));
      chassis_input_time_error_delay_flag = true;
      HLOG_ERROR << "DR: receive chassis data stamp is delay:"
                 << "cur_chassis_time:" << cur_chassis_time << ","
                 << "last_chassis_time:" << last_chassis_time;
    } else {
      if (chassis_input_time_error_delay_flag) {
        dr_fault->Report(MAKE_FM_TUPLE(
            hozon::perception::base::FmModuleId::MAPPING,
            hozon::perception::base::FaultType::
                CHASSIS_INPUT_TIME_ERROR_MUL_FPS,
            hozon::perception::base::FaultStatus::RESET,
            hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
        chassis_input_time_error_delay_flag = false;
      }
    }
  }
  double count_rl = chassis_proto->wheel_counter().wheel_counter_rl();
  double count_rr = chassis_proto->wheel_counter().wheel_counter_rr();
  if (std::isnan(count_rl) || std::isnan(count_rr)) {
    dr_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::CHASSIS_INPUT_VALUE_ERROR_MUL_FPS,
        hozon::perception::base::FaultStatus::OCCUR,
        hozon::perception::base::SensorOrientation::UNKNOWN, 6, 100));
    chassis_input_data_nan_flag = true;
    HLOG_ERROR << "DR: receive chassis data Nan";
  } else {
    if (chassis_input_data_nan_flag) {
      dr_fault->Report(MAKE_FM_TUPLE(
          hozon::perception::base::FmModuleId::MAPPING,
          hozon::perception::base::FaultType::CHASSIS_INPUT_VALUE_ERROR_MUL_FPS,
          hozon::perception::base::FaultStatus::RESET,
          hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
      chassis_input_data_nan_flag = false;
    }
  }
  dr_interface_->AddChassisData(chassis_proto);
  dr_interface_->Process();
  last_chassis_time = cur_chassis_time;
  return 0;
}

bool DeadReckoning::IsDrDrift(double cur_time, double vel_x, double p_x,
                              double p_y) {
  static double d_p = 0.0, d_p_vt = 0.0;
  static std::vector<double> last_po{-1.0, 0.0, 0.0, 0.0};
  std::vector<double> curr_po{cur_time, vel_x, p_x, p_y};

  if (last_po[0] < 0) {
    last_po = curr_po;
    return false;
  }
  if (curr_po[0] - last_po[0] >= 0.1) {
    HLOG_WARN << "last_time: " << last_po[0] << " curr_time: " << curr_po[0]
              << " diff is larger than 0.1s !!!";
    last_po = curr_po;
    return true;
  }

  if (last_po[1] > 6.0 && curr_po[1] > 6.0) {
    d_p_vt = (curr_po[0] - last_po[0]) * last_po[1];
    d_p = std::sqrt(((curr_po[2] - last_po[2]) * (curr_po[2] - last_po[2])) +
                    ((curr_po[3] - last_po[3]) * (curr_po[3] - last_po[3])));
    if (std::abs(d_p_vt - d_p) > 0.2) {
      HLOG_WARN << "d_p_vt: " << d_p_vt << " d_p: " << d_p
                << " diff is larger than 0.2m!!!";
      last_po = curr_po;
      return true;
    }
  }
  last_po = curr_po;
  return false;
}

void DeadReckoning::CheckTriggerDrDrift(double vel_x, double p_x, double p_y,
                                        double cur_time) {
  if (FLAGS_map_service_mode != 1) {
    return;
  }

  static bool enable_12 = true;
  static double last_time_12 = -1;
  if (IsDrDrift(cur_time, vel_x, p_x, p_y)) {
    if (enable_12) {
      HLOG_WARN << "Start to trigger dc 1012";
      GLOBAL_DC_TRIGGER.TriggerCollect(1012);
      enable_12 = false;
      last_time_12 = cur_time;
    }
  }
  enable_12 = (cur_time - last_time_12) > 600;
}

int32_t DeadReckoning::receive_imu(Bundle* input) {
  static double last_imu_time = -1.0;
  // get one imu data
  auto ptr_rec_imu = input->GetOne("imu_ins");
  auto dr_fault = hozon::perception::lib::FaultManager::Instance();
  auto dr_health = hozon::perception::lib::HealthManager::Instance();
  static bool input_data_error_flag = false;
  static bool input_time_error_flag = false;
  static bool input_time_error_delay_flag = false;
  static bool input_data_nan_flag = false;
  static bool output_data_nan_flag = false;
  static bool output_vel_error_flag = false;

  static int count_imu = 0;
  count_imu++;
  if (count_imu > 100) {
    count_imu = 0;
    HLOG_INFO << "receive imu heartbeat";
  }

  dr_health->HealthReport(
      MAKE_HM_TUPLE(hozon::perception::base::HmModuleId::MAPPING,
                    hozon::perception::base::HealthId::
                        CPID_IMU_FPS_AFTER_DETECT_DATA_ABSTRACT));
  if (!ptr_rec_imu) {
    dr_fault->ReportDebounceTime(MAKE_TIME_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::IMU_DATA_ERROR_MUL_FPS,
        hozon::perception::base::FaultStatus::OCCUR,
        hozon::perception::base::SensorOrientation::UNKNOWN, 400,
        base::DebounceType::DEBOUNCE_TYPE_TIME));
    input_data_error_flag = true;
    HLOG_ERROR << "DR: receive imu data is null";
    return -1;
  } else {
    if (input_data_error_flag) {
      dr_fault->ReportDebounceTime(MAKE_TIME_FM_TUPLE(
          hozon::perception::base::FmModuleId::MAPPING,
          hozon::perception::base::FaultType::IMU_DATA_ERROR_MUL_FPS,
          hozon::perception::base::FaultStatus::RESET,
          hozon::perception::base::SensorOrientation::UNKNOWN, 400,
          base::DebounceType::DEBOUNCE_TYPE_TIME));
      input_data_error_flag = false;
    }
  }

  std::shared_ptr<hozon::soc::ImuIns> imu_proto =
      std::static_pointer_cast<hozon::soc::ImuIns>(ptr_rec_imu->proto_msg);
  double cur_imu_time = imu_proto->header().sensor_stamp().imuins_stamp();
  double sync_time = imu_proto->sync_domain_time_s() * 1e-9;

  if (last_imu_time > 0) {
    if (cur_imu_time - last_imu_time < 0) {
      dr_fault->Report(MAKE_FM_TUPLE(
          hozon::perception::base::FmModuleId::MAPPING,
          hozon::perception::base::FaultType::IMU_DATA_TIME_ERROR_MUL_FPS,
          hozon::perception::base::FaultStatus::OCCUR,
          hozon::perception::base::SensorOrientation::UNKNOWN, 6, 100));
      input_time_error_flag = true;
      HLOG_ERROR << "DR: receive imu data stamp is error:" << "cur_imu_time:"
                 << cur_imu_time << "," << "last_imu_time:" << last_imu_time;
    } else {
      if (input_time_error_flag) {
        dr_fault->Report(MAKE_FM_TUPLE(
            hozon::perception::base::FmModuleId::MAPPING,
            hozon::perception::base::FaultType::IMU_DATA_TIME_ERROR_MUL_FPS,
            hozon::perception::base::FaultStatus::RESET,
            hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
        input_time_error_flag = false;
      }
    }

    if (cur_imu_time - last_imu_time >= 0.03) {
      dr_fault->Report(MAKE_FM_TUPLE(
          hozon::perception::base::FmModuleId::MAPPING,
          hozon::perception::base::FaultType::IMU_DATA_TIME_ERROR_MUL_FPS,
          hozon::perception::base::FaultStatus::OCCUR,
          hozon::perception::base::SensorOrientation::UNKNOWN, 6, 100));
      input_time_error_delay_flag = true;
      HLOG_ERROR << "DR: receive imu data stamp is delay:" << "cur_imu_time:"
                 << cur_imu_time << "," << "last_imu_time:" << last_imu_time;
    } else {
      if (input_time_error_delay_flag) {
        dr_fault->Report(MAKE_FM_TUPLE(
            hozon::perception::base::FmModuleId::MAPPING,
            hozon::perception::base::FaultType::IMU_DATA_TIME_ERROR_MUL_FPS,
            hozon::perception::base::FaultStatus::RESET,
            hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
        input_time_error_delay_flag = false;
      }
    }
  }
  double acc_x = imu_proto->imu_info().linear_acceleration().x();
  double gyr_x = imu_proto->imu_info().angular_velocity().x();
  if (std::isnan(acc_x) || std::isnan(gyr_x)) {
    dr_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::IMU_DATA_VALUE_ERROR_MUL_FPS,
        hozon::perception::base::FaultStatus::OCCUR,
        hozon::perception::base::SensorOrientation::UNKNOWN, 6, 100));
    input_data_nan_flag = true;
    HLOG_ERROR << "DR: receive imu data Nan";
  } else {
    if (input_data_nan_flag) {
      dr_fault->Report(MAKE_FM_TUPLE(
          hozon::perception::base::FmModuleId::MAPPING,
          hozon::perception::base::FaultType::IMU_DATA_VALUE_ERROR_MUL_FPS,
          hozon::perception::base::FaultStatus::RESET,
          hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
      input_data_nan_flag = false;
    }
  }

  dr_interface_->AddImuData(imu_proto);
  last_imu_time = cur_imu_time;

  // 发送dr数据
  std::shared_ptr<hozon::dead_reckoning::DeadReckoning> msg(
      new hozon::dead_reckoning::DeadReckoning);
  if (dr_interface_->GetLatestPose(cur_imu_time, msg, sync_time)) {
    auto dr_output_data = std::make_shared<hozon::netaos::adf_lite::BaseData>();
    dr_output_data->proto_msg = msg;
    Bundle bundle;
    bundle.Add("dr", dr_output_data);
    SendOutput(&bundle);

    double pose_x = msg->pose().pose_local().position().x();
    double pose_y = msg->pose().pose_local().position().y();
    double qua_w = msg->pose().pose_local().quaternion().w();
    double qua_x = msg->pose().pose_local().quaternion().x();
    double qua_y = msg->pose().pose_local().quaternion().y();
    double qua_z = msg->pose().pose_local().quaternion().z();
    double magnitude = std::sqrt(qua_w * qua_w + qua_x * qua_x + qua_y * qua_y +
                                 qua_z * qua_z);
    double vel_x = msg->velocity().twist_vrf().linear_vrf().x();
    if (std::isnan(pose_x) || (abs(magnitude) < 1e-7)) {
      dr_fault->Report(MAKE_FM_TUPLE(
          hozon::perception::base::FmModuleId::MAPPING,
          hozon::perception::base::FaultType::DR_OUTPUT_VALUE_ERROR,
          hozon::perception::base::FaultStatus::OCCUR,
          hozon::perception::base::SensorOrientation::UNKNOWN, 6, 100));
      output_data_nan_flag = true;
      HLOG_ERROR << "DR: output Nan";
    } else {
      if (output_data_nan_flag) {
        dr_fault->Report(MAKE_FM_TUPLE(
            hozon::perception::base::FmModuleId::MAPPING,
            hozon::perception::base::FaultType::DR_OUTPUT_VALUE_ERROR,
            hozon::perception::base::FaultStatus::RESET,
            hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
        output_data_nan_flag = false;
      }
    }

    /*
     * dr ins速度存到队列当中，维持队列大小为100
     * 队列大小为100时，计算队列的均值，进行速度判断
     */
    double dr_vel_x = msg->velocity().twist_vrf().linear_vrf().x();
    Eigen::Vector3d dr_vel(dr_vel_x, 0, 0);
    deque_dr_.push_back(dr_vel);
    if (deque_dr_.size() > 100) {
      deque_dr_.pop_front();
    }

    double ins_vel_x = imu_proto->ins_info().linear_velocity().x();
    double ins_vel_y = imu_proto->ins_info().linear_velocity().y();
    double ins_vel_z = imu_proto->ins_info().linear_velocity().z();
    Eigen::Vector3d ins_vel(ins_vel_x, ins_vel_y, ins_vel_z);
    deque_ins_.push_back(ins_vel);
    if (deque_ins_.size() > 100) {
      deque_ins_.pop_front();
    }

    Eigen::Vector3d mean_dr_vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d mean_ins_vel = Eigen::Vector3d::Zero();
    if (deque_ins_.size() == 100 && deque_dr_.size() == 100) {
      for (auto dr_vel : deque_dr_) {
        mean_dr_vel += dr_vel;
      }
      mean_dr_vel /= deque_dr_.size();

      for (auto ins_vel : deque_ins_) {
        mean_ins_vel += ins_vel;
      }
      mean_ins_vel /= deque_ins_.size();
    }

    HLOG_DEBUG << "mean_dr_vel:" << mean_dr_vel.x() << "," << mean_dr_vel.y()
               << "," << mean_dr_vel.z();
    HLOG_DEBUG << "mean_ins_vel:" << mean_ins_vel.x() << "," << mean_ins_vel.y()
               << "," << mean_ins_vel.z();

    bool vel_error = isVelError(mean_dr_vel, mean_ins_vel);
    static int count = 0;
    if ((imu_proto->ins_info().gps_status() == 4) &&
        (imu_proto->ins_info().sd_position().x() < 0.05) &&
        (imu_proto->ins_info().sd_position().y() < 0.05)) {
      count++;
      if (count > 100) {
        count = 101;
      }
    } else {
      count = 0;
    }

    HLOG_DEBUG << "count:" << count;
    if (vel_error && count > 100) {
      dr_fault->Report(MAKE_FM_TUPLE(
          hozon::perception::base::FmModuleId::MAPPING,
          hozon::perception::base::FaultType::DR_OUTPUT_VELOCITY_ERROR,
          hozon::perception::base::FaultStatus::OCCUR,
          hozon::perception::base::SensorOrientation::UNKNOWN, 6, 100));
      output_vel_error_flag = true;
      HLOG_ERROR << "DR: output vel error";
    } else {
      if (output_vel_error_flag) {
        dr_fault->Report(MAKE_FM_TUPLE(
            hozon::perception::base::FmModuleId::MAPPING,
            hozon::perception::base::FaultType::DR_OUTPUT_VELOCITY_ERROR,
            hozon::perception::base::FaultStatus::RESET,
            hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
        output_vel_error_flag = false;
      }
    }

#ifdef ISORIN
    // mapping trigger 轮速计跳变
    CheckTriggerDrDrift(vel_x, pose_x, pose_y, cur_imu_time);
#endif

  } else {
    HLOG_WARN << "DR: is not init";
  }

  return 0;
}

template <typename T1, typename T2>
bool DeadReckoning::isVelError(const T1& t1, const T2& t2) {
  double vel_dr_norm =
      std::sqrt(t1.x() * t1.x() + t1.y() * t1.y() + t1.z() * t1.z());
  double vel_ins_norm =
      std::sqrt(t2.x() * t2.x() + t2.y() * t2.y() + t2.z() * t2.z());

  HLOG_DEBUG << "vel_dr_norm_test:" << vel_dr_norm << ","
             << "vel_ins_norm_test:" << vel_ins_norm;
  HLOG_DEBUG << "vel_error_threshold_:" << vel_error_threshold_;

  if (std::abs(vel_dr_norm - vel_ins_norm) > vel_error_threshold_ &&
      vel_dr_norm != 0) {
    HLOG_ERROR << "vel_dr_norm:" << vel_dr_norm << ","
               << "vel_ins_norm:" << vel_ins_norm;
    return true;
  }
  return false;
}

}  //  namespace common_onboard
}  //  namespace perception
}  //  namespace hozon
