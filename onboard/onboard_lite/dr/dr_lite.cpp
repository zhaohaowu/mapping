/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: Hozon
 *Date: 2023-08-30
 *****************************************************************************/

#include "adf-lite/include/base.h"
#include "base/utils/log.h"
#include "depend/nos/x86_2004/include/adf-lite/include/executor.h"
#include "depend/nos/x86_2004/include/adf/include/node_proto_register.h"
#include "gtest/gtest.h"
#include "modules/dr/include/dr.h"
#include "onboard/onboard_lite/phm_comment_lite/proto/running_mode.pb.h"
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
    REGISTER_PROTO_MESSAGE_TYPE(
        "running_mode", hozon::perception::common_onboard::running_mode);
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

    // 接收行泊切换信号
    RegistAlgProcessFunc(
        "receive_running_mode",
        std::bind(&DeadReckoning::running_mode, this, std::placeholders::_1));

    // 接收数据线程
    RegistAlgProcessFunc("receive_chassis",
                         std::bind(&DeadReckoning::receive_chassis, this,
                                   std::placeholders::_1));
    RegistAlgProcessFunc("receive_imu", std::bind(&DeadReckoning::receive_imu,
                                                  this, std::placeholders::_1));

    HLOG_INFO << "DR: AlgInit successfully ";
    return 0;
  }

  int32_t running_mode(Bundle* input);
  int32_t receive_chassis(Bundle* input);
  int32_t receive_imu(Bundle* input);

  void AlgRelease() override {}

 private:
  std::shared_ptr<hozon::mp::dr::DRInterface> dr_interface_ = nullptr;
};

REGISTER_ADF_CLASS(DeadReckoning, DeadReckoning);

// send in-process data and interprocess data
int32_t DeadReckoning::running_mode(Bundle* input) {
  HLOG_ERROR << "DR: receive running_mode signal";
  return 0;
}

// recieve in-process data and interprocess data
int32_t DeadReckoning::receive_chassis(Bundle* input) {
  static double last_chassis_time = -1.0;
  // get one chassis data
  auto ptr_rec_chassis = input->GetOne("chassis");
  if (!ptr_rec_chassis) {
    HLOG_ERROR << "DR: receive chassis is null";
    return -1;
  }
  std::shared_ptr<hozon::soc::Chassis> chassis_proto =
      std::static_pointer_cast<hozon::soc::Chassis>(ptr_rec_chassis->proto_msg);
  double cur_chassis_time =
      chassis_proto->header().sensor_stamp().chassis_stamp();
  if (last_chassis_time > 0) {
    if (cur_chassis_time - last_chassis_time < 0) {
      HLOG_ERROR << "DR: receive chassis data stamp is error";
    }
    if (cur_chassis_time - last_chassis_time >= 0.03) {
      HLOG_ERROR << "DR: receive chassis data stamp is delay";
    }
  }
  dr_interface_->AddChassisData(chassis_proto);
  dr_interface_->Process();
  last_chassis_time = cur_chassis_time;
  return 0;
}

int32_t DeadReckoning::receive_imu(Bundle* input) {
  static double last_imu_time = -1.0;
  // get one imu data
  auto ptr_rec_imu = input->GetOne("imu_ins");
  if (!ptr_rec_imu) {
    HLOG_ERROR << "DR: receive imu data is null";
    return -1;
  }
  std::shared_ptr<hozon::soc::ImuIns> imu_proto =
      std::static_pointer_cast<hozon::soc::ImuIns>(ptr_rec_imu->proto_msg);
  double cur_imu_time = imu_proto->header().sensor_stamp().imuins_stamp();
  if (last_imu_time > 0) {
    if (cur_imu_time - last_imu_time < 0) {
      HLOG_ERROR << "DR: receive imu data stamp is error";
    }
    if (cur_imu_time - last_imu_time >= 0.03) {
      HLOG_ERROR << "DR: receive imu data stamp is delay";
    }
  }
  dr_interface_->AddImuData(imu_proto);
  last_imu_time = cur_imu_time;

  // 发送dr数据
  std::shared_ptr<hozon::dead_reckoning::DeadReckoning> msg(
      new hozon::dead_reckoning::DeadReckoning);
  if (dr_interface_->GetLatestPose(cur_imu_time, msg)) {
    auto dr_output_data = std::make_shared<hozon::netaos::adf_lite::BaseData>();
    dr_output_data->proto_msg = msg;
    Bundle bundle;
    bundle.Add("dr", dr_output_data);
    SendOutput(&bundle);
  } else {
    HLOG_WARN << "DR: is not init";
  }
  return 0;
}

}  //  namespace common_onboard
}  //  namespace perception
}  //  namespace hozon
