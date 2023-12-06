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
#include "proto/dead_reckoning/dr.pb.h"
#include "proto/soc/chassis.pb.h"
#include "proto/soc/sensor_imu_ins.pb.h"

namespace hozon {
namespace perception {
namespace common_onboard {

using hozon::netaos::adf_lite::Bundle;

class DeadReckoning : public hozon::netaos::adf_lite::Executor {
 public:
  DeadReckoning() = default;
  ~DeadReckoning() = default;
  int32_t AlgInit() override {
    // register proto for ipc
    hozon::netaos::log::InitLogging("dr_executor", "dr_executor test",
                                    hozon::netaos::log::LogLevel::kInfo,
                                    HZ_LOG2CONSOLE, "./", 10, (20));

    hozon::netaos::adf::NodeLogger::GetInstance().CreateLogger(
        "dr_executor", "dr_executor test", hozon::netaos::log::LogLevel::kInfo);

    REGISTER_PROTO_MESSAGE_TYPE("imu_ins", hozon::soc::ImuIns);
    REGISTER_PROTO_MESSAGE_TYPE("chassis", hozon::soc::Chassis);
    REGISTER_PROTO_MESSAGE_TYPE("dr", hozon::dead_reckoning::DeadReckoning);

    // 输出DR数据
    RegistAlgProcessFunc("dr_proc", std::bind(&DeadReckoning::dr_process, this,
                                              std::placeholders::_1));

    // 接收数据线程
    RegistAlgProcessFunc(
        "receive_imu_chassis",
        std::bind(&DeadReckoning::data_receive, this, std::placeholders::_1));

    HLOG_INFO << "DR: AlgInit successfully ";
    return 0;
  }

  int32_t dr_process(Bundle* input);
  int32_t data_receive(Bundle* input);

  void AlgRelease() override {}

 private:
  hozon::mp::dr::DRInterface dr_interface;
};

REGISTER_ADF_CLASS(DeadReckoning, DeadReckoning);

// send in-process data and interprocess data
int32_t DeadReckoning::dr_process(Bundle* input) {
  std::shared_ptr<hozon::dead_reckoning::DeadReckoning> msg(
      new hozon::dead_reckoning::DeadReckoning);
  if (dr_interface.Process(msg)) {
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

// recieve in-process data and interprocess data
int32_t DeadReckoning::data_receive(Bundle* input) {
  static double last_imu_time = -1.0;
  static double last_chassis_time = -1.0;

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
  dr_interface.AddImuData(imu_proto);
  last_imu_time = cur_imu_time;

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
  dr_interface.AddChassisData(chassis_proto);
  last_chassis_time = cur_chassis_time;
  return 0;
}

}  //  namespace common_onboard
}  //  namespace perception
}  //  namespace hozon
