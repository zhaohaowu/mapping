/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: Hozon
 *Date: 2023-08-30
 *****************************************************************************/

#include "adf-lite/include/base.h"
#include "base/utils/log.h"
#include "common_onboard/adapter/onboard_lite/onboard_lite.h"
#include "gtest/gtest.h"
#include "modules/dr/include/dr.h"
#include "proto/soc/chassis.pb.h"
#include "proto/dead_reckoning/dr.pb.h"
#include "proto/soc/sensor_imu_ins.pb.h"

namespace hozon {
namespace perception {
namespace common_onboard {

class DeadReckoning : public OnboardLite {
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

    REGISTER_MESSAGE_TYPE("imu_ins", hozon::soc::ImuIns);
    REGISTER_MESSAGE_TYPE("chassis", hozon::soc::Chassis);
    REGISTER_MESSAGE_TYPE("dr", hozon::dead_reckoning::DeadReckoning);

    // 输出DR数据
    RegistAlgProcessFunc("dr_proc", std::bind(&DeadReckoning::dr_process, this,
                                              std::placeholders::_1));

    // 接收数据线程
    RegistAlgProcessFunc(
        "receive_imu_chassis",
        std::bind(&DeadReckoning::data_receive, this, std::placeholders::_1));

    HLOG_INFO << "AlgInit successfully ";
    return 0;
  }

  int32_t dr_process(Bundle* input);
  int32_t data_receive(Bundle* input);

  void AlgRelease() override {}

 private:
  hozon::mp::dr::DRInterface dr_interface;
};

REGISTER_EXECUTOR_CLASS("DeadReckoning", DeadReckoning);

// send in-process data and interprocess data
int32_t DeadReckoning::dr_process(Bundle* input) {
  BaseDataTypePtr workflow1 =
      std::make_shared<hozon::netaos::adf_lite::BaseData>();

  std::shared_ptr<hozon::dead_reckoning::DeadReckoning> msg(
      new hozon::dead_reckoning::DeadReckoning);

  dr_interface.SetLocation(msg);

  workflow1->proto_msg = msg;

  Bundle bundle;
  bundle.Add("dr", workflow1);
  SendOutput(&bundle);
  HLOG_INFO << "==== init ==== ------------------detect send odom-2-3 success.------------";
  return 0;
}

// recieve in-process data and interprocess data
int32_t DeadReckoning::data_receive(Bundle* input) {
  HLOG_INFO << "==== init ==== ------------------=====.------------";

  BaseDataTypePtr ptr_rec_imu = input->GetOne("imu_ins");
  if (!ptr_rec_imu) {
    // HLOG_INFO << "detect alg process fun2 call\n";
    HLOG_ERROR << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!imu null!!!!!!!!!!!!.";
    return -1;
  }

  std::shared_ptr<hozon::soc::ImuIns> imu_proto =
      std::static_pointer_cast<hozon::soc::ImuIns>(
          ptr_rec_imu->proto_msg);

  dr_interface.AddImuData(imu_proto);

  ////////////////////////////////////////////////////////////////////
  BaseDataTypePtr ptr_rec_chassis = input->GetOne("chassis");
  if (!ptr_rec_chassis) {
    // HLOG_INFO << "detect alg process fun2 call\n";
    HLOG_ERROR << "!!!!!!!!!!!!!!!!!!!!Chassis null !!!!!!!!!!!!.";
    return -1;
  }

  std::shared_ptr<hozon::soc::Chassis> chassis_proto =
      std::static_pointer_cast<hozon::soc::Chassis>(ptr_rec_chassis->proto_msg);


  HLOG_INFO << "================= fr wheel: "
            << double(chassis_proto->wheel_counter().wheel_counter_fl())
            << " ,gear: " << int(chassis_proto->gear_location()) << ", fl dir: "
            << chassis_proto->wheel_speed().wheel_direction_fl();

  dr_interface.AddChassisData(chassis_proto);
  return 0;
}

}  //  namespace common_onboard
}  //  namespace perception
}  //  namespace hozon
