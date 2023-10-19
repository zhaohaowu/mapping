/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： ins_fusion.cc
 *   author     ： nihongjie
 *   date       ： 2023.09
 ******************************************************************************/
#include "onboard/onboard_lite/location/ins_fusion/ins_fusion_lite.h"

#include <gflags/gflags.h>

#include "base/utils/log.h"
#include "modules/util/include/util/temp_log.h"

DEFINE_string(ins_config,
              "/home/zhangyu/zy/code/mapping/release/x86/conf/mapping/location/ins_fusion/"
              "ins_config.yaml",
              "config file path for ins_config");

namespace hozon {
namespace perception {
namespace common_onboard {

int32_t InsFusionLite::AlgInit() {
  ins_fusion_ = std::make_unique<hozon::mp::loc::InsFusion>();
  if (ins_fusion_->Init(FLAGS_ins_config) !=
      hozon::mp::loc::InsInitStatus::OK) {
    return -1;
  }
  // register proto for ipc
  hozon::netaos::log::InitLogging(
      "ins_fusion_executor", "ins_fusion_executor test",
      hozon::netaos::log::LogLevel::kInfo, hozon::netaos::log::HZ_LOG2CONSOLE,
      "./", 10, (20));

  hozon::netaos::adf::NodeLogger::GetInstance().CreateLogger(
      "ins_fusion_executor", "ins_fusion_executor test",
      hozon::netaos::log::LogLevel::kInfo);

  REGISTER_MESSAGE_TYPE("inspva", hozon::localization::HafNodeInfo);
  REGISTER_MESSAGE_TYPE("imu_ins", hozon::soc::ImuIns);
  REGISTER_MESSAGE_TYPE("/location/ins_fusion",
                        hozon::localization::HafNodeInfo);
  HLOG_INFO << "RegistAlgProcessFunc";
  // 输出Ins数据
  RegistAlgProcessFunc("send_ins_proc", std::bind(&InsFusionLite::send_ins,
                                                  this, std::placeholders::_1));
  // 接收数据线程
  RegistAlgProcessFunc("receive_ins", std::bind(&InsFusionLite::receive_ins,
                                                this, std::placeholders::_1));
  RegistAlgProcessFunc(
      "receive_inspva",
      std::bind(&InsFusionLite::receive_inspva, this, std::placeholders::_1));

  HLOG_INFO << "AlgInit successfully ";
  return 0;
}

// send in-process data and interprocess data
int32_t InsFusionLite::send_ins(Bundle* input) {
  BaseDataTypePtr ins_workflow =
      std::make_shared<hozon::netaos::adf_lite::BaseData>();

  std::shared_ptr<hozon::localization::HafNodeInfo> msg(
      new hozon::localization::HafNodeInfo);
  if (!ins_fusion_->GetResult(msg.get())) {
    return -1;
  }

  ins_workflow->proto_msg = msg;
  SendOutput("/location/ins_fusion", ins_workflow);
  return 0;
}

// recieve in-process data and interprocess data
int32_t InsFusionLite::receive_ins(Bundle* input) {
  BaseDataTypePtr ptr_rec_ins = input->GetOne("imu_ins");
  if (!ptr_rec_ins) {
    return -1;
  }
  std::shared_ptr<hozon::soc::ImuIns> ins_proto =
      std::static_pointer_cast<hozon::soc::ImuIns>(ptr_rec_ins->proto_msg);
  ins_fusion_->OnOriginIns(*ins_proto.get());
  return 0;
}

int32_t InsFusionLite::receive_inspva(Bundle* input) {
  BaseDataTypePtr ptr_rec_inspva = input->GetOne("inspva");
  if (!ptr_rec_inspva) {
    return -1;
  }

  std::shared_ptr<hozon::localization::HafNodeInfo> inspva_proto =
      std::static_pointer_cast<hozon::localization::HafNodeInfo>(
          ptr_rec_inspva->proto_msg);
  ins_fusion_->OnInspva(*inspva_proto.get());
  return 0;
}
}  //  namespace common_onboard
}  //  namespace perception
}  //  namespace hozon
