/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： ins_fusion.cc
 *   author     ： nihongjie
 *   date       ： 2023.09
 ******************************************************************************/
#include "onboard/onboard_lite/location/ins_fusion/ins_fusion_lite.h"
#include <gflags/gflags.h>
#include <perception-lib/lib/environment/environment.h>

#include "base/utils/log.h"
#include "depend/proto/localization/node_info.pb.h"
#include "depend/proto/perception/transport_element.pb.h"
#include "depend/proto/soc/sensor_imu_ins.pb.h"
#include "modules/util/include/util/temp_log.h"

namespace hozon {
namespace perception {
namespace common_onboard {

const char* const kInsFusionConfSuffix =
    "runtime_service/mapping/conf/mapping/location/ins_fusion/ins_config.yaml";

int32_t InsFusionLite::AlgInit() {
  const std::string adflite_root_path =
      hozon::perception::lib::GetEnv("ADFLITE_ROOT_PATH", ".");
  const std::string ins_fusion_config =
      adflite_root_path + "/" + kInsFusionConfSuffix;
  ins_fusion_ = std::make_unique<hozon::mp::loc::InsFusion>();
  if (ins_fusion_->Init(ins_fusion_config) !=
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

  REGISTER_PROTO_MESSAGE_TYPE("inspva", hozon::localization::HafNodeInfo);
  REGISTER_PROTO_MESSAGE_TYPE("imu_ins", hozon::soc::ImuIns);
  REGISTER_PROTO_MESSAGE_TYPE("/location/ins_fusion",
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
  auto ins_workflow = std::make_shared<hozon::netaos::adf_lite::BaseData>();

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
  auto ptr_rec_ins = input->GetOne("imu_ins");
  if (!ptr_rec_ins) {
    return -1;
  }
  std::shared_ptr<hozon::soc::ImuIns> ins_proto =
      std::static_pointer_cast<hozon::soc::ImuIns>(ptr_rec_ins->proto_msg);
  ins_fusion_->OnOriginIns(*ins_proto.get());
  return 0;
}

int32_t InsFusionLite::receive_inspva(Bundle* input) {
  auto ptr_rec_inspva = input->GetOne("inspva");
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
