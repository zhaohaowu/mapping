/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： dr_fusion.cc
 *   author     ： nihongjie
 *   date       ： 2023.09
 ******************************************************************************/
#include "onboard/onboard_lite/location/dr_fusion/dr_fusion_lite.h"

#include <gflags/gflags.h>

#include "base/utils/log.h"
#include "modules/util/include/util/temp_log.h"

DEFINE_string(
    dr_config,
    "/svp_data/release/orin/conf/mapping/location/dr_fusion/dr_config.yaml",
    "config file path for ins_config");

namespace hozon {
namespace perception {
namespace common_onboard {

int32_t DrFusionLite::AlgInit() {
  dr_fusion_ = std::make_unique<hozon::mp::loc::DrFusion>();
  if (dr_fusion_->Init(FLAGS_dr_config) != hozon::mp::loc::InsInitStatus::OK) {
    return -1;
  }
  // register proto for ipc
  hozon::netaos::log::InitLogging(
      "dr_fusion_executor", "dr_fusion_executor test",
      hozon::netaos::log::LogLevel::kInfo, hozon::netaos::log::HZ_LOG2CONSOLE,
      "./", 10, (20));

  hozon::netaos::adf::NodeLogger::GetInstance().CreateLogger(
      "dr_fusion_executor", "dr_fusion_executor test",
      hozon::netaos::log::LogLevel::kInfo);

  REGISTER_MESSAGE_TYPE("/location/ins_fusion",
                        hozon::localization::HafNodeInfo);
  REGISTER_MESSAGE_TYPE("dr", hozon::localization::HafNodeInfo);
  REGISTER_MESSAGE_TYPE("/location/dr_fusion",
                        hozon::localization::HafNodeInfo);
  HLOG_INFO << "RegistAlgProcessFunc";
  // 输出Ins数据
  RegistAlgProcessFunc("send_dr_proc", std::bind(&DrFusionLite::send_dr, this,
                                                 std::placeholders::_1));
  // 接收数据线程
  RegistAlgProcessFunc("receive_dr", std::bind(&DrFusionLite::receive_dr, this,
                                               std::placeholders::_1));
  RegistAlgProcessFunc(
      "receive_inspva",
      std::bind(&DrFusionLite::receive_inspva, this, std::placeholders::_1));

  HLOG_INFO << "AlgInit successfully ";
  return 0;
}

// send in-process data and interprocess data
int32_t DrFusionLite::send_dr(Bundle* input) {
  BaseDataTypePtr dr_workflow =
      std::make_shared<hozon::netaos::adf_lite::BaseData>();

  std::shared_ptr<hozon::localization::HafNodeInfo> msg(
      new hozon::localization::HafNodeInfo);
  if (!dr_fusion_->GetResult(msg.get())) {
    return -1;
  }
  dr_workflow->proto_msg = msg;
  SendOutput("/location/deadreckoning", dr_workflow);
  return 0;
}

// recieve in-process data and interprocess data
int32_t DrFusionLite::receive_dr(Bundle* input) {
  BaseDataTypePtr ptr_rec_dr = input->GetOne("dr");
  if (!ptr_rec_dr) {
    return -1;
  }
  std::shared_ptr<hozon::localization::HafNodeInfo> dr_proto =
      std::static_pointer_cast<hozon::localization::HafNodeInfo>(
          ptr_rec_dr->proto_msg);
  dr_fusion_->OnDr(*dr_proto.get());
  return 0;
}

int32_t DrFusionLite::receive_inspva(Bundle* input) {
  BaseDataTypePtr ptr_rec_inspva = input->GetOne("/location/ins_fusion");
  if (!ptr_rec_inspva) {
    return -1;
  }
  std::shared_ptr<hozon::localization::HafNodeInfo> inspva_proto =
      std::static_pointer_cast<hozon::localization::HafNodeInfo>(
          ptr_rec_inspva->proto_msg);
  dr_fusion_->OnInspva(*inspva_proto.get());
  return 0;
}
}  //  namespace common_onboard
}  //  namespace perception
}  //  namespace hozon
