/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： dr_fusion.cc
 *   author     ： nihongjie
 *   date       ： 2023.09
 ******************************************************************************/
#include "onboard/onboard_lite/location/dr_fusion/dr_fusion_lite.h"

#include <gflags/gflags.h>

#include <perception-lib/lib/environment/environment.h>
#include "base/utils/log.h"
#include "modules/util/include/util/temp_log.h"

namespace hozon {
namespace perception {
namespace common_onboard {

const char* const kDrFusionConfSuffix =
    "runtime_service/mapping/conf/mapping/location/dr_fusion/dr_config.yaml";

int32_t DrFusionLite::AlgInit() {
  const std::string adflite_root_path =
      hozon::perception::lib::GetEnv("ADFLITE_ROOT_PATH", ".");
  const std::string dr_fusion_config =
      adflite_root_path + "/" + kDrFusionConfSuffix;
  dr_fusion_ = std::make_unique<hozon::mp::loc::DrFusion>();
  if (dr_fusion_->Init(dr_fusion_config) != hozon::mp::loc::DrInitStatus::OK) {
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
      "receive_ins_fusion",
      std::bind(&DrFusionLite::receive_ins_fusion, this, std::placeholders::_1));

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
  SendOutput("/location/dr_fusion", dr_workflow);
  return 0;
}

// recieve in-process data and interprocess data
int32_t DrFusionLite::receive_dr(Bundle* input) {
  BaseDataTypePtr ptr_rec_dr = input->GetOne("dr");
  if (!ptr_rec_dr) {
    return -1;
  }
  std::shared_ptr<hozon::dead_reckoning::DeadReckoning> dr_proto =
      std::static_pointer_cast<hozon::dead_reckoning::DeadReckoning>(
          ptr_rec_dr->proto_msg);
  dr_fusion_->OnDr(*dr_proto.get());
  return 0;
}

int32_t DrFusionLite::receive_ins_fusion(Bundle* input) {
  BaseDataTypePtr ptr_rec_inspva = input->GetOne("/location/ins_fusion");
  if (!ptr_rec_inspva) {
    return -1;
  }
  std::shared_ptr<hozon::localization::HafNodeInfo> inspva_proto =
      std::static_pointer_cast<hozon::localization::HafNodeInfo>(
          ptr_rec_inspva->proto_msg);
  dr_fusion_->OnInsFusion(*inspva_proto.get());
  return 0;
}
}  //  namespace common_onboard
}  //  namespace perception
}  //  namespace hozon
