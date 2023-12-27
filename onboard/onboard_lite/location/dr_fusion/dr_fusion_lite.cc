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
#include "depend/proto/localization/node_info.pb.h"
#include "modules/util/include/util/mapping_log.h"
#include "modules/util/include/util/rviz_agent/rviz_agent.h"
#include "yaml-cpp/yaml.h"
#include "perception-base/base/state_machine/state_machine_info.h"

namespace hozon {
namespace perception {
namespace common_onboard {

const char* const kDrFusionConfSuffix =
    "runtime_service/mapping/conf/mapping/location/dr_fusion/dr_config.yaml";
const char* const KDrFusionLiteConfig =
    "runtime_service/mapping/conf/mapping/location/dr_fusion/"
    "dr_fusion_lite_config.yaml";

int32_t DrFusionLite::AlgInit() {
  const std::string adflite_root_path =
      hozon::perception::lib::GetEnv("ADFLITE_ROOT_PATH", ".");
  const std::string dr_fusion_config =
      adflite_root_path + "/" + kDrFusionConfSuffix;
  const std::string dr_fusion_lite_config =
      adflite_root_path + "/" + KDrFusionLiteConfig;
  dr_fusion_ = std::make_unique<hozon::mp::loc::DrFusion>();
  if (dr_fusion_->Init(dr_fusion_config) != hozon::mp::loc::DrInitStatus::OK) {
    return -1;
  }

  REGISTER_PROTO_MESSAGE_TYPE("/location/ins_fusion",
                              hozon::localization::HafNodeInfo);
  REGISTER_PROTO_MESSAGE_TYPE("dr", hozon::localization::HafNodeInfo);
  REGISTER_PROTO_MESSAGE_TYPE("/location/dr_fusion",
                              hozon::localization::HafNodeInfo);
  REGISTER_PROTO_MESSAGE_TYPE("running_mode",
                              hozon::perception::common_onboard::running_mode);
  HLOG_INFO << "RegistAlgProcessFunc";
  // 输出Ins数据
  RegistAlgProcessFunc("send_dr_proc", std::bind(&DrFusionLite::send_dr, this,
                                                 std::placeholders::_1));
  // 接收数据线程
  RegistAlgProcessFunc("receive_dr", std::bind(&DrFusionLite::receive_dr, this,
                                               std::placeholders::_1));
  RegistAlgProcessFunc("receive_ins_fusion",
                       std::bind(&DrFusionLite::receive_ins_fusion, this,
                                 std::placeholders::_1));
  RegistAlgProcessFunc(
      "recv_running_mode",
      std::bind(&DrFusionLite::OnRunningMode, this, std::placeholders::_1));

  HLOG_INFO << "AlgInit successfully ";
  YAML::Node config = YAML::LoadFile(dr_fusion_lite_config);
  auto use_rviz_bridge = config["use_rviz_bridge"].as<bool>();
  if (use_rviz_bridge) {
    auto viz_addr = config["viz_addr"].as<std::string>();
    int ret = mp::util::RvizAgent::Instance().Init(viz_addr);
    if (ret < 0) {
      HLOG_WARN << "RvizAgent init failed:" << viz_addr;
    }
  }
  return 0;
}

void DrFusionLite::AlgRelease() {
  if (mp::util::RvizAgent::Instance().Ok()) {
    mp::util::RvizAgent::Instance().Term();
  }
}

// send in-process data and interprocess data
int32_t DrFusionLite::send_dr(Bundle* input) {
  auto dr_workflow = std::make_shared<hozon::netaos::adf_lite::BaseData>();

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
  auto ptr_rec_dr = input->GetOne("dr");
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
  auto ptr_rec_inspva = input->GetOne("/location/ins_fusion");
  if (!ptr_rec_inspva) {
    return -1;
  }
  std::shared_ptr<hozon::localization::HafNodeInfo> inspva_proto =
      std::static_pointer_cast<hozon::localization::HafNodeInfo>(
          ptr_rec_inspva->proto_msg);
  dr_fusion_->OnInsFusion(*inspva_proto.get());
  return 0;
}

int32_t DrFusionLite::OnRunningMode(Bundle* input) {
  auto rm_msg = input->GetOne("running_mode");
  if (rm_msg == nullptr) {
    HLOG_ERROR << "nullptr rm_msg plugin";
    return -1;
  }
  auto msg =
      std::static_pointer_cast<hozon::perception::common_onboard::running_mode>(
          rm_msg->proto_msg);
  if (msg == nullptr) {
    HLOG_ERROR << "nullptr rm_msg->proto_msg";
    return -1;
  }
  int runmode = msg->mode();
  // HLOG_ERROR << "!!!!!!!!!!get run mode : " << runmode;
  if (runmode ==
      static_cast<int>(hozon::perception::base::RunningMode::PARKING)) {
    PauseTrigger("send_dr_proc");
    PauseTrigger("receive_ins_fusion");
    PauseTrigger("receive_dr");
    // HLOG_ERROR << "!!!!!!!!!!get run mode PARKING";
  } else if (runmode == static_cast<int>(
                            hozon::perception::base::RunningMode::DRIVING) ||
             runmode ==
                 static_cast<int>(hozon::perception::base::RunningMode::ALL)) {
    ResumeTrigger("send_dr_proc");
    ResumeTrigger("receive_ins_fusion");
    ResumeTrigger("receive_dr");
    // HLOG_ERROR << "!!!!!!!!!!get run mode DRIVER & ALL";
  }
  return 0;
}

}  //  namespace common_onboard
}  //  namespace perception
}  //  namespace hozon
