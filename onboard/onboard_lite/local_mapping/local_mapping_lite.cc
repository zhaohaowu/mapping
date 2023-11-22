/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： prior_provider_lite.cc
 *   author     ： taoshaoyuan
 *   date       ： 2023.09
 ******************************************************************************/
#include "onboard/onboard_lite/local_mapping/local_mapping_lite.h"
// #include "perception-lib/lib/location_manager/location_manager.h"
#include <adf-lite/include/base.h>
#include <base/utils/log.h>
#include <common_onboard/adapter/adapter.h>
#include <common_onboard/adapter/onboard_lite/onboard_lite.h>
#include <gflags/gflags.h>
#include <proto/localization/node_info.pb.h>

#include <filesystem>
#include <string>

#include "perception-lib/lib/environment/environment.h"

namespace hozon {
namespace perception {
namespace common_onboard {

int32_t LocalMappingOnboard::AlgInit() {
  hozon::netaos::log::InitLogging("lm_executor", "lm_executor test",
                                  hozon::netaos::log::LogLevel::kInfo,
                                  HZ_LOG2CONSOLE, "./", 10, (20));

  hozon::netaos::adf::NodeLogger::GetInstance().CreateLogger(
      "lm_executor", "lm_executor test", hozon::netaos::log::LogLevel::kInfo);

  REGISTER_MESSAGE_TYPE("percep_transport",
                        hozon::perception::TransportElement);
  REGISTER_MESSAGE_TYPE("dr", hozon::dead_reckoning::DeadReckoning);

  std::string default_work_root = "/app/";
  std::string work_root = lib::GetEnv("ADFLITE_ROOT_PATH", default_work_root);
  if (work_root == "") {
    HLOG_ERROR << "ENV: ADFLITE_ROOT_PATH is not set.";
    return false;
  }
  std::string config_file = work_root +
                            "/runtime_service/mapping/conf/mapping/"
                            "local_mapping/local_mapping_conf.yaml";

  YAML::Node config = YAML::LoadFile(config_file);
  if (config["use_rviz"].as<bool>()) {
    HLOG_INFO << "Start RvizAgent!!!";
    int ret = hozon::mp::util::RvizAgent::Instance().Init(
        config["rviz_addr"].as<std::string>());
    if (ret < 0) {
      HLOG_ERROR << "RvizAgent start failed";
    }
  }
  lmap_ = std::make_shared<LMapApp>(config_file);

  RegistAlgProcessFunc(
      "recv_laneline",
      std::bind(&LocalMappingOnboard::OnLaneLine, this, std::placeholders::_1));

  RegistAlgProcessFunc("recv_dr", std::bind(&LocalMappingOnboard::OnDr, this,
                                            std::placeholders::_1));

  RegistAlgProcessFunc("recv_ins", std::bind(&LocalMappingOnboard::OnIns, this,
                                             std::placeholders::_1));

  // RegistAlgProcessFunc(
  //     "recv_roadedge",
  //     std::bind(&LocalMappingOnboard::OnRoadEdge, this,
  //     std::placeholders::_1));

  RegistAlgProcessFunc("send_lm",
                       std::bind(&LocalMappingOnboard::LocalMapPublish, this,
                                 std::placeholders::_1));

  return 0;
}

void LocalMappingOnboard::AlgRelease() {
  hozon::mp::util::RvizAgent::Instance().Term();
}

int32_t LocalMappingOnboard::OnLaneLine(Bundle* input) {
  HLOG_DEBUG << "receive laneline data...";
  BaseDataTypePtr laneline_msg = input->GetOne("percep_transport");
  if (!laneline_msg) {
    HLOG_ERROR << "nullptr track lane plugin";
    return -1;
  }

  if (!lmap_) {
    HLOG_ERROR << "LMapApp init failed!!!";
    return false;
  }

  auto msg = std::static_pointer_cast<hozon::perception::TransportElement>(
      laneline_msg->proto_msg);

  lmap_->OnLaneLine(msg);
  // HLOG_INFO << "processed laneline data";
  return 0;
}

int32_t LocalMappingOnboard::OnDr(Bundle* input) {
  HLOG_DEBUG << "receive dr data...";
  BaseDataTypePtr dr_msg = input->GetOne("dr");
  if (!dr_msg) {
    HLOG_ERROR << "nullptr dr_msg plugin";
    return -1;
  }

  if (!lmap_) {
    return false;
  }

  auto msg = std::static_pointer_cast<hozon::dead_reckoning::DeadReckoning>(
      dr_msg->proto_msg);

  if (msg->pose().pose_local().quaternion().w() == 0 &&
      msg->pose().pose_local().quaternion().x() == 0 &&
      msg->pose().pose_local().quaternion().y() == 0 &&
      msg->pose().pose_local().quaternion().z() == 0) {
    return false;
  }
  lmap_->OnDr(msg);
  // HLOG_INFO << "processed dr data";
  return 0;
}

int32_t LocalMappingOnboard::OnIns(Bundle* input) {
  BaseDataTypePtr ins_msg = input->GetOne("ins");
  auto msg = std::static_pointer_cast<hozon::localization::HafNodeInfo>(
      ins_msg->proto_msg);
  if (!lmap_) {
    return false;
  }

  lmap_->OnIns(msg);
  return true;
}

// int32_t LocalMappingOnboard::OnRoadEdge(Bundle* input) {
//   BaseDataTypePtr road_edge_msg = input->GetOne("percep_transport");
//   auto msg = std::static_pointer_cast<common_onboard::NetaTransportElement>(
//       road_edge_msg->proto_msg);
//   if (!lmap_) {
//     return false;
//   }
//   lmap_->OnRoadEdge(msg);
// }

int32_t LocalMappingOnboard::LocalMapPublish(common_onboard::Bundle* output) {
  HLOG_DEBUG << "start publish localmap...";
  std::shared_ptr<hozon::mapping::LocalMap> result =
      std::make_shared<hozon::mapping::LocalMap>();
  if (lmap_->FetchLocalMap(result)) {
    BaseDataTypePtr workflow1 =
        std::make_shared<hozon::netaos::adf_lite::BaseData>();

    workflow1->proto_msg = result;
    Bundle bundle;
    bundle.Add("local_map", workflow1);
    SendOutput(&bundle);
    HLOG_DEBUG << "publish localmap suceessed...";
  }
  HLOG_DEBUG << "processed publish localmap";
  usleep(100 * 1e3);
  return 0;
}

REGISTER_EXECUTOR_CLASS(LocalMappingOnboard, LocalMappingOnboard);

}  // namespace common_onboard
}  // namespace perception
}  // namespace hozon
