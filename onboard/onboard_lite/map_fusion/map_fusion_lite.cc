/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： map_fusion_lite.cc
 *   author     ： xuliang
 *   date       ： 2023.11
 ******************************************************************************/
#include <base/utils/log.h>
#include <gflags/gflags.h>
#include <perception-lib/lib/environment/environment.h>

#include <string>

#include "modules/util/include/util/rviz_agent/rviz_agent.h"
#include "onboard/onboard_lite/map_fusion/map_fusion_config_lite.h"
#include "onboard/onboard_lite/map_fusion/map_fusion_lite.h"

namespace hozon {
namespace perception {
namespace common_onboard {

int32_t MapFusionLite::AlgInit() {
  std::string default_work_root = "/app/";
  std::string work_root = lib::GetEnv("ADFLITE_ROOT_PATH", default_work_root);
  if (work_root == "") {
    HLOG_ERROR << "ENV: ADFLITE_ROOT_PATH is not set.";
    return false;
  }
  std::string config_file = work_root +
                            "/runtime_service/mapping/conf/mapping/"
                            "map_fusion/map_fusion.yaml";

  YAML::Node config = YAML::LoadFile(config_file);

  FLAGS_orin_viz = config["orin_viz"].as<bool>();
  FLAGS_orin_viz_addr = config["orin_viz_addr"].as<std::string>();
  FLAGS_map_dir = work_root + config["map_dir"].as<std::string>();
  FLAGS_use_ehp = config["use_ehp"].as<bool>();
  FLAGS_use_ehp_odd_to_stop = config["use_ehp_odd_to_stop"].as<bool>();
  FLAGS_ehp_monitor = config["ehp_monitor"].as<int32_t>();
  FLAGS_enable_ehp_routing = config["enable_ehp_routing"].as<bool>();
  FLAGS_using_record_ehp_data = config["using_record_ehp_data"].as<bool>();
  FLAGS_ehp_log_root_dir = config["ehp_log_root_dir"].as<std::string>();
  FLAGS_map_service_mode = config["map_service_mode"].as<int32_t>();
  FLAGS_topo_rviz = config["topo_rviz"].as<bool>();
  FLAGS_viz_odom_map_in_local = config["viz_odom_map_in_local"].as<bool>();

  if (FLAGS_orin_viz) {
    HLOG_INFO << "Start RvizAgent on " << FLAGS_orin_viz_addr;
    int ret = RVIZ_AGENT.Init(FLAGS_orin_viz_addr);
    if (ret < 0) {
      HLOG_ERROR << "RvizAgent start failed";
    }
  }

  mf_ = std::make_unique<MapFusion>();
  int ret = mf_->Init("");
  if (ret < 0) {
    HLOG_ERROR << "Init map fusion failed";
    return -1;
  }

  RegistLog();
  RegistMessageType();
  RegistProcessFunc();

  return 0;
}

void MapFusionLite::AlgRelease() {
  if (mf_) {
    HLOG_INFO << "try stopping map fusion";
    mf_->Stop();
    HLOG_INFO << "done stopping map fusion";
  }

  if (RVIZ_AGENT.Ok()) {
    RVIZ_AGENT.Term();
  }
}

void MapFusionLite::RegistLog() {
  hozon::netaos::log::InitLogging("map_fusion_executor", "map_fusion",
                                  hozon::netaos::log::LogLevel::kInfo,
                                  HZ_LOG2CONSOLE, "./", 10, (20));

  hozon::netaos::adf::NodeLogger::GetInstance().CreateLogger(
      "map_fusion_executor", "map_fusion", hozon::netaos::log::LogLevel::kInfo);
}

void MapFusionLite::RegistMessageType() {
  // 消息名称最终都需要改一下
  REGISTER_MESSAGE_TYPE("localization", hozon::localization::Localization);
  REGISTER_MESSAGE_TYPE("/location/ins_fusion",
                        hozon::localization::HafNodeInfo);
  REGISTER_MESSAGE_TYPE("local_map", hozon::mapping::LocalMap);
  // map fusion output message
  REGISTER_MESSAGE_TYPE("map_fusion", hozon::hdmap::Map);
  REGISTER_MESSAGE_TYPE("routing_response", hozon::routing::RoutingResponse);
}

void MapFusionLite::RegistProcessFunc() {
  RegistAlgProcessFunc("recv_location", std::bind(&MapFusionLite::OnLocation,
                                                  this, std::placeholders::_1));

  RegistAlgProcessFunc(
      "recv_local_map",
      std::bind(&MapFusionLite::OnLocalMap, this, std::placeholders::_1));

  RegistAlgProcessFunc(
      "recv_loc_plugin",
      std::bind(&MapFusionLite::OnLocPlugin, this, std::placeholders::_1));

  RegistAlgProcessFunc(
      "send_map_fusion",
      std::bind(&MapFusionLite::MapFusionOutput, this, std::placeholders::_1));
}

int32_t MapFusionLite::OnLocation(Bundle* input) {
  if (!input) {
    return -1;
  }
  BaseDataTypePtr p_loc = input->GetOne("localization");
  if (!p_loc) {
    HLOG_ERROR << "nullptr local map message";
    return -1;
  }
  const auto loc_res =
      std::static_pointer_cast<hozon::localization::Localization>(
          p_loc->proto_msg);

  if (!loc_res) {
    HLOG_ERROR << "nullptr location";
    return -1;
  }
  {
    std::lock_guard<std::mutex> lock(loc_mtx_);
    curr_loc_ = std::make_shared<hozon::localization::Localization>(*loc_res);
  }

  return 0;
}

int32_t MapFusionLite::OnLocalMap(Bundle* input) {
  if (!input) {
    return -1;
  }
  BaseDataTypePtr p_local_map = input->GetOne("local_map");
  if (!p_local_map) {
    HLOG_ERROR << "nullptr local map message";
    return -1;
  }
  const auto local_map_res = std::static_pointer_cast<hozon::mapping::LocalMap>(
      p_local_map->proto_msg);

  if (!local_map_res) {
    HLOG_ERROR << "nullptr local map";
    return -1;
  }
  {
    std::lock_guard<std::mutex> lock(local_map_mtx_);
    curr_local_map_ =
        std::make_shared<hozon::mapping::LocalMap>(*local_map_res);
  }

  return 0;
}

int32_t MapFusionLite::OnLocPlugin(Bundle* input) {
  if (!input) {
    return -1;
  }
  BaseDataTypePtr p_loc_plugin = input->GetOne("/location/ins_fusion");
  if (!p_loc_plugin) {
    HLOG_ERROR << "nullptr location plugin message";
    return -1;
  }
  const auto loc_plugin_res =
      std::static_pointer_cast<hozon::localization::HafNodeInfo>(
          p_loc_plugin->proto_msg);

  if (!loc_plugin_res) {
    HLOG_ERROR << "nullptr local map";
    return -1;
  }
  {
    std::lock_guard<std::mutex> lock(plugin_mtx_);
    curr_plugin_ =
        std::make_shared<hozon::localization::HafNodeInfo>(*loc_plugin_res);
  }

  return 0;
}

int32_t MapFusionLite::MapFusionOutput(Bundle* output) {
  if (!mf_) {
    HLOG_INFO << "nullptr map fusion";
    return -1;
  }

  std::shared_ptr<hozon::localization::HafNodeInfo> latest_plugin =
      GetLatestLocPlugin();
  if (latest_plugin == nullptr) {
    HLOG_INFO << "nullptr latest plugin node info";
    return -1;
  }
  std::shared_ptr<hozon::planning::ADCTrajectory> latest_planning = nullptr;
  hozon::routing::RoutingResponse routing;
  mf_->ProcService(latest_plugin, latest_planning, &routing);

  std::shared_ptr<hozon::routing::RoutingResponse> latest_routing =
      std::make_shared<hozon::routing::RoutingResponse>(routing);

  std::shared_ptr<hozon::localization::Localization> latest_loc =
      GetLatestLoc();
  if (latest_loc == nullptr) {
    HLOG_INFO << "nullptr latest loc";
    return -1;
  }
  std::shared_ptr<hozon::mapping::LocalMap> latest_local_map =
      GetLatestLocalMap();
  if (latest_local_map == nullptr) {
    HLOG_INFO << "nullptr latest local map";
    return -1;
  }

  auto latest_map = std::make_shared<hozon::hdmap::Map>();
  int ret = mf_->ProcFusion(latest_loc, latest_local_map, latest_map.get());
  if (ret < 0) {
    HLOG_INFO << "map fusion ProcFusion failed";
    return -1;
  }

  ret = SendFusionResult(latest_map, latest_routing);
  if (ret < 0) {
    HLOG_INFO << "SendFusionResult failed";
    return -1;
  }

  return 0;
}

std::shared_ptr<hozon::localization::Localization>
MapFusionLite::GetLatestLoc() {
  std::shared_ptr<hozon::localization::Localization> latest_loc = nullptr;
  std::lock_guard<std::mutex> lock(loc_mtx_);
  if (curr_loc_ != nullptr) {
    latest_loc =
        std::make_shared<hozon::localization::Localization>(*curr_loc_);
  }
  return latest_loc;
}

std::shared_ptr<hozon::mapping::LocalMap> MapFusionLite::GetLatestLocalMap() {
  std::shared_ptr<hozon::mapping::LocalMap> latest_local_map = nullptr;
  std::lock_guard<std::mutex> lock(local_map_mtx_);
  if (curr_local_map_ != nullptr) {
    latest_local_map =
        std::make_shared<hozon::mapping::LocalMap>(*curr_local_map_);
  }
  return latest_local_map;
}

std::shared_ptr<hozon::localization::HafNodeInfo>
MapFusionLite::GetLatestLocPlugin() {
  std::shared_ptr<hozon::localization::HafNodeInfo> latest_plugin = nullptr;
  std::lock_guard<std::mutex> lock(plugin_mtx_);
  if (curr_plugin_ != nullptr) {
    latest_plugin =
        std::make_shared<hozon::localization::HafNodeInfo>(*curr_plugin_);
  }
  return latest_plugin;
}

int MapFusionLite::SendFusionResult(
    const std::shared_ptr<hozon::hdmap::Map>& map,
    const std::shared_ptr<hozon::routing::RoutingResponse>& routing) {
  if (map == nullptr || routing == nullptr) {
    HLOG_ERROR << "input nullptr map or routing";
    return -1;
  }

  BaseDataTypePtr map_res =
      std::make_shared<hozon::netaos::adf_lite::BaseData>();
  map_res->proto_msg = map;

  SendOutput("map_fusion", map_res);

  BaseDataTypePtr routing_res =
      std::make_shared<hozon::netaos::adf_lite::BaseData>();
  routing_res->proto_msg = routing;

  SendOutput("routing_response", routing_res);

  return 0;
}

}  // namespace common_onboard
}  // namespace perception
}  // namespace hozon
