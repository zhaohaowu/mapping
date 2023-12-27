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
#include "perception-base/base/state_machine/state_machine_info.h"
#include "yaml-cpp/yaml.h"

namespace hozon {
namespace perception {
namespace common_onboard {

int32_t MapFusionLite::AlgInit() {
  std::string default_work_root = "/app/";
  std::string work_root = lib::GetEnv("ADFLITE_ROOT_PATH", default_work_root);
  if (work_root == "") {
    HLOG_ERROR << "ENV: ADFLITE_ROOT_PATH is not set.";
    return -1;
  }
  std::string config_file = work_root +
                            "/runtime_service/mapping/conf/mapping/"
                            "map_fusion/map_fusion.yaml";

  YAML::Node config = YAML::LoadFile(config_file);

  FLAGS_orin_viz = config["orin_viz"].as<bool>();
  FLAGS_orin_viz_addr = config["orin_viz_addr"].as<std::string>();
  FLAGS_map_dir = work_root + config["map_dir"].as<std::string>();
  FLAGS_ehp_monitor = config["ehp_monitor"].as<int32_t>();
  FLAGS_enable_ehp_routing = config["enable_ehp_routing"].as<bool>();
  FLAGS_ehp_log_root_dir = config["ehp_log_root_dir"].as<std::string>();
  FLAGS_map_service_mode = config["map_service_mode"].as<int32_t>();
  FLAGS_topo_rviz = config["topo_rviz"].as<bool>();
  FLAGS_viz_odom_map_in_local = config["viz_odom_map_in_local"].as<bool>();
  FLAGS_output_hd_map = config["output_hd_map"].as<bool>();

  if (FLAGS_orin_viz) {
    HLOG_ERROR << "Start RvizAgent on " << FLAGS_orin_viz_addr;
    int ret = RVIZ_AGENT.Init(FLAGS_orin_viz_addr);
    if (ret < 0) {
      HLOG_ERROR << "RvizAgent start failed";
    }
  }

  mf_ = std::make_unique<MapFusion>();
  int ret = mf_->Init("");
  if (ret < 0) {
    HLOG_INFO << "Init map fusion failed";
    return -1;
  }

  RegistMessageType();
  RegistProcessFunc();

  return 0;
}

void MapFusionLite::AlgRelease() {
  if (mf_) {
    HLOG_ERROR << "try stopping map fusion";
    mf_->Stop();
    HLOG_ERROR << "done stopping map fusion";
  }

  if (RVIZ_AGENT.Ok()) {
    RVIZ_AGENT.Term();
  }
}

void MapFusionLite::RegistMessageType() {
  REGISTER_PROTO_MESSAGE_TYPE("localization",
                              hozon::localization::Localization);
  REGISTER_PROTO_MESSAGE_TYPE("/location/ins_fusion",
                              hozon::localization::HafNodeInfo);
  REGISTER_PROTO_MESSAGE_TYPE("local_map", hozon::mapping::LocalMap);
  // map fusion output message
  REGISTER_PROTO_MESSAGE_TYPE("map_fusion", hozon::navigation_hdmap::MapMsg);
  REGISTER_PROTO_MESSAGE_TYPE("running_mode",
                              hozon::perception::common_onboard::running_mode);
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
  RegistAlgProcessFunc(
      "recv_running_mode",
      std::bind(&MapFusionLite::OnRunningMode, this, std::placeholders::_1));
}

int32_t MapFusionLite::OnLocation(Bundle* input) {
  auto phm_fault = hozon::perception::lib::FaultManager::Instance();
  if (!input) {
    phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::MUL_FRAM_LOCATION_UNUSEFUL,
        hozon::perception::base::FaultStatus::OCCUR,
        hozon::perception::base::SensorOrientation::UNKNOWN, 1, 1000));
    return -1;
  }
  auto p_loc = input->GetOne("localization");
  if (!p_loc) {
    phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::MUL_FRAM_LOCATION_UNUSEFUL,
        hozon::perception::base::FaultStatus::OCCUR,
        hozon::perception::base::SensorOrientation::UNKNOWN, 1, 1000));
    HLOG_ERROR << "nullptr location message";
    return -1;
  }
  const auto loc_res =
      std::static_pointer_cast<hozon::localization::Localization>(
          p_loc->proto_msg);

  if (!loc_res) {
    phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::MUL_FRAM_LOCATION_UNUSEFUL,
        hozon::perception::base::FaultStatus::OCCUR,
        hozon::perception::base::SensorOrientation::UNKNOWN, 1, 1000));
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
  auto phm_fault = hozon::perception::lib::FaultManager::Instance();
  if (!input) {
    phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::
            LOCALMAPPING_LOCATION_INPUT_DATA_ERROR,
        hozon::perception::base::FaultStatus::OCCUR,
        hozon::perception::base::SensorOrientation::UNKNOWN, 1, 1000));
    return -1;
  }
  auto p_local_map = input->GetOne("local_map");
  if (!p_local_map) {
    phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::
            LOCALMAPPING_LOCATION_INPUT_DATA_ERROR,
        hozon::perception::base::FaultStatus::OCCUR,
        hozon::perception::base::SensorOrientation::UNKNOWN, 1, 1000));
    HLOG_ERROR << "nullptr local map message";
    return -1;
  }
  const auto local_map_res = std::static_pointer_cast<hozon::mapping::LocalMap>(
      p_local_map->proto_msg);

  if (!local_map_res) {
    phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::
            LOCALMAPPING_LOCATION_INPUT_DATA_ERROR,
        hozon::perception::base::FaultStatus::OCCUR,
        hozon::perception::base::SensorOrientation::UNKNOWN, 1, 1000));
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
  auto p_loc_plugin = input->GetOne("/location/ins_fusion");
  if (!p_loc_plugin) {
    HLOG_INFO << "nullptr location plugin message";
    return -1;
  }
  const auto loc_plugin_res =
      std::static_pointer_cast<hozon::localization::HafNodeInfo>(
          p_loc_plugin->proto_msg);

  if (!loc_plugin_res) {
    HLOG_ERROR << "nullptr location plugin ";
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
    HLOG_ERROR << "nullptr map fusion";
    return -1;
  }

  std::shared_ptr<hozon::localization::HafNodeInfo> latest_plugin =
      GetLatestLocPlugin();
  if (latest_plugin == nullptr) {
    HLOG_ERROR << "nullptr latest plugin node info";
    return -1;
  }
  std::shared_ptr<hozon::planning::ADCTrajectory> latest_planning = nullptr;
  hozon::routing::RoutingResponse routing;
  mf_->ProcService(latest_plugin, latest_planning, &routing);

  // std::shared_ptr<hozon::routing::RoutingResponse> latest_routing =
  //     std::make_shared<hozon::routing::RoutingResponse>(routing);

  std::shared_ptr<hozon::localization::Localization> latest_loc =
      GetLatestLoc();
  if (latest_loc == nullptr) {
    HLOG_ERROR << "nullptr latest loc";
    return -1;
  }
  std::shared_ptr<hozon::mapping::LocalMap> latest_local_map =
      GetLatestLocalMap();
  if (latest_local_map == nullptr) {
    HLOG_ERROR << "nullptr latest local map";
    return -1;
  }

  auto latest_map = std::make_shared<hozon::hdmap::Map>();
  int ret = mf_->ProcFusion(latest_loc, latest_local_map, latest_map.get());
  if (ret < 0) {
    HLOG_ERROR << "map fusion ProcFusion failed";
    return -1;
  }

  ret = SendFusionResult(latest_loc, latest_map, &routing);
  if (ret < 0) {
    HLOG_ERROR << "SendFusionResult failed";
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
    const std::shared_ptr<hozon::localization::Localization>& location,
    const std::shared_ptr<hozon::hdmap::Map>& map,
    hozon::routing::RoutingResponse* routing) {
  auto phm_fault = hozon::perception::lib::FaultManager::Instance();
  if (map == nullptr || routing == nullptr) {
    phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::
            LOCALMAPPING_MAPFUSION_CAN_NOT_OUPT_LOCAL_MAP,
        hozon::perception::base::FaultStatus::OCCUR,
        hozon::perception::base::SensorOrientation::UNKNOWN, 1, 1000));
    HLOG_ERROR << "input nullptr map or routing";
    return -1;
  }
  auto phm_health = hozon::perception::lib::HealthManager::Instance();
  phm_health->HealthReport(MAKE_HM_TUPLE(
      hozon::perception::base::HmModuleId::MAPPING,
      hozon::perception::base::HealthId::CPID_FUSION_MAP_SEND_MAP_DATA_FPS));
  std::shared_ptr<hozon::navigation_hdmap::MapMsg> map_fusion =
      std::make_shared<hozon::navigation_hdmap::MapMsg>();

  map_fusion->mutable_header()->CopyFrom(location->header());
  map_fusion->mutable_header()->set_frame_id("map_msg");
  map_fusion->mutable_hdmap()->CopyFrom(*map);

  map_fusion->mutable_hdmap()->mutable_header()->mutable_header()->CopyFrom(
      location->header());

  map_fusion->mutable_hdmap()->mutable_header()->mutable_header()->set_frame_id(
      "hd_map");

  routing->mutable_routing_request()->mutable_waypoint()->Clear();
  bool found = false;
  for (auto road_it = routing->road().rbegin();
       road_it != routing->road().rend(); ++road_it) {
    if (road_it->passage_size() > 0) {
      int count = road_it->passage_size() - 1;
      for (const auto& lane : map->lane()) {
        if (lane.id().id() == road_it->passage()[count].segment()[0].id()) {
          auto* waypoints =
              routing->mutable_routing_request()->mutable_waypoint();
          auto* start_point = waypoints->Add();
          start_point->set_id(lane.id().id());
          start_point->set_s(0.0);
          auto* start_pose = start_point->mutable_pose();
          const auto& segments = lane.central_curve().segment();
          auto segment_size = segments.size();
          if (segment_size > 0 && segments[0].line_segment().point_size() > 0) {
            start_pose->set_x(segments[0].line_segment().point()[0].x());
            start_pose->set_y(segments[0].line_segment().point()[0].y());
            start_pose->set_z(0.0);
          }
          start_point->set_type(hozon::routing::LaneWaypointType::NORMAL);

          auto* end_point = waypoints->Add();
          end_point->set_id(lane.id().id());
          end_point->set_s(lane.length());
          auto* end_pose = end_point->mutable_pose();
          if (segment_size > 0 &&
              segments[segment_size - 1].line_segment().point_size() > 0) {
            end_pose->set_x(segments[segment_size - 1]
                                .line_segment()
                                .point()[segments[segment_size - 1]
                                             .line_segment()
                                             .point_size() -
                                         1]
                                .x());
            end_pose->set_y(segments[segment_size - 1]
                                .line_segment()
                                .point()[segments[segment_size - 1]
                                             .line_segment()
                                             .point_size() -
                                         1]
                                .y());
            end_pose->set_z(0.0);
            end_point->set_type(hozon::routing::LaneWaypointType::NORMAL);
            found = true;
            break;
          }
        }
      }
      if (found) {
        break;
      }
    }
  }

  map_fusion->mutable_routing()->CopyFrom(*routing);

  auto map_res = std::make_shared<hozon::netaos::adf_lite::BaseData>();
  map_res->proto_msg = map_fusion;

  SendOutput("map_fusion", map_res);

  return 0;
}

int32_t MapFusionLite::OnRunningMode(hozon::netaos::adf_lite::Bundle* input) {
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
    PauseTrigger("recv_location");
    PauseTrigger("recv_local_map");
    PauseTrigger("recv_loc_plugin");
    PauseTrigger("send_map_fusion");
    // HLOG_ERROR << "!!!!!!!!!!get run mode PARKING";
  } else if (runmode == static_cast<int>(
                            hozon::perception::base::RunningMode::DRIVING) ||
             runmode ==
                 static_cast<int>(hozon::perception::base::RunningMode::ALL)) {
    ResumeTrigger("recv_location");
    ResumeTrigger("recv_local_map");
    ResumeTrigger("recv_loc_plugin");
    ResumeTrigger("send_map_fusion");
    // HLOG_ERROR << "!!!!!!!!!!get run mode DRIVER & UNKNOWN";
  }
  return 0;
}

}  // namespace common_onboard
}  // namespace perception
}  // namespace hozon
