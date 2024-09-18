/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： map_fusion_lite.cc
 *   author     ： xuliang
 *   date       ： 2023.11
 ******************************************************************************/
#include <base/utils/log.h>
#include <common/time/clock.h>
#include <gflags/gflags.h>
#include <perception-lib/lib/environment/environment.h>

#include <set>
#include <string>

#include "modules/map_fusion/include/map_fusion/map_service/global_hd_map.h"
#include "modules/util/include/util/rviz_agent/rviz_agent.h"
#include "onboard/onboard_lite/map_fusion/map_fusion_config_lite.h"
#include "onboard/onboard_lite/map_fusion/map_fusion_lite.h"
#include "perception-base/base/state_machine/state_machine_info.h"
// #include "proto/fsm/function_manager.pb.h"
#include "proto/routing/nav_data.pb.h"
#include "yaml-cpp/yaml.h"

namespace hozon {
namespace perception {
namespace common_onboard {

int32_t MapFusionLite::AlgInit() {
  // #ifdef ISORIN
  //  hozon::netaos::log::InitLogging(
  //      "mapping", "mapping", hozon::netaos::log::LogLevel::kDebug,
  //      hozon::netaos::log::HZ_LOG2FILE, "/opt/usr/log/soc_log/", 10,
  //      (20 * 1024 * 1024), true);
  // #endif
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
  FLAGS_ldmap_dir = work_root + config["ldmap_dir"].as<std::string>();
  FLAGS_ehp_monitor = config["ehp_monitor"].as<int32_t>();
  FLAGS_enable_ehp_routing = config["enable_ehp_routing"].as<bool>();
  FLAGS_ehp_log_root_dir = config["ehp_log_root_dir"].as<std::string>();
  FLAGS_map_service_mode = config["map_service_mode"].as<int32_t>();
  FLAGS_topo_rviz = config["topo_rviz"].as<bool>();
  FLAGS_viz_odom_map_in_local = config["viz_odom_map_in_local"].as<bool>();
  FLAGS_output_hd_map = config["output_hd_map"].as<bool>();
  FLAGS_service_update_interval =
      config["service_update_interval"].as<double>();
  FLAGS_work_mode = config["work_mode"].as<std::string>();

  if (FLAGS_orin_viz) {
    HLOG_ERROR << "Start RvizAgent on " << FLAGS_orin_viz_addr;
    int ret = RVIZ_AGENT.Init(FLAGS_orin_viz_addr);
    if (ret < 0) {
      HLOG_ERROR << "RvizAgent start failed";
    }
  }

  curr_routing_ = std::make_shared<hozon::routing::RoutingResponse>();
  curr_routing_->Clear();
  mf_ = std::make_unique<MapFusion>();
  int ret = mf_->Init(config);
  if (ret < 0) {
    HLOG_INFO << "Init map fusion failed";
    return -1;
  }
  map_select_ = std::make_unique<MapSelectLite>();
  if (!map_select_->Init()) {
    HLOG_INFO << "Init map select failed";
    return -1;
  }
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
  REGISTER_PROTO_MESSAGE_TYPE("percep_transport",
                              hozon::perception::TransportElement);
  REGISTER_PROTO_MESSAGE_TYPE("dr", hozon::dead_reckoning::DeadReckoning);
  REGISTER_PROTO_MESSAGE_TYPE("function_manager_in",
                              hozon::functionmanager::FunctionManagerIn);
  REGISTER_PROTO_MESSAGE_TYPE("dv_nav_data", hozon::hmi::NAVDataService);
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
  RegistAlgProcessFunc("recv_percep_transport",
                       std::bind(&MapFusionLite::OnPercepTransport, this,
                                 std::placeholders::_1));
  RegistAlgProcessFunc(
      "recv_dr", std::bind(&MapFusionLite::OnDR, this, std::placeholders::_1));
  RegistAlgProcessFunc(
      "recv_dv_nav_data",
      std::bind(&MapFusionLite::OnNavData, this, std::placeholders::_1));
  RegistAlgProcessFunc("recv_fct_in", std::bind(&MapFusionLite::OnFCTIn, this,
                                                std::placeholders::_1));
}

int32_t MapFusionLite::OnNavData(Bundle* input) {
  if (!input) {
    HLOG_ERROR << "route: OnNavData input is nullptr";
    return -1;
  }
  auto p_nav_data = input->GetOne("dv_nav_data");
  if (!p_nav_data) {
    HLOG_ERROR << "OnNavData GetOne error ";
    return -1;
  }
  HLOG_WARN << "route: OnNavData ======================================";

  const auto nav_data_msg =
      std::static_pointer_cast<hozon::hmi::NAVDataService>(
          p_nav_data->proto_msg);
  if (!nav_data_msg) {
    HLOG_ERROR << "route: nav_data_msg is nullptr ";
    return -1;
  }

  {
    std::lock_guard<std::mutex> lock(hmi_nav_mtx_);
    hmi_nav_data_ = std::make_shared<hozon::hmi::NAVDataService>(*nav_data_msg);
    HLOG_ERROR << "route: start to UpdateHMINavData ";
    mf_->UpdateHMINavData(hmi_nav_data_);
  }

  return 0;
}

int32_t MapFusionLite::OnLocation(Bundle* input) {
  static int location_unavailable_frames = 0;
  static bool location_unavailable_flags = false;
  static bool location_error_flags = false;
  auto phm_fault = hozon::perception::lib::FaultManager::Instance();
  if (!input) {
    location_unavailable_frames += 1;
    return -1;
  } else {
    location_unavailable_frames = 0;
  }
  if (location_unavailable_frames >= 5) {
    phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::MUL_FRAM_LOCATION_UNUSEFUL,
        hozon::perception::base::FaultStatus::OCCUR,
        hozon::perception::base::SensorOrientation::UNKNOWN, 1, 1000));
    location_unavailable_flags = true;
    HLOG_ERROR << "Consecutive frames with unavailable location";
  } else {
    if (location_unavailable_flags) {
      phm_fault->Report(MAKE_FM_TUPLE(
          hozon::perception::base::FmModuleId::MAPPING,
          hozon::perception::base::FaultType::MUL_FRAM_LOCATION_UNUSEFUL,
          hozon::perception::base::FaultStatus::RESET,
          hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
      location_unavailable_flags = false;
    }
  }
  auto p_loc = input->GetOne("localization");
  if (!p_loc) {
    HLOG_ERROR << "nullptr location message";
    return -1;
  }
  const auto loc_res =
      std::static_pointer_cast<hozon::localization::Localization>(
          p_loc->proto_msg);

  if (!loc_res) {
    HLOG_ERROR << "nullptr location";
    return -1;
  }
  static std::shared_ptr<hozon::localization::Localization> prev_loc_res =
      nullptr;
  prev_loc_res = curr_loc_;
  {
    std::lock_guard<std::mutex> lock(loc_mtx_);
    curr_loc_ = std::make_shared<hozon::localization::Localization>(*loc_res);
  }
  if (prev_loc_res != nullptr && curr_loc_ != nullptr) {
    auto loc_distance =
        std::sqrt(std::pow(curr_loc_->pose_local().position().x() -
                               prev_loc_res->pose_local().position().x(),
                           2.0) +
                  std::pow(curr_loc_->pose_local().position().y() -
                               prev_loc_res->pose_local().position().y(),
                           2.0) +
                  std::pow(curr_loc_->pose_local().position().z() -
                               prev_loc_res->pose_local().position().z(),
                           2.0));
    auto pre_loc_quaternion = prev_loc_res->pose_local().quaternion();
    auto cur_loc_quaternion = curr_loc_->pose_local().quaternion();
    Eigen::Quaterniond pre_quaternion(
        pre_loc_quaternion.w(), pre_loc_quaternion.x(), pre_loc_quaternion.y(),
        pre_loc_quaternion.z());
    Eigen::Quaterniond cur_quaternion(
        cur_loc_quaternion.w(), cur_loc_quaternion.x(), cur_loc_quaternion.y(),
        cur_loc_quaternion.z());
    auto pre_yaw = Qat2EulerAngle(pre_quaternion).z();
    auto cur_yaw = Qat2EulerAngle(cur_quaternion).z();
    auto loc_yaw = pre_yaw - cur_yaw;
    if (loc_yaw > M_PI) {
      loc_yaw -= 2 * M_PI;
    }
    if (loc_yaw < -M_PI) {
      loc_yaw += 2 * M_PI;
    }
    loc_yaw = std::abs(loc_yaw);
    // 前后帧定位距离大于 1.2m，或者yaw角大于0.04时候认为位置和姿态发生突变
    if (loc_distance > 1.2 || loc_yaw > 0.06) {
      phm_fault->Report(MAKE_FM_TUPLE(
          hozon::perception::base::FmModuleId::MAPPING,
          hozon::perception::base::FaultType::LOCATION_POS_ATTITUDE_ERROR,
          hozon::perception::base::FaultStatus::OCCUR,
          hozon::perception::base::SensorOrientation::UNKNOWN, 5, 100));
      location_error_flags = true;
      HLOG_ERROR << "Location pos error";
    } else {
      if (location_error_flags) {
        phm_fault->Report(MAKE_FM_TUPLE(
            hozon::perception::base::FmModuleId::MAPPING,
            hozon::perception::base::FaultType::LOCATION_POS_ATTITUDE_ERROR,
            hozon::perception::base::FaultStatus::RESET,
            hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
        location_error_flags = false;
      }
    }
  }

  return 0;
}

int32_t MapFusionLite::OnLocalMap(Bundle* input) {
  auto phm_fault = hozon::perception::lib::FaultManager::Instance();
  static bool localmapping_cm_errror_flags = false;
  static bool nullptr_localmapping_message = false;
  if (!input) {
    phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::LOCALMAPPING_LOCATION_CM_ERROR,
        hozon::perception::base::FaultStatus::OCCUR,
        hozon::perception::base::SensorOrientation::UNKNOWN, 4, 1000));
    localmapping_cm_errror_flags = true;
    return -1;
  } else {
    if (localmapping_cm_errror_flags) {
      phm_fault->Report(MAKE_FM_TUPLE(
          hozon::perception::base::FmModuleId::MAPPING,
          hozon::perception::base::FaultType::LOCALMAPPING_LOCATION_CM_ERROR,
          hozon::perception::base::FaultStatus::RESET,
          hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
      localmapping_cm_errror_flags = false;
    }
  }
  auto p_local_map = input->GetOne("local_map");
  if (!p_local_map) {
    phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::
            LOCALMAPPING_LOCATION_INPUT_DATA_ERROR,
        hozon::perception::base::FaultStatus::OCCUR,
        hozon::perception::base::SensorOrientation::UNKNOWN, 4, 1000));
    nullptr_localmapping_message = true;
    HLOG_ERROR << "nullptr local map message";
    return -1;
  } else {
    if (nullptr_localmapping_message) {
      phm_fault->Report(MAKE_FM_TUPLE(
          hozon::perception::base::FmModuleId::MAPPING,
          hozon::perception::base::FaultType::
              LOCALMAPPING_LOCATION_INPUT_DATA_ERROR,
          hozon::perception::base::FaultStatus::RESET,
          hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
      nullptr_localmapping_message = false;
    }
  }
  const auto local_map_res = std::static_pointer_cast<hozon::mapping::LocalMap>(
      p_local_map->proto_msg);

  if (!local_map_res) {
    phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::
            LOCALMAPPING_LOCATION_INPUT_DATA_ERROR,
        hozon::perception::base::FaultStatus::OCCUR,
        hozon::perception::base::SensorOrientation::UNKNOWN, 4, 1000));
    nullptr_localmapping_message = true;
    HLOG_ERROR << "nullptr local map";
    return -1;
  } else {
    if (nullptr_localmapping_message) {
      phm_fault->Report(MAKE_FM_TUPLE(
          hozon::perception::base::FmModuleId::MAPPING,
          hozon::perception::base::FaultType::
              LOCALMAPPING_LOCATION_INPUT_DATA_ERROR,
          hozon::perception::base::FaultStatus::RESET,
          hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
      nullptr_localmapping_message = false;
    }
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
std::shared_ptr<hozon::hmi::NAVDataService>
MapFusionLite::GetLatestHMINavData() {
  std::shared_ptr<hozon::hmi::NAVDataService> latest_nav_data = nullptr;
  std::lock_guard<std::mutex> lock(hmi_nav_mtx_);
  if (hmi_nav_data_ != nullptr) {
    latest_nav_data =
        std::make_shared<hozon::hmi::NAVDataService>(*hmi_nav_data_);
  }
  return latest_nav_data;
}
int32_t MapFusionLite::OnFCTIn(Bundle* input) {
  if (!input) {
    return -1;
  }
  auto f_ct = input->GetOne("function_manager_in");
  if (!f_ct) {
    HLOG_ERROR << "nullptr function_manager_in message";
    return -1;
  }
  const auto fct_res =
      std::static_pointer_cast<hozon::functionmanager::FunctionManagerIn>(
          f_ct->proto_msg);

  if (!fct_res) {
    HLOG_ERROR << "nullptr function_manager_in";
    return -1;
  }
  {
    std::lock_guard<std::mutex> lock(fct_mtx_);
    curr_fct_in_ =
        std::make_shared<hozon::functionmanager::FunctionManagerIn>(*fct_res);
  }
  return 0;
}

int32_t MapFusionLite::OnPercepTransport(Bundle* input) {
  auto phm_fault = hozon::perception::lib::FaultManager::Instance();
  if (!input) {
    return -1;
  }
  auto p_per = input->GetOne("percep_transport");
  if (!p_per) {
    HLOG_ERROR << "nullptr percep_transport message";
    return -1;
  }
  const auto percep_res =
      std::static_pointer_cast<hozon::perception::TransportElement>(
          p_per->proto_msg);

  if (!percep_res) {
    HLOG_ERROR << "nullptr percep_transport";
    return -1;
  }
  {
    std::lock_guard<std::mutex> lock(percep_map_mtx_);
    curr_percep_ =
        std::make_shared<hozon::perception::TransportElement>(*percep_res);
  }
  return 0;
}

int32_t MapFusionLite::OnDR(Bundle* input) {
  auto phm_fault = hozon::perception::lib::FaultManager::Instance();
  if (!input) {
    return -1;
  }
  auto p_dr = input->GetOne("dr");
  if (!p_dr) {
    HLOG_ERROR << "nullptr dr message";
    return -1;
  }
  const auto dr_res =
      std::static_pointer_cast<hozon::dead_reckoning::DeadReckoning>(
          p_dr->proto_msg);

  if (!dr_res) {
    HLOG_ERROR << "nullptr dr";
    return -1;
  }
  {
    std::lock_guard<std::mutex> lock(dr_mtx_);
    curr_dr_ = std::make_shared<hozon::dead_reckoning::DeadReckoning>(*dr_res);
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
  static double last = -1;
  auto now = hozon::common::Clock::NowInSeconds() * 1000;
  bool global_hd_updated = false;
  if (last < 0 || now - last > FLAGS_service_update_interval) {
    auto phm_health = hozon::perception::lib::HealthManager::Instance();
    phm_health->HealthReport(MAKE_HM_TUPLE(
        hozon::perception::base::HmModuleId::MAPPING,
        hozon::perception::base::HealthId::CPID_MAP_SEND_HD_AND_HQ_FPS));
    mf_->ProcService(latest_plugin, latest_planning, curr_routing_.get());
    global_hd_updated = true;
    last = now;
  }
  std::shared_ptr<hozon::localization::Localization> latest_loc =
      GetLatestLoc();
  if (latest_loc == nullptr) {
    HLOG_ERROR << "nullptr latest loc";
    return -1;
  }
  std::shared_ptr<hozon::mapping::LocalMap> latest_local_map =
      GetLatestLocalMap();
  // if (latest_local_map == nullptr) {
  //   HLOG_ERROR << "nullptr latest local map";
  //   return -1;
  // }

  std::shared_ptr<hozon::perception::TransportElement> latest_percep =
      GetLatestPercep();
  // if (latest_percep == nullptr) {
  //   HLOG_ERROR << "nullptr latest latest_percep";
  //   return -1;
  // }

  std::shared_ptr<hozon::dead_reckoning::DeadReckoning> latest_dr =
      GetLatestDR();
  // if (latest_percep == nullptr) {
  //   HLOG_ERROR << "nullptr latest latest_dr";
  //   return -1;
  // }

  std::shared_ptr<hozon::functionmanager::FunctionManagerIn> latest_fct_in =
      GetLatestFCTIn();
  // if (curr_fct_in_ == nullptr) {
  //   HLOG_ERROR << "nullptr latest curr_fct_in_";
  //   return -1;
  // }

  if (!map_select_) {
    HLOG_ERROR << "nullptr map select";
    return -1;
  }
  std::set<std::string> legal_mode = {
      kWorkModeFusionMap,
      kWorkModePercepMap,
      kWorkModeFusionAndPercepMap,
  };
  if (legal_mode.find(FLAGS_work_mode) == legal_mode.end()) {
    HLOG_ERROR << "Invalid work mode: " << FLAGS_work_mode;
    return -1;
  }

  std::shared_ptr<hozon::hdmap::Map> latest_fusion_map = nullptr;
  std::shared_ptr<hozon::hdmap::Map> latest_percep_map = nullptr;
  std::shared_ptr<hozon::routing::RoutingResponse> latest_percep_routing =
      nullptr;
  // auto latest_map = std::make_shared<hozon::hdmap::Map>();
  // auto percep_map = std::make_shared<hozon::hdmap::Map>();
  static WorkMode pre_word_mode = WorkMode::PerceptMap;
  if (FLAGS_work_mode == kWorkModeFusionMap ||
      FLAGS_work_mode == kWorkModeFusionAndPercepMap) {
    work_mode_ = WorkMode::FusionMap;
    if (work_mode_ != pre_word_mode) {
      HLOG_INFO << "turn to FusionMap Mode...";
    }
    std::shared_ptr<hozon::hdmap::Map> fusion_map = nullptr;
    int ret =
        mf_->ProcFusion(latest_plugin, latest_loc, latest_local_map,
                        global_hd_updated, fusion_map, curr_routing_.get());
    if (ret < 0 || fusion_map == nullptr) {
      HLOG_ERROR << "map fusion ProcFusion failed";
    } else {
      latest_fusion_map = fusion_map;
    }
  }

  if (FLAGS_work_mode == kWorkModePercepMap ||
      FLAGS_work_mode == kWorkModeFusionAndPercepMap) {
    // HLOG_INFO << "ProcPercep";
    work_mode_ = WorkMode::PerceptMap;
    if (work_mode_ != pre_word_mode) {
      HLOG_INFO << "turn to PerceptMap Mode...";
    }
    auto percep_map = std::make_shared<hozon::hdmap::Map>();
    percep_map->Clear();
    auto percep_routing = std::make_shared<hozon::routing::RoutingResponse>();
    int ret = mf_->ProcPercep(latest_loc, latest_local_map, percep_map.get(),
                              percep_routing.get());
    if (ret < 0) {
      HLOG_ERROR << "map fusion ProcPercep failed";
      percep_map->Clear();
    } else {
      latest_percep_map = percep_map;
      latest_percep_routing = percep_routing;
    }
  }

  pre_word_mode = work_mode_;

  static mp::mf::select::MapSelectResult pre_map_type = {
      hozon::navigation_hdmap::MapMsg_MapType_INVALID, false, 2};
  auto fault = mf_->GetMapServiceFault();
  MapServiceFaultOutput(fault);
  {
    std::lock_guard<std::mutex> lock(process_mtx_);
    HLOG_DEBUG << ">>>>>>>>>>>START<<<<<<<<<<<<<<<<";
    curr_map_type_ = map_select_->Process(latest_loc, latest_fusion_map,
                                          latest_percep_map, latest_fct_in);
    if (select_debug_) {
      DebugSelectMap();
    }
    if (pre_map_type.map_type != curr_map_type_.map_type ||
        pre_map_type.valid != curr_map_type_.valid) {
      HLOG_INFO << "select map type turn to:" << curr_map_type_.map_type << ", "
                << curr_map_type_.valid;
    }

    pre_map_type = curr_map_type_;
    if (curr_map_type_.map_type ==
            hozon::navigation_hdmap::MapMsg_MapType_FUSION_NNP_MAP ||
        curr_map_type_.map_type ==
            hozon::navigation_hdmap::MapMsg_MapType_FUSION_NCP_MAP) {
      if (0 == FLAGS_map_service_mode) {
        curr_map_type_.map_type =
            hozon::navigation_hdmap::MapMsg_MapType_FUSION_NCP_MAP;
      }
      int ret = SendFusionResult(latest_loc, latest_fusion_map, curr_map_type_,
                                 curr_routing_.get());
      if (ret < 0) {
        HLOG_ERROR << "SendFusionResult failed";
        latest_fusion_map->Clear();
      }
    } else if (curr_map_type_.map_type ==
               hozon::navigation_hdmap::MapMsg_MapType_PERCEP_MAP) {
      int ret = SendPercepResult(latest_loc, latest_percep_map, curr_map_type_,
                                 latest_percep_routing);
      if (ret < 0) {
        HLOG_ERROR << "SendPercepResult failed";
      }
    } else if (curr_map_type_.map_type ==
               hozon::navigation_hdmap::MapMsg_MapType_INVALID) {
      if (latest_fusion_map == nullptr) {
        latest_fusion_map = std::make_shared<hozon::hdmap::Map>();
      }
      if (!latest_fusion_map->lane().empty()) {
        latest_fusion_map = std::make_shared<hozon::hdmap::Map>();
        curr_routing_ = std::make_shared<hozon::routing::RoutingResponse>();
      }
      int ret = SendFusionResult(latest_loc, latest_fusion_map, curr_map_type_,
                                 curr_routing_.get());
      if (ret < 0) {
        HLOG_ERROR << "SendFusionResult failed during invalid map";
        latest_fusion_map->Clear();
      }
    } else {
      HLOG_ERROR << "map select return wrong map type:"
                 << curr_map_type_.map_type << ", " << curr_map_type_.valid
                 << ", " << curr_map_type_.fault_level;
    }

    MapFusionOutputEvaluation(latest_loc);
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
    auto local_map_size = curr_local_map_->ByteSizeLong();
    auto one_mb = static_cast<size_t>(1 * 1024 * 1024);
    if (local_map_size > one_mb) {
      HLOG_ERROR << "LocalMap size is greater than 1MB";
    }
    latest_local_map = curr_local_map_;
  }
  return latest_local_map;
}
int MapFusionLite::DebugSelectMap() {
  auto debug_result = std::make_shared<hozon::navigation_hdmap::MapMsg>();
  debug_result->mutable_routing()->CopyFrom(map_select_->GetDebug());
  auto msg = std::make_shared<hozon::netaos::adf_lite::BaseData>();
  msg->proto_msg = debug_result;
  SendOutput("map_select_dbg", msg);
  return 0;
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
    mp::mf::select::MapSelectResult select,
    hozon::routing::RoutingResponse* routing) {
  auto phm_fault = hozon::perception::lib::FaultManager::Instance();
  static bool input_nullptr_map_or_routing_flags = false;
  if (map == nullptr || routing == nullptr) {
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

  // // 裁剪地图
  // std::unordered_set<std::string> routing_setction;
  // for (const auto& road_it : routing->road()) {
  //   routing_setction.emplace(road_it.id());
  // }

  // std::unordered_set<std::string> routing_lane;
  // for (auto& road_it : *map->mutable_road()) {
  //   for (auto road_section_it = (*road_it.mutable_section()).begin();
  //        road_section_it != (*road_it.mutable_section()).end();) {
  //     if (routing_setction.find((*road_section_it).id().id()) !=
  //         routing_setction.end()) {
  //       for (const auto& lane_it : (*road_section_it).lane_id()) {
  //         routing_lane.emplace(lane_it.id());
  //       }
  //       ++road_section_it;
  //     } else {
  //       road_section_it =
  //       (*road_it.mutable_section()).erase(road_section_it);
  //     }
  //   }
  // }

  // // 删除对应lane
  // for (auto lane_it = (*map->mutable_lane()).begin();
  //      lane_it != (*map->mutable_lane()).end();) {
  //   if (routing_lane.find((*lane_it).id().id()) != routing_lane.end()) {
  //     ++lane_it;
  //   } else {
  //     lane_it = (*map->mutable_lane()).erase(lane_it);
  //   }
  // }

  map_fusion->mutable_hdmap()->CopyFrom(*map);

  map_fusion->mutable_hdmap()->mutable_header()->mutable_header()->CopyFrom(
      location->header());

  map_fusion->mutable_hdmap()->mutable_header()->mutable_header()->set_frame_id(
      "hd_map");
  if (!FLAGS_output_hd_map) {
    routing->mutable_routing_request()->mutable_waypoint()->Clear();
    bool found_start = false;
    bool found_end = false;
    for (auto road_it = routing->road().rbegin();
         road_it != routing->road().rend(); ++road_it) {
      if (road_it->passage_size() > 0) {
        int count = road_it->passage_size() - 1;
        for (const auto& lane : map->lane()) {
          if (lane.id().id() == road_it->passage()[count].segment()[0].id()) {
            const auto& segments = lane.central_curve().segment();
            auto segment_size = segments.size();
            auto* waypoints =
                routing->mutable_routing_request()->mutable_waypoint();
            if (segment_size > 0 &&
                segments[0].line_segment().point_size() > 0) {
              auto* start_point = waypoints->Add();
              start_point->set_id(lane.id().id());
              start_point->set_s(0.0);
              auto* start_pose = start_point->mutable_pose();
              start_pose->set_x(segments[0].line_segment().point()[0].x());
              start_pose->set_y(segments[0].line_segment().point()[0].y());
              start_pose->set_z(0.0);
              start_point->set_type(hozon::routing::LaneWaypointType::NORMAL);
              found_start = true;
            }

            if (segment_size > 0 &&
                segments[segment_size - 1].line_segment().point_size() > 0) {
              auto* end_point = waypoints->Add();
              end_point->set_id(lane.id().id());
              end_point->set_s(lane.length());
              auto* end_pose = end_point->mutable_pose();
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
              found_end = true;
              break;
            }
          }
        }
        if (found_start && found_end) {
          break;
        }
      }
    }
  }
  map_fusion->mutable_routing()->CopyFrom(*routing);

  map_fusion->mutable_routing()->mutable_header()->set_publish_stamp(
      location->header().publish_stamp());
  std::string switch_reason = map_select_->GetSwitchMapReason();
  map_fusion->mutable_routing()->add_origin_response(switch_reason);
  if (FLAGS_map_service_mode == 0 || FLAGS_map_service_mode == 2) {
    map_fusion->mutable_routing()->mutable_header()->set_seq(
        location->header().seq());
  }
  map_fusion->set_map_type(select.map_type);
  map_fusion->set_is_valid(select.valid);
  map_fusion->set_fault_level(select.fault_level);

  auto map_res = std::make_shared<hozon::netaos::adf_lite::BaseData>();
  map_res->proto_msg = map_fusion;

  SendOutput("map_fusion", map_res);

  return 0;
}

int MapFusionLite::SendPercepResult(
    const std::shared_ptr<hozon::localization::Localization>& location,
    const std::shared_ptr<hozon::hdmap::Map>& map,
    mp::mf::select::MapSelectResult select,
    const std::shared_ptr<hozon::routing::RoutingResponse>& routing) {
  auto percep_result = std::make_shared<hozon::navigation_hdmap::MapMsg>();
  percep_result->mutable_header()->CopyFrom(location->header());
  percep_result->mutable_header()->set_frame_id("percep_map");
  percep_result->mutable_hdmap()->CopyFrom(*map);
  percep_result->mutable_hdmap()->mutable_header()->mutable_header()->CopyFrom(
      location->header());
  percep_result->mutable_hdmap()
      ->mutable_header()
      ->mutable_header()
      ->set_frame_id("percep_map");
  percep_result->mutable_routing()->CopyFrom(*routing);
  percep_result->mutable_routing()->mutable_header()->CopyFrom(
      location->header());
  percep_result->mutable_routing()->mutable_header()->set_frame_id(
      "percep_map");
  std::string switch_reason = map_select_->GetSwitchMapReason();
  if (curr_routing_.get() == nullptr) {
    switch_reason = "fusion map routing from mapservice is nullptr";
  } else {
    if (curr_routing_.get()->road().empty()) {
      switch_reason = "the road of fusion map routing from mapservice is empty";
    }
  }
  if (curr_routing_.get() != nullptr) {
    if (curr_routing_.get()->has_ehp_reason()) {
      std::string::size_type idx =
          curr_routing_.get()->ehp_reason().find("finish");
      if (idx != std::string::npos) {
        select.fault_level = 2;
        switch_reason = "reason message is 3, Route finish";
      }
      percep_result->mutable_routing()->set_ehp_reason(
          curr_routing_.get()->ehp_reason());
    }
  }
  percep_result->mutable_routing()->add_origin_response(switch_reason);
  percep_result->set_map_type(select.map_type);
  percep_result->set_is_valid(select.valid);
  percep_result->set_fault_level(select.fault_level);

  auto msg = std::make_shared<hozon::netaos::adf_lite::BaseData>();
  msg->proto_msg = percep_result;
  // SendOutput("map_percep", msg);
  auto phm_health = hozon::perception::lib::HealthManager::Instance();
  phm_health->HealthReport(MAKE_HM_TUPLE(
      hozon::perception::base::HmModuleId::MAPPING,
      hozon::perception::base::HealthId::CPID_FUSION_MAP_SEND_MAP_DATA_FPS));
  SendOutput("map_fusion", msg);
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
  static int last_runmode =
      static_cast<int>(hozon::perception::base::RunningMode::DRIVING);
  if (runmode ==
      static_cast<int>(hozon::perception::base::RunningMode::PARKING)) {
    if (last_runmode != runmode) {
      PauseTrigger("recv_location");
      PauseTrigger("recv_local_map");
      PauseTrigger("recv_loc_plugin");
      PauseTrigger("send_map_fusion");
      HLOG_INFO << "!!!!!!!!!!get run mode PARKING";
      last_runmode = runmode;
    }
    // HLOG_ERROR << "!!!!!!!!!!get run mode PARKING";
  } else if (runmode == static_cast<int>(
                            hozon::perception::base::RunningMode::DRIVING) ||
             runmode ==
                 static_cast<int>(hozon::perception::base::RunningMode::ALL)) {
    if (last_runmode != runmode) {
      ResumeTrigger("recv_location");
      ResumeTrigger("recv_local_map");
      ResumeTrigger("recv_loc_plugin");
      ResumeTrigger("send_map_fusion");
      HLOG_INFO << "!!!!!!!!!!get run mode DRIVER & UNKNOWN";
      last_runmode = runmode;
    }
    // HLOG_ERROR << "!!!!!!!!!!get run mode DRIVER & UNKNOWN";
  }
  return 0;
}

std::shared_ptr<hozon::perception::TransportElement>
MapFusionLite::GetLatestPercep() {
  std::shared_ptr<hozon::perception::TransportElement> latest_percep = nullptr;
  std::lock_guard<std::mutex> lock(percep_map_mtx_);
  if (curr_percep_ != nullptr) {
    latest_percep =
        std::make_shared<hozon::perception::TransportElement>(*curr_percep_);
  }
  return latest_percep;
}
std::shared_ptr<hozon::dead_reckoning::DeadReckoning>
MapFusionLite::GetLatestDR() {
  std::shared_ptr<hozon::dead_reckoning::DeadReckoning> latest_dr = nullptr;
  std::lock_guard<std::mutex> lock(dr_mtx_);
  if (curr_dr_ != nullptr) {
    latest_dr =
        std::make_shared<hozon::dead_reckoning::DeadReckoning>(*curr_dr_);
  }
  return latest_dr;
}
std::shared_ptr<hozon::functionmanager::FunctionManagerIn>
MapFusionLite::GetLatestFCTIn() {
  std::shared_ptr<hozon::functionmanager::FunctionManagerIn> latest_fct_in =
      nullptr;
  std::lock_guard<std::mutex> lock(fct_mtx_);
  if (curr_fct_in_ != nullptr) {
    latest_fct_in = std::make_shared<hozon::functionmanager::FunctionManagerIn>(
        *curr_fct_in_);
  }
  return latest_fct_in;
}

int MapFusionLite::MapFusionOutputEvaluation(
    const std::shared_ptr<hozon::localization::Localization>& location) {
  auto phm_fault = hozon::perception::lib::FaultManager::Instance();
  static double pre_publish_stamp = 0;
  static bool map_fusion_outputs_abnormal_flags = false;
  auto cur_publish_stamp = location->header().publish_stamp();
  static double time_stamp_delta = 0;
  static double time_stamp_start = 0;
  if (pre_publish_stamp != 0 &&
      (cur_publish_stamp - pre_publish_stamp > time_stamp_delta)) {
    time_stamp_delta = cur_publish_stamp - pre_publish_stamp;
  }

  // 10分钟打印一次
  if (pre_publish_stamp != 0 && (cur_publish_stamp - time_stamp_start > 600)) {
    time_stamp_start = cur_publish_stamp;
    HLOG_INFO << "mapfusion publish time_stamp delta max:"
              << std::to_string(time_stamp_delta);
  }
  // 时间戳可容忍程度
  auto epsilon = 0.05;

  if (pre_publish_stamp != 0 &&
      cur_publish_stamp - pre_publish_stamp > (0.1 + epsilon)) {
    phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::
            LOCALMAPPING_MAPFUSION_OUTPUT_MAP_DATA_ERROR,
        hozon::perception::base::FaultStatus::OCCUR,
        hozon::perception::base::SensorOrientation::UNKNOWN, 4, 1000));
    HLOG_ERROR << "Map fusion outputs abnormal time difference between frames"
               << " ,cur_publish_stamp:" << std::to_string(cur_publish_stamp)
               << " ,pre_publish_stamp:" << std::to_string(pre_publish_stamp)
               << " ,delta publish_stamp:"
               << std::to_string(cur_publish_stamp - pre_publish_stamp);
    pre_publish_stamp = cur_publish_stamp;
    map_fusion_outputs_abnormal_flags = true;

    return -1;
  } else {
    if (map_fusion_outputs_abnormal_flags) {
      phm_fault->Report(MAKE_FM_TUPLE(
          hozon::perception::base::FmModuleId::MAPPING,
          hozon::perception::base::FaultType::
              LOCALMAPPING_MAPFUSION_OUTPUT_MAP_DATA_ERROR,
          hozon::perception::base::FaultStatus::RESET,
          hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
      map_fusion_outputs_abnormal_flags = false;
    }
  }
  pre_publish_stamp = cur_publish_stamp;

  return 0;
}

int MapFusionLite::MapServiceFaultOutput(
    const hozon::mp::mf::MapServiceFault& fault) {
  auto phm_fault = hozon::perception::lib::FaultManager::Instance();
  static bool hd_map_input_loc_error = false;
  static bool hd_map_ehp_init_error = false;
  static bool hd_map_data_rom_error = false;
  static bool hd_map_uuid_error = false;
  static bool hd_map_sdk_input_error = false;
  static bool hd_map_data_path_error = false;
  static bool hd_map_path_rw_error = false;
  static bool hd_map_active_error = false;
  static bool hd_map_init_fail = false;
  int fault_value = static_cast<int>(fault);
  HLOG_INFO << "Map service fault:" << pre_fault_value_ << "," << fault_value;
  if (fault_value != pre_fault_value_ && pre_fault_value_ != -1) {
    if (hd_map_input_loc_error) {
      phm_fault->Report(MAKE_FM_TUPLE(
          hozon::perception::base::FmModuleId::MAPPING,
          hozon::perception::base::FaultType::HD_MAP_INPUT_LOC_ERROR,
          hozon::perception::base::FaultStatus::RESET,
          hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
      hd_map_input_loc_error = false;
    }
    if (hd_map_ehp_init_error) {
      phm_fault->Report(MAKE_FM_TUPLE(
          hozon::perception::base::FmModuleId::MAPPING,
          hozon::perception::base::FaultType::HD_MAP_EHP_INIT_ERROR,
          hozon::perception::base::FaultStatus::RESET,
          hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
      hd_map_ehp_init_error = false;
    }
    if (hd_map_data_rom_error) {
      phm_fault->Report(MAKE_FM_TUPLE(
          hozon::perception::base::FmModuleId::MAPPING,
          hozon::perception::base::FaultType::HD_MAP_MAP_DATA_ROM_ERROR,
          hozon::perception::base::FaultStatus::RESET,
          hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
      hd_map_data_rom_error = false;
    }
    if (hd_map_uuid_error) {
      phm_fault->Report(MAKE_FM_TUPLE(
          hozon::perception::base::FmModuleId::MAPPING,
          hozon::perception::base::FaultType::HD_MAP_UUID_ERROR,
          hozon::perception::base::FaultStatus::RESET,
          hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
      hd_map_uuid_error = false;
    }
    if (hd_map_sdk_input_error) {
      phm_fault->Report(MAKE_FM_TUPLE(
          hozon::perception::base::FmModuleId::MAPPING,
          hozon::perception::base::FaultType::HD_MAP_SDK_INNER_ERROR,
          hozon::perception::base::FaultStatus::RESET,
          hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
      hd_map_sdk_input_error = false;
    }
    if (hd_map_data_path_error) {
      phm_fault->Report(MAKE_FM_TUPLE(
          hozon::perception::base::FmModuleId::MAPPING,
          hozon::perception::base::FaultType::HD_MAP_MAP_DATA_PATH_ERROR,
          hozon::perception::base::FaultStatus::RESET,
          hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
      hd_map_data_path_error = false;
    }
    if (hd_map_path_rw_error) {
      phm_fault->Report(MAKE_FM_TUPLE(
          hozon::perception::base::FmModuleId::MAPPING,
          hozon::perception::base::FaultType::HD_MAP_PATH_RW_ERROR,
          hozon::perception::base::FaultStatus::RESET,
          hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
      hd_map_path_rw_error = false;
    }
    if (hd_map_active_error) {
      phm_fault->Report(MAKE_FM_TUPLE(
          hozon::perception::base::FmModuleId::MAPPING,
          hozon::perception::base::FaultType::HD_MAP_SDK_ACTIVE_ERROR,
          hozon::perception::base::FaultStatus::RESET,
          hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
      hd_map_active_error = false;
    }
    if (hd_map_init_fail) {
      phm_fault->Report(MAKE_FM_TUPLE(
          hozon::perception::base::FmModuleId::MAPPING,
          hozon::perception::base::FaultType::HD_MAP_SDK_INIT_FAIL,
          hozon::perception::base::FaultStatus::RESET,
          hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
      hd_map_init_fail = false;
    }
  }
  pre_fault_value_ = fault_value;
  if (fault_value == 0) {
    phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::HD_MAP_INPUT_LOC_ERROR,
        hozon::perception::base::FaultStatus::OCCUR,
        hozon::perception::base::SensorOrientation::UNKNOWN, 4, 1000));
    hd_map_input_loc_error = true;
    HLOG_ERROR << "HD_MAP_INPUT_LOC_ERROR";
  } else if (fault_value == 1) {
    phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::HD_MAP_EHP_INIT_ERROR,
        hozon::perception::base::FaultStatus::OCCUR,
        hozon::perception::base::SensorOrientation::UNKNOWN, 4, 1000));
    hd_map_ehp_init_error = true;
    HLOG_ERROR << "HD_MAP_EHP_INIT_ERROR";
  } else if (fault_value == 2) {
    phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::HD_MAP_MAP_DATA_ROM_ERROR,
        hozon::perception::base::FaultStatus::OCCUR,
        hozon::perception::base::SensorOrientation::UNKNOWN, 4, 1000));
    hd_map_data_rom_error = true;
    HLOG_ERROR << "HD_MAP_MAP_DATA_ROM_ERROR";
  } else if (fault_value == 3) {
    phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::HD_MAP_UUID_ERROR,
        hozon::perception::base::FaultStatus::OCCUR,
        hozon::perception::base::SensorOrientation::UNKNOWN, 4, 1000));
    hd_map_uuid_error = true;
    HLOG_ERROR << "HD_MAP_UUID_ERROR";
  } else if (fault_value == 4) {
    phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::HD_MAP_SDK_INNER_ERROR,
        hozon::perception::base::FaultStatus::OCCUR,
        hozon::perception::base::SensorOrientation::UNKNOWN, 4, 1000));
    hd_map_sdk_input_error = true;
    HLOG_ERROR << "HD_MAP_SDK_INNER_ERROR";
  } else if (fault_value == 5) {
    phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::HD_MAP_MAP_DATA_PATH_ERROR,
        hozon::perception::base::FaultStatus::OCCUR,
        hozon::perception::base::SensorOrientation::UNKNOWN, 4, 1000));
    hd_map_data_path_error = true;
    HLOG_ERROR << "HD_MAP_MAP_DATA_PATH_ERROR";
  } else if (fault_value == 6) {
    phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::HD_MAP_PATH_RW_ERROR,
        hozon::perception::base::FaultStatus::OCCUR,
        hozon::perception::base::SensorOrientation::UNKNOWN, 4, 1000));
    hd_map_path_rw_error = true;
    HLOG_ERROR << "HD_MAP_PATH_RW_ERROR ";
  } else if (fault_value == 7) {
    phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::HD_MAP_SDK_ACTIVE_ERROR,
        hozon::perception::base::FaultStatus::OCCUR,
        hozon::perception::base::SensorOrientation::UNKNOWN, 4, 1000));
    hd_map_active_error = true;
    HLOG_ERROR << "HD_MAP_SDK_ACTIVE_ERROR ";
  } else if (fault_value == 8) {
    phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::HD_MAP_SDK_INIT_FAIL,
        hozon::perception::base::FaultStatus::OCCUR,
        hozon::perception::base::SensorOrientation::UNKNOWN, 4, 1000));
    hd_map_init_fail = true;
    HLOG_ERROR << "HD_MAP_SDK_INIT_FAIL ";
  }
  return 0;
}

Eigen::Vector3d MapFusionLite::Qat2EulerAngle(const Eigen::Quaterniond& q) {
  Eigen::Vector3d eulerangle = {0, 0, 0};
  double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
  double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
  eulerangle[0] = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
  if (fabs(sinp) >= 1) {
    eulerangle[1] = copysign(M_PI / 2, sinp);
  } else {
    eulerangle[1] = asin(sinp);
  }
  // yaw (z-axis rotation)
  double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
  double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  eulerangle[2] = atan2(siny_cosp, cosy_cosp);
  return eulerangle;
}

}  // namespace common_onboard
}  // namespace perception
}  // namespace hozon
