/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: shenlianchen
 *Date: 2023-08-31
 *****************************************************************************/

#include "onboard/onboard_adc/include/mapping_onboard.h"

#include "ap-release/include/adsfi/include/data_types/debug/pbdebug.h"
#include "util/tic_toc.h"

// NOLINTBEGIN
DEFINE_string(lm_config, "conf/mapping/local_mapping/local_mapping_conf.yaml",
              "path to local mapping conf yaml");
DEFINE_bool(adc_viz, false, "if use rviz");
DEFINE_string(adc_viz_addr, "tcp://10.6.73.235:9100",
              "RvizAgent's working address");
DEFINE_int32(save_map_every_cnt, 0,
             "save sent map to file every specified count");
// NOLINTEND

namespace hozon {
namespace mp {
int32_t MappingAdc::MappingAdc::AlgInit() {
  if (FLAGS_adc_viz) {
    HLOG_INFO << "Start RvizAgent on " << FLAGS_adc_viz_addr;
    int ret = util::RvizAgent::Instance().Init(FLAGS_adc_viz_addr);
    if (ret < 0) {
      HLOG_ERROR << "RvizAgent start failed";
    }
  }
  lmap_ = std::make_unique<lm::LMapApp>(FLAGS_lm_config);
  dr_ = std::make_unique<dr::DRInterface>();
  loc_ = std::make_unique<loc::Localization>();
  if (!loc_->Init()) {
    return -1;
  }

  mf_ = std::make_unique<mf::MapFusion>();
  int ret = mf_->Init("");
  if (ret < 0) {
    HLOG_ERROR << "Init map fusion failed";
    return -1;
  }

  dr_data_ptr_ = std::make_shared<hozon::dead_reckoning::DeadReckoning>();

  return 0;
}

int32_t MappingAdc::ChassisImuCallBack(hz_Adsfi::NodeBundle* input) {
  HLOG_ERROR << "----imu callback";
  std::shared_ptr<hz_Adsfi::AlgImuIns> imuinsDataPtr_ =
      std::static_pointer_cast<hz_Adsfi::AlgImuIns>(input->GetOne("imu"));
  if (imuinsDataPtr_ == nullptr) {
    return -1;
  }

  board_.imu_proto.reset();
  board_.imu_proto = std::make_shared<hozon::soc::ImuIns>();
  hozon::mp::DataBoard::Adsfi2Proto(imuinsDataPtr_, board_.imu_proto);
  dr_->AddImuData(board_.imu_proto);

  loc_->OnImu(*(board_.imu_proto));
  loc_->OnOriginIns(*(board_.imu_proto));

  // localization output by imu frequency
  auto loc_res = std::make_shared<hozon::localization::Localization>();
  if (loc_->GetCurrentLocalization(loc_res.get())) {
    // get localization here
    std::string seri_loc;
    if (loc_res->SerializeToString(&seri_loc)) {
      auto debug_loc = std::make_shared<hz_Adsfi::AlgPbDebugFrame>();
      debug_loc->algDebugframe.header.seq = loc_res->header().seq();
      debug_loc->algDebugframe.header.frameId = loc_res->header().frame_id();
      auto sec = static_cast<uint32_t>(loc_res->header().publish_stamp());
      auto nsec = static_cast<uint32_t>(
          (loc_res->header().publish_stamp() - sec) * 1e9);
      debug_loc->algDebugframe.header.gnssStamp.sec = sec;
      debug_loc->algDebugframe.header.gnssStamp.nsec = nsec;
      std::stringstream ss_stream;
      ss_stream << "wgs:" << Node2Xyz(loc_res->pose().wgs()) << "\n"
                << "gcj02:" << Node2Xyz(loc_res->pose().gcj02()) << "\n"
                << "quaternion(x,y,z,w):"
                << Node2Xyzw(loc_res->pose().quaternion()) << "\n"
                << "euler_angle:" << Node2Xyz(loc_res->pose().euler_angle())
                << "\n"
                << "using_utm_zone:" << loc_res->pose().using_utm_zone() << "\n"
                << "rtk_status:" << loc_res->rtk_status() << "\n"
                << "location_state:" << loc_res->location_state() << "\n"
                << "laneid:" << loc_res->laneid() << "\n"
                << "local_pose:" << Node2Xyz(loc_res->pose().local_pose())
                << "\n"
                << "euler_angles_local:"
                << Node2Xyz(loc_res->pose().euler_angles_local()) << "\n"
                << "local_heading:" << loc_res->pose().local_heading();
      debug_loc->algDebugframe.msg_2 = ss_stream.str();

      debug_loc->algDebugframe.msg_1 = seri_loc;
      hz_Adsfi::NodeBundle output;
      output.Add("npp_debug_msg_31", debug_loc);
      SendOutput(&output);

      {
        std::lock_guard<std::mutex> lock(loc_mtx_);
        curr_loc_ =
            std::make_shared<hozon::localization::Localization>(*loc_res);
      }
    }
  }

  std::shared_ptr<hz_Adsfi::AlgChassisInfo> chassisDataPtr_ =
      std::static_pointer_cast<hz_Adsfi::AlgChassisInfo>(
          input->GetOne("chassis"));
  if (chassisDataPtr_ == nullptr) {
    return -1;
  }

  board_.chassis_proto.reset();
  board_.chassis_proto = std::make_shared<hozon::soc::Chassis>();
  hozon::mp::DataBoard::Adsfi2Proto(chassisDataPtr_, board_.chassis_proto);
  dr_->AddChassisData(board_.chassis_proto);

  if (dr_->SetLocation(dr_data_ptr_)) {
    lmap_->OnDr(dr_data_ptr_);
    loc_->OnDr(*dr_data_ptr_);
  }
  return 0;
}

int32_t MappingAdc::LaneCallBack(hz_Adsfi::NodeBundle* input) {
  // road_marking
  const auto p_road_marking =
      std::static_pointer_cast<hz_Adsfi::AlgLaneDetectionOutArray>(
          input->GetOne("nnp_cam_lane"));
  if (p_road_marking) {
    // HLOG_ERROR << "-----lane callback";
    board_.adsfi_lane_proto.reset();
    board_.adsfi_lane_proto =
        std::make_shared<hozon::perception::TransportElement>();
    DataBoard::Adsfi2Proto(*p_road_marking, board_.adsfi_lane_proto.get());
    lmap_->OnLaneLine(board_.adsfi_lane_proto);
    std::shared_ptr<hozon::mapping::LocalMap> result =
        std::make_shared<hozon::mapping::LocalMap>();
    if (lmap_->FetchLocalMap(result)) {
      loc_->OnLocalMap(*result);
      {
        std::lock_guard<std::mutex> lock(local_map_mtx_);
        curr_local_map_ = std::make_shared<hozon::mapping::LocalMap>(*result);
      }
    }
  } else {
    std::cout << "null lane data" << std::endl;
  }

  return 0;
}

int32_t MappingAdc::PluginCallback(hz_Adsfi::NodeBundle* input) {
  if (input == nullptr) {
    return -1;
  }

  const auto p_plugin = std::static_pointer_cast<hz_Adsfi::AlgLocationNodeInfo>(
      input->GetOne("plugin"));
  if (p_plugin) {
    DataBoard::Adsfi2Proto(*p_plugin, board_.plugin_proto.get());
    loc_->OnInspva(*(board_.plugin_proto));
    {
      std::lock_guard<std::mutex> lock(plugin_mtx_);
      curr_plugin_ = std::make_shared<hozon::localization::HafNodeInfo>(
          *(board_.plugin_proto));
    }
  }

  return 0;
}

int32_t MappingAdc::MapServiceCycleCallback(hz_Adsfi::NodeBundle* input) {
  (void)(input);
  if (!mf_) {
    HLOG_ERROR << "nullptr map fusion";
    return -1;
  }

  std::shared_ptr<hozon::localization::HafNodeInfo> latest_plugin = nullptr;
  {
    std::lock_guard<std::mutex> lock(plugin_mtx_);
    if (curr_plugin_) {
      latest_plugin =
          std::make_shared<hozon::localization::HafNodeInfo>(*curr_plugin_);
    } else {
      HLOG_ERROR << "nullptr current plugin node info";
      return -1;
    }
  }
  std::shared_ptr<hozon::planning::ADCTrajectory> latest_planning = nullptr;
  hozon::routing::RoutingResponse routing;
  mf_->ProcService(latest_plugin, latest_planning, &routing);
  {
    std::lock_guard<std::mutex> lock(routing_mtx_);
    curr_routing_ = std::make_shared<hozon::routing::RoutingResponse>(routing);
  }
  return 0;
}

std::shared_ptr<hozon::localization::Localization> MappingAdc::GetLatestLoc() {
  std::shared_ptr<hozon::localization::Localization> latest_loc = nullptr;
  std::lock_guard<std::mutex> lock(loc_mtx_);
  if (curr_loc_ != nullptr) {
    latest_loc =
        std::make_shared<hozon::localization::Localization>(*curr_loc_);
  }
  return latest_loc;
}

std::shared_ptr<hozon::mapping::LocalMap> MappingAdc::GetLatestLocalMap() {
  std::shared_ptr<hozon::mapping::LocalMap> latest_local_map = nullptr;
  std::lock_guard<std::mutex> lock(local_map_mtx_);
  if (curr_local_map_ != nullptr) {
    latest_local_map =
        std::make_shared<hozon::mapping::LocalMap>(*curr_local_map_);
  }
  return latest_local_map;
}

std::shared_ptr<hozon::routing::RoutingResponse>
MappingAdc::GetLatestRouting() {
  std::shared_ptr<hozon::routing::RoutingResponse> latest_routing = nullptr;
  std::lock_guard<std::mutex> lock(routing_mtx_);
  if (curr_routing_ != nullptr) {
    latest_routing =
        std::make_shared<hozon::routing::RoutingResponse>(*curr_routing_);
  }
  return latest_routing;
}

int32_t MappingAdc::MapFusionCycleCallback(hz_Adsfi::NodeBundle* input) {
  (void)(input);
  if (!mf_) {
    HLOG_ERROR << "nullptr map fusion";
    return -1;
  }

  util::TicToc tic;
  std::shared_ptr<hozon::localization::Localization> latest_loc =
      GetLatestLoc();
  if (latest_loc == nullptr) {
    HLOG_ERROR << "nullptr latest loc";
    return -1;
  }
  HLOG_INFO << "get latest loc cost " << tic.Toc();
  tic.Tic();

  std::shared_ptr<hozon::mapping::LocalMap> latest_local_map =
      GetLatestLocalMap();
  if (latest_local_map == nullptr) {
    HLOG_ERROR << "nullptr latest local map";
    return -1;
  }
  HLOG_INFO << "get latest local map cost " << tic.Toc();
  tic.Tic();

  std::shared_ptr<hozon::routing::RoutingResponse> latest_routing =
      GetLatestRouting();
  if (latest_routing == nullptr) {
    HLOG_ERROR << "nullptr latest routing";
    return -1;
  }
  HLOG_INFO << "get latest routing cost " << tic.Toc();
  tic.Tic();

  auto latest_map = std::make_shared<hozon::hdmap::Map>();
  int ret = mf_->ProcFusion(latest_loc, latest_local_map, latest_map.get());
  if (ret < 0) {
    HLOG_ERROR << "map fusion ProcFusion failed";
    return -1;
  }
  HLOG_INFO << "ProcFusion cost " << tic.Toc();
  tic.Tic();

  ret = SendFusionResult(latest_loc, latest_map, latest_routing);
  HLOG_INFO << "SendFusionResult cost " << tic.Toc();
  if (ret < 0) {
    HLOG_ERROR << "SendFusionResult failed";
    return -1;
  }

  return 0;
}

int MappingAdc::SendFusionResult(
    const std::shared_ptr<hozon::localization::Localization>& loc,
    const std::shared_ptr<hozon::hdmap::Map>& map,
    const std::shared_ptr<hozon::routing::RoutingResponse>& routing) {
  if (loc == nullptr || map == nullptr || routing == nullptr) {
    HLOG_ERROR << "input nullptr loc or map or routing";
    return -1;
  }
  util::TicToc tic;
  std::string ser_map = map->SerializeAsString();
  std::string ser_routing = routing->SerializeAsString();
  if (ser_map.empty() || ser_routing.empty()) {
    HLOG_ERROR << "empty serialized map or routing, map size " << ser_map.size()
               << ", routing size " << ser_routing.size();
    return -1;
  }
  HLOG_INFO << "serialize map and routing cost " << tic.Toc();
  tic.Tic();

  auto debug_map = std::make_shared<hz_Adsfi::AlgPbDebugFrame>();
  debug_map->algDebugframe.header.seq = map->header().header().seq();
  debug_map->algDebugframe.header.frameId = map->header().header().frame_id();
  const auto sec =
      static_cast<uint32_t>(map->header().header().publish_stamp());
  const auto nsec = static_cast<uint32_t>(
      (map->header().header().publish_stamp() - sec) * 1e9);
  debug_map->algDebugframe.header.stamp.sec = sec;
  debug_map->algDebugframe.header.stamp.nsec = nsec;
  debug_map->algDebugframe.msg_1 = ser_map;
  debug_map->algDebugframe.msg_2 = ser_routing;
  hz_Adsfi::NodeBundle output;
  output.Add("npp_debug_msg_32", debug_map);
  SendOutput(&output);
  HLOG_INFO << "assemble msg_32 cost " << tic.Toc();
  tic.Tic();

  if (FLAGS_save_map_every_cnt > 0) {
    static int cnt = 0;
    if (cnt % FLAGS_save_map_every_cnt == 0) {
      std::ofstream file;
      std::string file_name = "fusion_map_" + std::to_string(cnt) + ".txt";
      file.open(file_name);
      file << "------loc.header------\n";
      file << loc->header().DebugString();
      file << "------loc.pose------\n";
      file << loc->pose().DebugString();
      file << "------map------\n";
      file << map->DebugString();
      file << "------routing------\n";
      file << routing->DebugString();
      file.close();
    }
    cnt++;
    HLOG_INFO << "save map to file cost " << tic.Toc();
    tic.Tic();
  }

  return 0;
}

void MappingAdc::AlgRelease() {
  if (mf_) {
    HLOG_INFO << "try stopping map fusion";
    mf_->Stop();
    HLOG_INFO << "done stopping map fusion";
  }

  if (RVIZ_AGENT.Ok()) {
    RVIZ_AGENT.Term();
  }
}

template <typename T>
std::string MappingAdc::Node2Xyz(const T& p) {
  std::stringstream stream;
  stream << std::fixed << std::setprecision(10) << p.x() << "," << p.y() << ","
         << p.z() << ",";
  return stream.str();
}

template <typename T>
std::string MappingAdc::Node2Xyzw(const T& p) {
  std::stringstream stream;
  stream << std::fixed << std::setprecision(10) << p.x() << "," << p.y() << ","
         << p.z() << "," << p.w() << ",";
  return stream.str();
}

}  // namespace mp
}  // namespace hozon
