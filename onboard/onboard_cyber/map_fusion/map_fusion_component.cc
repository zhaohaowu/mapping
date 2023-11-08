/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： map_fusion_component.cc
 *   author     ： taoshaoyuan
 *   date       ： 2023.08
 ******************************************************************************/

#include "onboard/onboard_cyber/map_fusion/map_fusion_component.h"
#include <cyber/time/rate.h>
#include <depend/common/adapters/adapter_gflags.h>
#include <depend/common/configs/config_gflags.h>
#include <depend/common/utm_projection/coordinate_convertor.h>
#include <gflags/gflags.h>

#include <Eigen/Geometry>
#include <memory>

#include "map_fusion/map_fusion.h"
#include "util/rviz_agent/rviz_agent.h"

DEFINE_string(mf_config, "conf/mapping/map_fusion/map_fusion.yaml",
              "path to map fusion's config yaml");
DEFINE_string(mf_viz, "ipc:///tmp/rviz_agent_mf",
              "RvizAgent's working address, this should corresponds to the "
              "address used in RvizBridge. Leaving empty represents not using "
              "RvizAgent for visualization");
DEFINE_string(channel_fusion_map, "/fusion_map",
              "channel of map fusion result");
DEFINE_string(channel_routing_response, "/hozon/routing_response",
              "channel of map fusion result");
DEFINE_string(channel_localization, "/mapping/location/fc",
              "channel of local and global location from localization");
DEFINE_string(channel_plugin_node_info, "/PluginNodeInfo",
              "channel of ins node info");
DEFINE_string(channel_lm_local_map, "/local_map",
              "channel of local map from local mapping");
DEFINE_bool(
    use_localization, true,
    "whether use localization result directly from Localization module");

namespace hozon {
namespace mp {
namespace mf {

bool MapFusionComponent::Init() {
  if (!FLAGS_mf_viz.empty() && !RVIZ_AGENT.Ok()) {
    int ret = RVIZ_AGENT.Init(FLAGS_mf_viz);
    if (ret < 0) {
      HLOG_WARN << "RvizAgent init failed";
    }
  }

  mf_ = std::make_shared<MapFusion>();
  int ret = mf_->Init(FLAGS_mf_config);
  if (ret < 0) {
    HLOG_ERROR << "Init MapFusion failed";
    return false;
  }

  map_writer_ =
      node_->CreateWriter<hozon::hdmap::Map>(FLAGS_channel_fusion_map);
  routing_writer_ = node_->CreateWriter<hozon::routing::RoutingResponse>(
      FLAGS_channel_routing_response);

  localization_reader_ = node_->CreateReader<hozon::localization::Localization>(
      FLAGS_channel_localization);

  plugin_node_info_reader_ =
      node_->CreateReader<hozon::localization::HafNodeInfo>(
          FLAGS_channel_plugin_node_info);

  local_map_reader_ =
      node_->CreateReader<hozon::mapping::LocalMap>(FLAGS_channel_lm_local_map);

  planning_reader_ = node_->CreateReader<hozon::planning::ADCTrajectory>(
      FLAGS_planning_trajectory_topic);

  running_proc_service_.store(true);
  proc_service_ =
      std::make_shared<std::thread>(&MapFusionComponent::ProcForService, this);

  return true;
}

void MapFusionComponent::Clear() {
  HLOG_INFO << "try stopping proc_service_ thread";
  if (proc_service_ && proc_service_->joinable()) {
    running_proc_service_.store(false);
    proc_service_->join();
  }
  HLOG_INFO << "done stopping proc_service_ thread";

  HLOG_INFO << "try stopping map fusion";
  if (mf_) {
    mf_->Stop();
  }
  HLOG_INFO << "done stopping map fusion";

  if (!FLAGS_mf_viz.empty() && RVIZ_AGENT.Ok()) {
    RVIZ_AGENT.Term();
  }
}

bool MapFusionComponent::Proc() {
  std::shared_ptr<hozon::localization::Localization> latest_loc = nullptr;
  if (FLAGS_use_localization) {
    localization_reader_->Observe();
    latest_loc = localization_reader_->GetLatestObserved();
    if (!latest_loc) {
      HLOG_ERROR << "localization msg not ready";
      return false;
    }
  } else {
    plugin_node_info_reader_->Observe();
    auto latest_plugin = plugin_node_info_reader_->GetLatestObserved();
    if (!latest_plugin) {
      HLOG_ERROR << "plugin node info msg not ready";
      return false;
    }

    latest_loc = std::make_shared<hozon::localization::Localization>();
    latest_loc->Clear();
    latest_loc->mutable_header()->CopyFrom(latest_plugin->header());
    latest_loc->mutable_header()->set_publish_stamp(
        latest_loc->header().gnss_stamp());

    // 从ins里获取全局定位
    Eigen::Quaterniond quat;
    quat.w() = latest_plugin->quaternion().w();
    quat.x() = latest_plugin->quaternion().x();
    quat.y() = latest_plugin->quaternion().y();
    quat.z() = latest_plugin->quaternion().z();
    Eigen::Matrix3d rot = quat.toRotationMatrix();
    Eigen::Vector3d euler = rot.eulerAngles(2, 0, 1);
    latest_loc->mutable_pose()->mutable_euler_angles()->set_x(euler[1]);
    latest_loc->mutable_pose()->mutable_euler_angles()->set_y(euler[2]);
    latest_loc->mutable_pose()->mutable_euler_angles()->set_z(euler[0]);
    uint32_t zone = std::floor(latest_plugin->pos_gcj02().y() / 6.0 + 31);
    double x = latest_plugin->pos_gcj02().x();
    double y = latest_plugin->pos_gcj02().y();
    hozon::common::coordinate_convertor::GCS2UTM(zone, &y, &x);
    latest_loc->mutable_pose()->mutable_pos_utm_01()->set_x(y);
    latest_loc->mutable_pose()->mutable_pos_utm_01()->set_y(x);
    latest_loc->mutable_pose()->mutable_pos_utm_01()->set_z(0);
    latest_loc->mutable_pose()->set_utm_zone_01(zone);
  }

  local_map_reader_->Observe();
  auto latest_local_map = local_map_reader_->GetLatestObserved();
  if (!latest_local_map) {
    HLOG_ERROR << "local map msg not ready";
    return false;
  }
  hozon::hdmap::Map fusion_map;
  int ret = mf_->ProcFusion(latest_loc, latest_local_map, &fusion_map);
  if (ret < 0) {
    HLOG_ERROR << "ProcFusion failed";
    return false;
  }

  map_writer_->Write(fusion_map);
  return true;
}

void MapFusionComponent::ProcForService() {
  apollo::cyber::Rate rate(10.);
  while (running_proc_service_.load()) {
    plugin_node_info_reader_->Observe();
    auto latest_plugin = plugin_node_info_reader_->GetLatestObserved();
    if (!latest_plugin) {
      HLOG_ERROR << "plugin node info msg not ready";
      rate.Sleep();
      continue;
    }

    std::shared_ptr<hozon::planning::ADCTrajectory> latest_planning = nullptr;
    if (FLAGS_ehp_monitor == 2 || FLAGS_ehp_monitor == 0) {
      planning_reader_->Observe();
      latest_planning = planning_reader_->GetLatestObserved();  // NOLINT
      if (!latest_planning) {
        HLOG_ERROR << "planning msg is not ready!";
        rate.Sleep();
        continue;
      }
    }

    hozon::routing::RoutingResponse routing;
    int ret = mf_->ProcService(latest_plugin, latest_planning, &routing);
    if (ret < 0) {
      HLOG_ERROR << "ProcService failed";
      rate.Sleep();
      continue;
    }
    routing_writer_->Write(routing);
    rate.Sleep();
  }
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
