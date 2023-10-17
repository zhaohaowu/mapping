/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： map_fusion_component.cc
 *   author     ： taoshaoyuan
 *   date       ： 2023.08
 ******************************************************************************/

#include "onboard/onboard_cyber/map_fusion/map_fusion_component.h"

#include <gflags/gflags.h>

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
DEFINE_string(channel_node_info, "/PluginNodeInfo", "channel of ins node info");
DEFINE_string(channel_prior_map, "mf/prior_map",
              "channel of prior map, maybe HQ or HD");
DEFINE_string(channel_lm_local_map, "/local_map",
              "channel of local map from local mapping");
DEFINE_string(channel_local_location, "/local_map/location",
              "channel of location in local map from localization");

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

  ins_node_info_reader_ = node_->CreateReader<hozon::localization::HafNodeInfo>(
      FLAGS_channel_node_info,
      [this](const std::shared_ptr<hozon::localization::HafNodeInfo>& msg) {
        OnInsNodeInfo(msg);
      });

  hq_map_reader_ = node_->CreateReader<hozon::hdmap::Map>(
      FLAGS_channel_prior_map,
      [this](const std::shared_ptr<hozon::hdmap::Map>& msg) { OnHQMap(msg); });

  local_map_reader_ = node_->CreateReader<hozon::mapping::LocalMap>(
      FLAGS_channel_lm_local_map,
      [this](const std::shared_ptr<hozon::mapping::LocalMap>& msg) {
        OnLocalMap(msg);
      });

  local_map_location_reader_ =
      node_->CreateReader<hozon::localization::Localization>(
          FLAGS_channel_local_location,
          [this](
              const std::shared_ptr<hozon::localization::Localization>& msg) {
            OnLocalMapLocation(msg);
          });

  return true;
}

void MapFusionComponent::Clear() {
  if (!FLAGS_mf_viz.empty() && RVIZ_AGENT.Ok()) {
    RVIZ_AGENT.Term();
  }
}

void MapFusionComponent::OnInsNodeInfo(
    const std::shared_ptr<hozon::localization::HafNodeInfo>& msg) {
  mf_->OnInsNodeInfo(msg);
}

void MapFusionComponent::OnHQMap(
    const std::shared_ptr<hozon::hdmap::Map>& msg) {
  mf_->OnHQMap(msg);
}

void MapFusionComponent::OnLocalMap(
    const std::shared_ptr<hozon::mapping::LocalMap>& msg) {
  mf_->OnLocalMap(msg);

  auto fusion_map = mf_->GetMap();
  if (!fusion_map) {
    map_writer_->Write(fusion_map);
  }
}

void MapFusionComponent::OnLocalMapLocation(
    const std::shared_ptr<hozon::localization::Localization>& msg) {
  mf_->OnLocalMapLocation(msg);
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
