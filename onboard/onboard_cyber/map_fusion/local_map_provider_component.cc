/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： local_map_provider_component.cc
 *   author     ： xuliang
 *   date       ： 2023.09
 ******************************************************************************/

#include "onboard/onboard_cyber/map_fusion/local_map_provider_component.h"

#include <gflags/gflags.h>

#include "map_fusion/local_map_provider/local_map_provider.h"
#include "util/temp_log.h"

DEFINE_string(channel_local_map_provider, "lmp/local_map_provider",
              "channel of local map msg from local map provider");
DEFINE_string(channel_ins_node_info_lmp, "/PluginNodeInfo",
              "channel of ins msg from ins fusion");
DEFINE_string(channel_location, "/dr_location",
              "channel of location msg from location");
DEFINE_string(channel_lane_line, "/LaneLine",
              "channel of lane line msg from perception");
DEFINE_string(channel_road_edge, "/lidar_roadedge",
              "channel of road edge msg from perception");

namespace hozon {
namespace mp {
namespace mf {

bool LocalMapProviderComponent::Init() {
  lm_provider_ = std::make_shared<LocalMapProvider>();
  lm_provider_->Init();

  lm_writer_ = node_->CreateWriter<hozon::mapping::LocalMap>(
      FLAGS_channel_local_map_provider);

  ins_reader_ = node_->CreateReader<hozon::localization::HafNodeInfo>(
      FLAGS_channel_ins_node_info_lmp,
      [this](const std::shared_ptr<hozon::localization::HafNodeInfo>& msg) {
        OnInsNodeInfo(msg);
      });
  location_reader_ = node_->CreateReader<hozon::localization::Localization>(
      FLAGS_channel_location,
      [this](const std::shared_ptr<hozon::localization::Localization>& msg) {
        OnLocation(msg);
      });
  laneline_reader_ = node_->CreateReader<hozon::perception::TransportElement>(
      FLAGS_channel_lane_line,
      [this](const std::shared_ptr<hozon::perception::TransportElement>& msg) {
        OnLaneLine(msg);
      });
  roadedge_reader_ = node_->CreateReader<hozon::perception::TransportElement>(
      FLAGS_channel_road_edge,
      [this](const std::shared_ptr<hozon::perception::TransportElement>& msg) {
        OnRoadEdge(msg);
      });

  return true;
}

void LocalMapProviderComponent::OnInsNodeInfo(
    const std::shared_ptr<hozon::localization::HafNodeInfo>& msg) {
  if (!msg) {
    HLOG_ERROR << "message ins node info is null";
    return;
  }
  if (!lm_provider_ || !lm_writer_) {
    HLOG_ERROR << "nullptr local map provider or local map writer";
    return;
  }
  lm_provider_->OnInsNodeInfo(msg);
}

void LocalMapProviderComponent::OnLocation(
    const std::shared_ptr<hozon::localization::Localization>& msg) {
  if (!msg) {
    HLOG_ERROR << "message location is null";
    return;
  }
  if (!lm_provider_ || !lm_writer_) {
    HLOG_ERROR << "nullptr local map provider or local map writer";
    return;
  }
  lm_provider_->OnLocation(msg);

  // 地图分发
  auto map = lm_provider_->GetLocalMap();
  lm_writer_->Write(map);
}

void LocalMapProviderComponent::OnLaneLine(
    const std::shared_ptr<hozon::perception::TransportElement>& msg) {
  if (!msg) {
    HLOG_ERROR << "message lane is null";
    return;
  }
  if (!lm_provider_) {
    HLOG_ERROR << "nullptr local map provider";
    return;
  }
  lm_provider_->OnLaneLine(msg);

  // 地图分发
  auto map = lm_provider_->GetLocalMap();
  lm_writer_->Write(map);
}

void LocalMapProviderComponent::OnRoadEdge(
    const std::shared_ptr<hozon::perception::TransportElement>& msg) {
  if (!msg) {
    HLOG_ERROR << "message road edge is null";
    return;
  }
  if (!lm_provider_) {
    HLOG_ERROR << "nullptr local map provider";
    return;
  }
  lm_provider_->OnRoadEdge(msg);
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
