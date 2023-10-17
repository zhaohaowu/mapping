/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: wangjianguo
 *Date: 2023-08-31
 *****************************************************************************/
#include "onboard/onboard_cyber/local_mapping/local_mapping_component.h"

DEFINE_string(location_topic, "/location", "location topic");
DEFINE_string(dr_topic, "/odom", "location topic");
DEFINE_string(ins_topic, "/PluginNodeInfo", "location topic");
DEFINE_string(laneline_topic, "/LaneLine", "location topic");
DEFINE_string(roadedge_topic, "/lidar_roadedge", "location topic");
DEFINE_string(output_topic, "/local_map", "location topic");
DEFINE_string(output_location_topic, "/local_map/location", "location topic");
DEFINE_string(config_yaml, "conf/mapping/local_mapping/local_mapping_conf.yaml",
              "path to local mapping conf yaml");
namespace hozon {
namespace mp {
namespace lm {

bool LMapComponent::Init() {
  lmap_ = std::make_shared<LMapApp>(FLAGS_config_yaml);

  location_listener_ = node_->CreateReader<hozon::localization::Localization>(
      FLAGS_location_topic,
      [this](
          const std::shared_ptr<const hozon::localization::Localization>& msg) {
        OnLocation(msg);
      });

  dr_listener_ = node_->CreateReader<hozon::dead_reckoning::DeadReckoning>(
      FLAGS_dr_topic,
      [this](const std::shared_ptr<const hozon::dead_reckoning::DeadReckoning>&
                 msg) { OnDr(msg); });

  ins_listener_ = node_->CreateReader<hozon::localization::HafNodeInfo>(
      FLAGS_ins_topic,
      [this](const std::shared_ptr<hozon::localization::HafNodeInfo>& msg) {
        OnIns(msg);
      });

  laneline_listener_ = node_->CreateReader<hozon::perception::TransportElement>(
      FLAGS_laneline_topic,
      [this](const std::shared_ptr<const hozon::perception::TransportElement>&
                 msg) { OnLaneLine(msg); });

  roadedge_listener_ = node_->CreateReader<hozon::perception::TransportElement>(
      FLAGS_roadedge_topic,
      [this](const std::shared_ptr<const hozon::perception::TransportElement>&
                 msg) { OnRoadEdge(msg); });

  result_talker_ =
      node_->CreateWriter<hozon::mapping::LocalMap>(FLAGS_output_topic);

  result_location_talker_ =
      node_->CreateWriter<hozon::localization::Localization>(
          FLAGS_output_location_topic);

  local_map_publish_thread_ =
      std::thread(&LMapComponent::LocalMapPublish, this);

  local_map_location_publish_thread_ =
      std::thread(&LMapComponent::LocalMapLocationPublish, this);

  return true;
}

bool LMapComponent::OnLocation(
    const std::shared_ptr<const hozon::localization::Localization>& msg) {
  if (!lmap_) {
    return false;
  }
  lmap_->OnLocation(msg);
  return true;
}

bool LMapComponent::OnDr(
    const std::shared_ptr<const hozon::dead_reckoning::DeadReckoning>& msg) {
  if (!lmap_ || !msg) {
    return false;
  }
  if (msg->pose().pose_local().quaternion().w() == 0 &&
      msg->pose().pose_local().quaternion().x() == 0 &&
      msg->pose().pose_local().quaternion().y() == 0 &&
      msg->pose().pose_local().quaternion().z() == 0) {
    return false;
  }
  lmap_->OnDr(msg);
  return true;
}

bool LMapComponent::OnIns(
    const std::shared_ptr<const hozon::localization::HafNodeInfo>& msg) {
  if (!lmap_) {
    return false;
  }
  lmap_->OnIns(msg);
  return true;
}

bool LMapComponent::OnLaneLine(
    const std::shared_ptr<const hozon::perception::TransportElement>& msg) {
  if (!lmap_) {
    return false;
  }
  lmap_->OnLaneLine(msg);
  return true;
}

bool LMapComponent::OnRoadEdge(
    const std::shared_ptr<const hozon::perception::TransportElement>& msg) {
  if (!lmap_) {
    return false;
  }
  lmap_->OnRoadEdge(msg);
  return true;
}

void LMapComponent::LocalMapPublish() {
  while (apollo::cyber::OK()) {
    std::shared_ptr<hozon::mapping::LocalMap> result =
        std::make_shared<hozon::mapping::LocalMap>();
    if (lmap_->FetchLocalMap(result) && result_talker_ != nullptr) {
      result_talker_->Write(result);
    }
    usleep(100 * 1e3);
  }
}

void LMapComponent::LocalMapLocationPublish() {
  while (apollo::cyber::OK()) {
    std::shared_ptr<hozon::localization::Localization> result =
        std::make_shared<hozon::localization::Localization>();
    if (lmap_->FetchLocalMapLocation(result) &&
        result_location_talker_ != nullptr) {
      result_location_talker_->Write(result);
    }
    usleep(10 * 1e3);
  }
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
