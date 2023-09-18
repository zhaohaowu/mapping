/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: wangjianguo
 *Date: 2023-08-31
 *****************************************************************************/
#include "modules/local_mapping/cyber/local_mapping_component.h"

DEFINE_string(location_topic, "/location", "location topic");
DEFINE_string(dr_topic, "/dr_location", "location topic");
DEFINE_string(laneline_topic, "/cam_post_laneline", "location topic");
DEFINE_string(roadedge_topic, "/lidar_roadedge", "location topic");
DEFINE_string(output_topic, "/local_map", "location topic");
DEFINE_string(config_yaml, "conf/mapping/local_mapping/local_mapping_conf.yaml",
              "path to local mapping conf yaml");
namespace hozon {
namespace mp {
namespace lm {

bool LMapComponent::Init() {
  lmap_ = std::make_shared<LMapApp>(FLAGS_config_yaml);

  local_map_publish_thread_ =
      std::thread(&LMapComponent::LocalMapPublish, this);

  location_listener_ = node_->CreateReader<adsfi_proto::hz_Adsfi::AlgLocation>(
      FLAGS_location_topic,
      [this](const std::shared_ptr<const adsfi_proto::hz_Adsfi::AlgLocation>&
                 msg) { OnLocation(msg); });

  dr_listener_ = node_->CreateReader<adsfi_proto::hz_Adsfi::AlgLocation>(
      FLAGS_dr_topic,
      [this](const std::shared_ptr<const adsfi_proto::hz_Adsfi::AlgLocation>&
                 msg) { OnDr(msg); });

  laneline_listener_ =
      node_->CreateReader<adsfi_proto::hz_Adsfi::AlgLaneDetectionOutArray>(
          FLAGS_laneline_topic,
          [this](const std::shared_ptr<
                 const adsfi_proto::hz_Adsfi::AlgLaneDetectionOutArray>& msg) {
            OnLaneLine(msg);
          });

  roadedge_listener_ =
      node_->CreateReader<adsfi_proto::hz_Adsfi::AlgLaneDetectionOutArray>(
          FLAGS_roadedge_topic,
          [this](const std::shared_ptr<
                 const adsfi_proto::hz_Adsfi::AlgLaneDetectionOutArray>& msg) {
            OnRoadEdge(msg);
          });

  result_talker_ =
      node_->CreateWriter<hozon::mapping::LocalMap>(FLAGS_output_topic);

  return true;
}

bool LMapComponent::OnLocation(
    const std::shared_ptr<const adsfi_proto::hz_Adsfi::AlgLocation>& msg) {
  if (!lmap_) {
    return false;
  }
  lmap_->OnLocation(msg);
  return true;
}

bool LMapComponent::OnDr(
    const std::shared_ptr<const adsfi_proto::hz_Adsfi::AlgLocation>& msg) {
  if (!lmap_) {
    return false;
  }
  lmap_->OnDr(msg);
  return true;
}

bool LMapComponent::OnLaneLine(
    const std::shared_ptr<
        const adsfi_proto::hz_Adsfi::AlgLaneDetectionOutArray>& msg) {
  if (!lmap_) {
    return false;
  }
  lmap_->OnLaneLine(msg);
  return true;
}

bool LMapComponent::OnRoadEdge(
    const std::shared_ptr<
        const adsfi_proto::hz_Adsfi::AlgLaneDetectionOutArray>& msg) {
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
      // std::cout << "LMapComponent fetch" << std::endl;
    } else {
      usleep(100 * 1e3);
      continue;
    }
    usleep(10 * 1e3);
  }
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
