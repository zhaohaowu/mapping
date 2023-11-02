/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： pose_estimation_component.cc
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/

#include "onboard/onboard_cyber/location/pose_estimation/pose_estimation_component.h"

#include <gflags/gflags.h>
#include <yaml-cpp/yaml.h>

#include "modules/util/include/util/rviz_agent/rviz_agent.h"
DEFINE_string(
    config_yaml,
    "conf/mapping/location/pose_estimation/pose_estimation_config.yaml",
    "path to pose estimation config yaml");
DEFINE_string(config_cam_yaml, "conf/mapping/location/pose_estimation_cam.yaml",
              "path to map matching camera config yaml");
DEFINE_string(pose_viz_addr, "ipc:///tmp/rviz_agent_pose_estimation",
              "RvizAgent's working address, this should corresponds to the "
              "address used in RvizBridge. Leaving empty represents not using "
              "RvizAgent for visualization");

namespace hozon {
namespace mp {
namespace loc {

PoseEstimationComponent::~PoseEstimationComponent() {
  if (!FLAGS_pose_viz_addr.empty()) {
    util::RvizAgent::Instance().Term();
  }
}

bool PoseEstimationComponent::Init() {
  if (!FLAGS_pose_viz_addr.empty()) {
    int ret = hozon::mp::util::RvizAgent::Instance().Init(FLAGS_pose_viz_addr);
    if (ret < 0) {
      HLOG_WARN << "RvizAgent init failed";
    }
  }

  mm_ = std::make_shared<MapMatching>();
  mm_->Init(FLAGS_config_yaml, FLAGS_config_cam_yaml);

  YAML::Node node;
  try {
    node = YAML::LoadFile(FLAGS_config_yaml);
  } catch (const YAML::Exception &e) {
    HLOG_ERROR << "load config yaml " << FLAGS_config_yaml
               << " failed, throw: " << e.what();
    return false;
  }

  std::string key = "ins_topic";
  if (!node[key]) {
    HLOG_ERROR << key << " not found in config yaml";
    return false;
  }
  ins_topic_ = node[key].as<std::string>();

  key = "lane_line_topic";
  if (!node[key]) {
    HLOG_ERROR << key << " not found in config yaml";
    return false;
  }
  lane_line_topic_ = node[key].as<std::string>();

  key = "node_info_topic";
  if (!node[key]) {
    HLOG_ERROR << key << " not found in config yaml";
    return false;
  }
  node_info_topic_ = node[key].as<std::string>();

  // key = "location_topic";
  // if (!node[key]) {
  //   HLOG_ERROR << key << " not found in config yaml";
  //   return false;
  // }
  // location_topic_ = node[key].as<std::string>();

  ins_reader_ = node_->CreateReader<::hozon::localization::HafNodeInfo>(
      ins_topic_,
      [this](const std::shared_ptr<::hozon::localization::HafNodeInfo> &msg) {
        OnIns(msg);
      });

  road_marking_reader_ =
      node_->CreateReader<::hozon::perception::TransportElement>(
          lane_line_topic_,
          [this](const std::shared_ptr<::hozon::perception::TransportElement>
                     &msg) {
            OnPerception(msg);
            // OnMarkPole(msg);
          });

  //   location_reader_ = node_->CreateReader<location::HafLocation>(
  //       location_topic_,
  //       [this](const std::shared_ptr<location::HafLocation> &msg) {
  //         OnLocation(msg);
  //       });

  mm_pub_thread_ = std::thread(&PoseEstimationComponent::pubMmMsg, this);

  node_info_writer_ =
      node_->CreateWriter<hozon::localization::HafNodeInfo>(node_info_topic_);
  return true;
}

void PoseEstimationComponent::pubMmMsg() {
  while (apollo::cyber::OK()) {
    std::shared_ptr<hozon::localization::HafNodeInfo> node_info =
        mm_->getMmNodeInfo();
    auto pub_position_x = (*node_info).pos_gcj02().x();
    auto pub_position_y = (*node_info).pos_gcj02().y();
    auto pub_position_z = (*node_info).pos_gcj02().z();

    auto pub_quaternion_w = (*node_info).quaternion().w();
    auto pub_quaternion_x = (*node_info).quaternion().x();
    auto pub_quaternion_y = (*node_info).quaternion().y();
    auto pub_quaternion_z = (*node_info).quaternion().z();

    bool is_updated = ((pub_position_x != last_pub_position_x_) ||
                       (pub_position_y != last_pub_position_y_) ||
                       (pub_position_z != last_pub_position_z_));

    if (node_info != nullptr && node_info_writer_ != nullptr && is_updated) {
      std::cout << "hit writer" << std::endl;
      node_info_writer_->Write(node_info);
    }
    last_pub_position_x_ = pub_position_x;
    last_pub_position_y_ = pub_position_y;
    last_pub_position_z_ = pub_position_z;
    usleep(10 * 1e3);
  }
}

// void PoseEstimationComponent::OnLocation(
//     const std::shared_ptr<location::HafLocation> &msg) {
//   if (!mm_) {
//     return;
//   }
//   mm_->OnLocation(msg);
// }

void PoseEstimationComponent::OnIns(
    const std::shared_ptr<const ::hozon::localization::HafNodeInfo> &msg) {
  if (!mm_) {
    return;
  }
  mm_->OnIns(msg);
}

void PoseEstimationComponent::OnPerception(
    const std::shared_ptr<const ::hozon::perception::TransportElement> &msg) {
  if (!mm_) {
    return;
  }
  mm_->OnPerception(msg);
}

// void PoseEstimationComponent::OnMarkPole(
//     const std::shared_ptr<::adsfi_proto::hz_Adsfi::AlgLaneDetectionOut> &msg)
//     {
//   if (!mm_) {
//     return;
//   }
//   mm_->OnMarkPole(msg);
// }

}  // namespace loc
}  // namespace mp
}  // namespace hozon
