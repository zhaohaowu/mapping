/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： pose_estimation_lite.cc
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/
#include <base/utils/log.h>
#include <gflags/gflags.h>
#include <perception-lib/lib/environment/environment.h>

#include "modules/util/include/util/rviz_agent/rviz_agent.h"
#include "onboard/onboard_lite/location/pose_estimation/pose_estimation_lite.h"
#include "yaml-cpp/yaml.h"
#include "perception-base/base/state_machine/state_machine_info.h"

DEFINE_string(config_yaml,
              "runtime_service/mapping/conf/mapping/location/"
              "pose_estimation/pose_estimation_config.yaml",
              "path to pose estimation config yaml");
DEFINE_string(config_cam_yaml,
              "runtime_service/mapping/conf/mapping/location/"
              "pose_estimation/pose_estimation_cam.yaml",
              "path to pose estimation camera config yaml");
DEFINE_string(pose_estimation_lite_config_yaml,
              "runtime_service/mapping/conf/mapping/location/"
              "pose_estimation/pose_estimation_lite_config.yaml",
              "path to pose estimation camera config yaml");

using hozon::netaos::adf_lite::Bundle;

namespace hozon {
namespace perception {
namespace common_onboard {

constexpr char* const kInsFusionTopic = "/location/ins_fusion";
constexpr char* const kFcTopic = "localization";
constexpr char* const kPerceptionTopic = "percep_transport";
constexpr char* const kPoseEstimationTopic = "/location/pose_estimation";
constexpr char* const kRunningModeTopic = "running_mode";
int32_t PoseEstimationLite::AlgInit() {
  pose_estimation_ = std::make_unique<MapMatching>();
  const std::string adflite_root_path =
      hozon::perception::lib::GetEnv("ADFLITE_ROOT_PATH", ".");
  const std::string config_yaml = adflite_root_path + "/" + FLAGS_config_yaml;
  const std::string config_cam_yaml =
      adflite_root_path + "/" + FLAGS_config_cam_yaml;
  const std::string pose_estimation_lite_config_yaml =
      adflite_root_path + "/" + FLAGS_pose_estimation_lite_config_yaml;
  if (!pose_estimation_->Init(config_yaml, config_cam_yaml)) {
    return -1;
  }

  RegistMessageType();
  RegistAlgProcessFunc(
      "recv_localization",
      std::bind(&PoseEstimationLite::OnLocation, this, std::placeholders::_1));
  RegistAlgProcessFunc(
      "recv_ins_fusion",
      std::bind(&PoseEstimationLite::OnIns, this, std::placeholders::_1));
  RegistAlgProcessFunc("recv_perception",
                       std::bind(&PoseEstimationLite::OnPerception, this,
                                 std::placeholders::_1));
  RegistAlgProcessFunc("send_pose_estimation_result",
                       std::bind(&PoseEstimationLite::OnPoseEstimation, this,
                                 std::placeholders::_1));
  RegistAlgProcessFunc(
      "recv_running_mode",
      std::bind(&PoseEstimationLite::OnRunningMode, this, std::placeholders::_1));
  YAML::Node config = YAML::LoadFile(pose_estimation_lite_config_yaml);
  auto use_rviz_bridge = config["use_rviz_bridge"].as<bool>();
  if (use_rviz_bridge) {
    auto viz_addr = config["viz_addr"].as<std::string>();
    int ret = mp::util::RvizAgent::Instance().Init(viz_addr);
    if (ret < 0) {
      HLOG_WARN << "RvizAgent init failed:" << viz_addr;
    }
  }

  return 0;
}

void PoseEstimationLite::AlgRelease() {
  if (mp::util::RvizAgent::Instance().Ok()) {
    mp::util::RvizAgent::Instance().Term();
  }
}

void PoseEstimationLite::RegistMessageType() const {
  REGISTER_PROTO_MESSAGE_TYPE(kInsFusionTopic,
                              hozon::localization::HafNodeInfo);
  REGISTER_PROTO_MESSAGE_TYPE(kFcTopic, hozon::localization::Localization);
  REGISTER_PROTO_MESSAGE_TYPE(kPerceptionTopic,
                              hozon::perception::TransportElement);
  REGISTER_PROTO_MESSAGE_TYPE(kPoseEstimationTopic,
                              hozon::localization::HafNodeInfo);
  REGISTER_PROTO_MESSAGE_TYPE(kRunningModeTopic,
                              hozon::perception::common_onboard::running_mode);
}

int32_t PoseEstimationLite::OnIns(Bundle* input) {
  if (!input) {
    return -1;
  }

  auto p_ins_fusion = input->GetOne(kInsFusionTopic);
  if (!p_ins_fusion) {
    return -1;
  }

  const auto ins_fusion =
      std::static_pointer_cast<hozon::localization::HafNodeInfo>(
          p_ins_fusion->proto_msg);
  if (!ins_fusion) {
    return -1;
  }

  pose_estimation_->OnIns(ins_fusion);

  return 0;
}
int32_t PoseEstimationLite::OnLocation(Bundle* input) {
  if (!input) {
    return -1;
  }
  auto p_fc_fusion = input->GetOne(kFcTopic);
  if (!p_fc_fusion) {
    return -1;
  }
  const auto fc_fusion =
      std::static_pointer_cast<hozon::localization::Localization>(
          p_fc_fusion->proto_msg);
  if (!fc_fusion) {
    return -1;
  }
  pose_estimation_->OnLocation(fc_fusion);
  return 0;
}

int32_t PoseEstimationLite::OnPerception(Bundle* input) {
  if (!input) {
    return -1;
  }
  auto p_perception = input->GetOne(kPerceptionTopic);
  if (!p_perception) {
    return -1;
  }

  const auto perception =
      std::static_pointer_cast<hozon::perception::TransportElement>(
          p_perception->proto_msg);
  if (!perception) {
    return -1;
  }

  pose_estimation_->OnPerception(perception);

  return 0;
}

int32_t PoseEstimationLite::OnPoseEstimation(Bundle* input) {
  if (!input) {
    return -1;
  }
  const auto pe_node_info = pose_estimation_->getMmNodeInfo();
  if (!pe_node_info) {
    HLOG_ERROR << "onboard get pose estimation result error!";
    return -1;
  }

  auto pe_workflow = std::make_shared<hozon::netaos::adf_lite::BaseData>();
  pe_workflow->proto_msg = pe_node_info;
  SendOutput(kPoseEstimationTopic, pe_workflow);

  return 0;
}

int32_t PoseEstimationLite::OnRunningMode(Bundle* input) {
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
  if (runmode ==
      static_cast<int>(hozon::perception::base::RunningMode::PARKING)) {
    PauseTrigger("recv_ins_fusion");
    PauseTrigger("recv_perception");
    PauseTrigger("send_pose_estimation_result");
  } else if (runmode == static_cast<int>(
                            hozon::perception::base::RunningMode::DRIVING) ||
             runmode ==
                 static_cast<int>(hozon::perception::base::RunningMode::ALL)) {
    ResumeTrigger("recv_ins_fusion");
    ResumeTrigger("recv_perception");
    ResumeTrigger("send_pose_estimation_result");
  }
  return 0;
}

}  // namespace common_onboard
}  // namespace perception
}  // namespace hozon
