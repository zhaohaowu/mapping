/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： pose_estimation_lite.cc
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/

#include "onboard/onboard_lite/location/pose_estimation/pose_estimation_lite.h"
#include <perception-lib/lib/environment/environment.h>
#include <base/utils/log.h>
#include <gflags/gflags.h>

DEFINE_string(config_yaml,
              "runtime_service/mapping/conf/mapping/location/"
              "pose_estimation/pose_estimation_config.yaml",
              "path to pose estimation config yaml");
DEFINE_string(config_cam_yaml,
              "runtime_service/mapping/conf/mapping/location/"
              "pose_estimation/pose_estimation_cam.yaml",
              "path to pose estimation camera config yaml");
DEFINE_string(viz_addr, "ipc:///tmp/rviz_agent_pose_estimation",
              "RvizAgent's working address, this should corresponds to the "
              "address used in RvizBridge. Leaving empty represents not using "
              "RvizAgent for visualization");

namespace hozon {
namespace perception {
namespace common_onboard {

constexpr char* const kInsFusionTopic = "/location/ins_fusion";
// constexpr char* const kFcTopic = "localization";
constexpr char* const kPerceptionTopic = "percep_transport";
constexpr char* const kPoseEstimationTopic = "/location/pose_estimation";
int32_t PoseEstimationLite::AlgInit() {
  pose_estimation_ = std::make_unique<MapMatching>();
    const std::string adflite_root_path =
      hozon::perception::lib::GetEnv("ADFLITE_ROOT_PATH", ".");
  const std::string config_yaml =
      adflite_root_path + "/" + FLAGS_config_yaml;
  const std::string config_cam_yaml =
      adflite_root_path + "/" + FLAGS_config_cam_yaml;
  if (!pose_estimation_->Init(config_yaml, config_cam_yaml)) {
    return -1;
  }

  RegistLog();
  RegistMessageType();
  RegistAlgProcessFunc(
      "recv_ins_fusion",
      std::bind(&PoseEstimationLite::OnIns, this, std::placeholders::_1));
  // RegistAlgProcessFunc(
  //     "recv_map_message",
  //     std::bind(&PoseEstimationLite::OnHdMap, this, std::placeholders::_1));
  RegistAlgProcessFunc("recv_perception",
                       std::bind(&PoseEstimationLite::OnPerception, this,
                                 std::placeholders::_1));
  RegistAlgProcessFunc("send_pose_estimation_result",
                       std::bind(&PoseEstimationLite::OnPoseEstimation, this,
                                 std::placeholders::_1));
  return 0;
}

void PoseEstimationLite::AlgRelease() {}

void PoseEstimationLite::RegistLog() const {
  hozon::netaos::log::InitLogging("loc_pe", "pose_estimation",
                                  hozon::netaos::log::LogLevel::kInfo,
                                  HZ_LOG2CONSOLE, "./", 10, (20));
  hozon::netaos::adf::NodeLogger::GetInstance().CreateLogger(
      "loc_pe", "pose_estimation", hozon::netaos::log::LogLevel::kInfo);
}

void PoseEstimationLite::RegistMessageType() const {
  REGISTER_MESSAGE_TYPE(kInsFusionTopic, hozon::localization::HafNodeInfo);
  // REGISTER_MESSAGE_TYPE(kFcTopic, hozon::localization::Localization);
  // REGISTER_MESSAGE_TYPE(kMapMessageTopic, adsfi_proto::internal::SubMap);
  REGISTER_MESSAGE_TYPE(kPerceptionTopic, hozon::perception::TransportElement);
  REGISTER_MESSAGE_TYPE(kPoseEstimationTopic, hozon::localization::HafNodeInfo);
}

int32_t PoseEstimationLite::OnIns(Bundle* input) {
  if (!input) {
    return -1;
  }

  BaseDataTypePtr p_ins_fusion = input->GetOne(kInsFusionTopic);
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

// int32_t PoseEstimationLite::OnHdMap(Bundle* input) {
//   if (!input) {
//     return -1;
//   }

//   BaseDataTypePtr p_map_message = input->GetOne("kMapMessageTopic");
//   if (!p_map_message) {
//     return -1;
//   }

//   const auto map_message =
//       std::static_pointer_cast<adsfi_proto::internal::SubMap>(
//           p_map_message->proto_msg);
//   if (!map_message) {
//     return -1;
//   }

//   pose_estimation_->OnHdMap(map_message);

//   return 0;
// }

int32_t PoseEstimationLite::OnPerception(Bundle* input) {
  if (!input) {
    return -1;
  }
  BaseDataTypePtr p_perception = input->GetOne(kPerceptionTopic);
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

  BaseDataTypePtr pe_workflow =
      std::make_shared<hozon::netaos::adf_lite::BaseData>();
  pe_workflow->proto_msg = pe_node_info;
  SendOutput(kPoseEstimationTopic, pe_workflow);

  return 0;
}

}  // namespace common_onboard
}  // namespace perception
}  // namespace hozon
