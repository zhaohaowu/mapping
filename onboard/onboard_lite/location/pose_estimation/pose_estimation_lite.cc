/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： pose_estimation_lite.cc
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/
#include "onboard/onboard_lite/location/pose_estimation/pose_estimation_lite.h"
#include <base/utils/log.h>
#include <gflags/gflags.h>
#include <perception-lib/lib/environment/environment.h>

#include <memory>
#include <utility>

#include "depend/nos/x86_2004/include/adf/include/node_proto_register.h"
#include "depend/perception-lib/lib/fault_manager/fault_manager.h"
#include "modules/location/pose_estimation/lib/pose_estimation.h"
#include "modules/util/include/util/rviz_agent/rviz_agent.h"
#include "onboard/onboard_lite/phm_comment_lite/proto/running_mode.pb.h"
#include "perception-base/base/state_machine/state_machine_info.h"

DEFINE_string(pose_estimation_yaml,
              "runtime_service/mapping/conf/mapping/location/"
              "pose_estimation/pose_estimation.yaml",
              "path to pose_estimation config yaml");
DEFINE_string(map_matching_yaml,
              "runtime_service/mapping/conf/mapping/location/"
              "pose_estimation/map_matching.yaml",
              "path to map_matching config yaml");
DEFINE_string(mapping_location_pose_estimation_config_yaml,
              "runtime_service/mapping/conf/lite/location/"
              "mapping_location_pose_estimation_config.yaml",
              "path to mapping_location_pose_estimation_config_yaml yaml");

using hozon::netaos::adf_lite::Bundle;

namespace hozon {
namespace perception {
namespace common_onboard {

constexpr char* const kPoseEstimationTopic = "/location/pose_estimation";
int32_t PoseEstimationLite::AlgInit() {
  pose_estimation_ = std::make_unique<PoseEstimation>();
  const std::string adflite_root_path =
      hozon::perception::lib::GetEnv("ADFLITE_ROOT_PATH", ".");
  const std::string pose_estimation_yaml =
      adflite_root_path + "/" + FLAGS_pose_estimation_yaml;
  const std::string map_matching_yaml =
      adflite_root_path + "/" + FLAGS_map_matching_yaml;
  const std::string mapping_location_pose_estimation_config_yaml =
      adflite_root_path + "/" +
      FLAGS_mapping_location_pose_estimation_config_yaml;
  if (!pose_estimation_->Init(pose_estimation_yaml, map_matching_yaml)) {
    return -1;
  }

  RegistMessageType();
  RegistAlgProcessFunc("recv_localization", [this](auto&& PH1) {
    return OnLocation(std::forward<decltype(PH1)>(PH1));
  });
  RegistAlgProcessFunc("recv_ins_fusion", [this](auto&& PH1) {
    return OnIns(std::forward<decltype(PH1)>(PH1));
  });
  RegistAlgProcessFunc("recv_perception", [this](auto&& PH1) {
    return OnPerception(std::forward<decltype(PH1)>(PH1));
  });
  RegistAlgProcessFunc("send_pose_estimation_result", [this](auto&& PH1) {
    return OnPoseEstimation(std::forward<decltype(PH1)>(PH1));
  });
  RegistAlgProcessFunc("recv_running_mode", [this](auto&& PH1) {
    return OnRunningMode(std::forward<decltype(PH1)>(PH1));
  });
  ExtractCmParameter(mapping_location_pose_estimation_config_yaml);
  return 0;
}

void PoseEstimationLite::ExtractCmParameter(const std::string& yamlpath) {
  YAML::Node cm_yaml_config = YAML::LoadFile(yamlpath);
  for (const auto& triggerNode : cm_yaml_config["trigger"]) {
    auto triggername = triggerNode["name"].as<std::string>();
    auto typeStr = triggerNode["type"].as<std::string>();
    if (typeStr == "EVENT") {
      for (const auto& sourceNode : triggerNode["mainSources"]) {
        if (triggername == "recv_perception") {
          kPerceptionTopic_ = sourceNode["name"].as<std::string>();
        }
        if (triggername == "recv_ins_fusion") {
          kinsFusionTopic_ = sourceNode["name"].as<std::string>();
        }
        if (triggername == "recv_running_mode") {
          kRunningModeTopic_ = sourceNode["name"].as<std::string>();
        }
        if (triggername == "recv_localization") {
          kFcTopic_ = sourceNode["name"].as<std::string>();
        }
      }
    } else if (typeStr == "PERIOD") {
    } else {
      HLOG_ERROR << "Invalid trigger type";
      continue;
    }
  }

  HLOG_ERROR << " kPoseEstimationTopic: " << kPoseEstimationTopic
             << " kPerceptionTopic_ " << kPerceptionTopic_
             << " kinsFusionTopic_ " << kinsFusionTopic_
             << " kRunningModeTopic_ " << kRunningModeTopic_ << " kFcTopic_ "
             << kFcTopic_;
}

void PoseEstimationLite::AlgRelease() {
  if (mp::util::RvizAgent::Instance().Ok()) {
    mp::util::RvizAgent::Instance().Term();
  }
}

void PoseEstimationLite::RegistMessageType() const {
  REGISTER_PROTO_MESSAGE_TYPE(kinsFusionTopic_,
                              hozon::localization::HafNodeInfo);
  REGISTER_PROTO_MESSAGE_TYPE(kFcTopic_, hozon::localization::Localization);
  REGISTER_PROTO_MESSAGE_TYPE(kPerceptionTopic_,
                              hozon::perception::TransportElement);
  REGISTER_PROTO_MESSAGE_TYPE(kPoseEstimationTopic,
                              hozon::localization::HafNodeInfo);
  REGISTER_PROTO_MESSAGE_TYPE(kRunningModeTopic_,
                              hozon::perception::common_onboard::running_mode);
}

int32_t PoseEstimationLite::OnIns(Bundle* input) {
  static int ins_count = 0;
  if (input == nullptr) {
    return -1;
  }

  auto p_ins_fusion = input->GetOne(kinsFusionTopic_);
  if (!p_ins_fusion) {
    return -1;
  }

  const auto ins_fusion =
      std::static_pointer_cast<hozon::localization::HafNodeInfo>(
          p_ins_fusion->proto_msg);
  if (!ins_fusion) {
    return -1;
  }
  ins_count++;
  if (ins_count >= 100) {
    ins_count = 0;
    HLOG_INFO << "rev ins heartbeat";
  }
  pose_estimation_->OnIns(ins_fusion);

  return 0;
}

int32_t PoseEstimationLite::OnLocation(Bundle* input) {
  static int location_count = 0;
  if (input == nullptr) {
    return -1;
  }
  auto p_fc_fusion = input->GetOne(kFcTopic_);
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
  location_count++;
  if (location_count >= 100) {
    location_count = 0;
    HLOG_INFO << "rev location heartbeat";
  }
  return 0;
}

int32_t PoseEstimationLite::OnPerception(Bundle* input) {
  static int perception_count = 0;
  if (input == nullptr) {
    return -1;
  }
  static double last_percep_time = -1.0;
  auto* phm_fault = hozon::perception::lib::FaultManager::Instance();
  auto p_perception = input->GetOne(kPerceptionTopic_);
  if (p_perception == nullptr) {
    phm_fault->Report(MAKE_FM_TUPLE(
        hozon::perception::base::FmModuleId::MAPPING,
        hozon::perception::base::FaultType::
            MULTI_FRAME_PERCEPTION_INPUT_DATA_LOSS,
        hozon::perception::base::FaultStatus::OCCUR,
        hozon::perception::base::SensorOrientation::UNKNOWN, 3, 500));
    HLOG_ERROR << "Location:perception input data loss";
    return -1;
  }
  phm_fault->Report(
      MAKE_FM_TUPLE(hozon::perception::base::FmModuleId::MAPPING,
                    hozon::perception::base::FaultType::
                        MULTI_FRAME_PERCEPTION_INPUT_DATA_LOSS,
                    hozon::perception::base::FaultStatus::RESET,
                    hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));

  const auto perception =
      std::static_pointer_cast<hozon::perception::TransportElement>(
          p_perception->proto_msg);
  if (!perception) {
    return -1;
  }
  double cur_percep_time = perception->header().data_stamp();
  if (last_percep_time > 0) {
    if (last_percep_time - cur_percep_time > 0) {
      phm_fault->Report(MAKE_FM_TUPLE(
          hozon::perception::base::FmModuleId::MAPPING,
          hozon::perception::base::FaultType::
              MULTI_FRAME_PERCEPTION_INPUT_TIME_ERROR,
          hozon::perception::base::FaultStatus::OCCUR,
          hozon::perception::base::SensorOrientation::UNKNOWN, 3, 500));
      HLOG_ERROR << "Location:receieve perception time error";
    } else {
      phm_fault->Report(MAKE_FM_TUPLE(
          hozon::perception::base::FmModuleId::MAPPING,
          hozon::perception::base::FaultType::
              MULTI_FRAME_PERCEPTION_INPUT_TIME_ERROR,
          hozon::perception::base::FaultStatus::RESET,
          hozon::perception::base::SensorOrientation::UNKNOWN, 0, 0));
    }
  }
  last_percep_time = cur_percep_time;
  ++perception_count;
  if (perception_count >= 1) {
    perception_count = 0;
    HLOG_INFO << "rev perception heartbeat";
  }

  pose_estimation_->OnPerception(perception);

  return 0;
}

int32_t PoseEstimationLite::OnPoseEstimation(Bundle* input) {
  static int pe_count = 0;
  if (input == nullptr) {
    return -1;
  }
  const auto pe_node_info = pose_estimation_->GetMmNodeInfo();
  if (!pe_node_info) {
    HLOG_ERROR << "onboard get pose estimation result error!";
    return -1;
  }

  auto pe_workflow = std::make_shared<hozon::netaos::adf_lite::BaseData>();
  pe_workflow->proto_msg = pe_node_info;
  SendOutput(kPoseEstimationTopic, pe_workflow);
  ++pe_count;
  if (pe_count >= 1) {
    pe_count = 0;
    HLOG_INFO << "send pe heartbeat";
  }
  return 0;
}

int32_t PoseEstimationLite::OnRunningMode(Bundle* input) {
  static int running_mode_count = 0;
  auto rm_msg = input->GetOne(kRunningModeTopic_);
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
  ++running_mode_count;
  if (running_mode_count >= 100) {
    running_mode_count = 0;
    HLOG_INFO << "on running model heartbeat";
  }
  return 0;
}

}  // namespace common_onboard
}  // namespace perception
}  // namespace hozon
