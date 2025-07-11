/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： ins_fusion_component.cpp
 *   author     ： nihongjie
 *   date       ： 2023.09
 ******************************************************************************/
#include "onboard/onboard_cyber/location/ins_fusion/ins_fusion_component.h"

#include "common/configs/config_gflags.h"
#include "modules/util/include/util/mapping_log.h"
#include "util/rviz_agent/rviz_agent.h"

DEFINE_string(ins_config, "conf/mapping/location/ins_fusion/ins_config.yaml",
              "ins_config");

DEFINE_string(ins_module_origin_ins_topic, "/minieye/origin_ins", "origin ins");
DEFINE_string(ins_module_inspva_topic, "/minieye/node_info_inspva",
              "inspva node topic");
DEFINE_string(ins_module_gnssvel_topic, "/minieye/gnssvel", "gnssvel topic");
DEFINE_string(ins_module_output_topic, "/mapping/location/ins", "output topic");

DEFINE_string(ins_viz_addr, "ipc:///tmp/rviz_agent_ins",
              "RvizAgent's working address, this should corresponds to the "
              "address used in RvizBridge. Leaving empty represents not using "
              "RvizAgent for visualization");

namespace hozon {
namespace mp {
namespace loc {

InsFusionComponent::~InsFusionComponent() {
  if (!FLAGS_ins_viz_addr.empty()) {
    mp::util::RvizAgent::Instance().Term();
  }
}

bool InsFusionComponent::Init() {
  if (!FLAGS_ins_viz_addr.empty()) {
    int ret = mp::util::RvizAgent::Instance().Init(FLAGS_ins_viz_addr);
    if (ret < 0) {
      HLOG_WARN << "RvizAgent init failed";
    }
  }
  ins_fusion_ = std::make_unique<InsFusion>();
  if (ins_fusion_->Init(FLAGS_ins_config) != InsInitStatus::OK) {
    HLOG_ERROR << "ins init failed";
    return false;
  }

  origin_ins_reader_ = node_->CreateReader<hozon::soc::ImuIns>(
      FLAGS_ins_module_origin_ins_topic,
      [this](const std::shared_ptr<const hozon::soc::ImuIns>& msg) {
        OnOriginIns(msg);
      });
  inspva_reader_ = node_->CreateReader<hozon::localization::HafNodeInfo>(
      FLAGS_ins_module_inspva_topic,
      [this](
          const std::shared_ptr<const hozon::localization::HafNodeInfo>& msg) {
        OnInspva(msg);
      });

  ins_writer_ = node_->CreateWriter<hozon::localization::HafNodeInfo>(
      FLAGS_ins_module_output_topic);
  return true;
}

void InsFusionComponent::OnOriginIns(
    const std::shared_ptr<const hozon::soc::ImuIns>& msg) {
  ins_fusion_->OnOriginIns(*msg);
  if (ins_fusion_->InsFusionState()) {
    return;
  }
  hozon::localization::HafNodeInfo result;
  if (!ins_fusion_->GetResult(&result)) {
    HLOG_ERROR << "ins core get result error";
    return;
  }

  ins_writer_->Write(
      std::make_shared<hozon::localization::HafNodeInfo>(result));
}

bool InsFusionComponent::OnInspva(
    const std::shared_ptr<const hozon::localization::HafNodeInfo>& msg) {
  if (!ins_fusion_) {
    return false;
  }
  ins_fusion_->OnInspva(*msg);
  if (!ins_fusion_->InsFusionState()) {
    return false;
  }
  hozon::localization::HafNodeInfo result;
  if (!ins_fusion_->GetResult(&result)) {
    HLOG_ERROR << "ins core get result error";
    return false;
  }

  ins_writer_->Write(
      std::make_shared<hozon::localization::HafNodeInfo>(result));
  return true;
}

}  // namespace loc
}  // namespace mp
}  // namespace hozon
