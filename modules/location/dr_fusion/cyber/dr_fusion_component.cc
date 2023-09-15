/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： dr_fusion_component.cpp
 *   author     ： nihongjie
 *   date       ： 2023.09
 ******************************************************************************/
#include "modules/location/dr_fusion/cyber/dr_fusion_component.h"

#include "common/configs/config_gflags.h"
#include "modules/util/include/util/temp_log.h"
#include "util/rviz_agent/rviz_agent.h"

DEFINE_string(dr_config, "conf/mapping/location/dr_fusion/dr_config.yaml",
              "dr_config");
DEFINE_string(inspva_module_input_topic, "/mapping/location/ins",
              "input topic");
DEFINE_string(dr_module_input_topic, "/mapping/dr", "input topic");
DEFINE_string(loc_dr_module_output_topic, "/mapping/location/dr",
              "output topic");
DEFINE_string(viz_addr, "ipc:///tmp/rviz_agent_dr",
              "RvizAgent's working address, this should corresponds to the "
              "address used in RvizBridge. Leaving empty represents not using "
              "RvizAgent for visualization");

namespace hozon {
namespace mp {
namespace loc {

DrFusionComponent::~DrFusionComponent() {
  if (!FLAGS_viz_addr.empty()) {
    mp::util::RvizAgent::Instance().Term();
  }
}

bool DrFusionComponent::Init() {
  if (!FLAGS_viz_addr.empty()) {
    int ret = mp::util::RvizAgent::Instance().Init(FLAGS_viz_addr);
    if (ret < 0) {
      HLOG_WARN << "RvizAgent init failed";
    }
  }
  dr_fusion_ = std::make_unique<DrFusion>();
  if (dr_fusion_->Init(FLAGS_dr_config) != InsInitStatus::OK) {
    HLOG_ERROR << "dr init failed";
    return false;
  }
  inspva_reader_ = node_->CreateReader<HafNodeInfo>(
      FLAGS_inspva_module_input_topic,
      [this](const std::shared_ptr<const HafNodeInfo>& msg) { OnInspva(msg); });
  dr_reader_ = node_->CreateReader<HafNodeInfo>(
      FLAGS_dr_module_input_topic,
      [this](const std::shared_ptr<const HafNodeInfo>& msg) { OnDr(msg); });
  loc_dr_writer_ =
      node_->CreateWriter<HafNodeInfo>(FLAGS_loc_dr_module_output_topic);
  return true;
}

bool DrFusionComponent::OnInspva(
    const std::shared_ptr<const HafNodeInfo>& msg) {
  if (!dr_fusion_) {
    return false;
  }
  dr_fusion_->OnInspva(*msg);

  adsfi_proto::internal::HafNodeInfo result;
  if (!dr_fusion_->GetResult(&result)) {
    HLOG_ERROR << "ins core get result error";
    return false;
  }

  loc_dr_writer_->Write(
      std::make_shared<adsfi_proto::internal::HafNodeInfo>(result));

  return true;
}

bool DrFusionComponent::OnDr(const std::shared_ptr<const HafNodeInfo>& msg) {
  if (!dr_fusion_) {
    return false;
  }
  dr_fusion_->OnDr(*msg);
  return true;
}

}  // namespace loc
}  // namespace mp
}  // namespace hozon
