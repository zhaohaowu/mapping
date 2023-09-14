/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： ins_fusion_component.cpp
 *   author     ： nihongjie
 *   date       ： 2023.09
 ******************************************************************************/
#include "modules/location/ins_fusion/cyber/ins_fusion_component.h"

#include "common/configs/config_gflags.h"
#include "modules/util/include/util/temp_log.h"
#include "util/rviz_agent/rviz_agent.h"

DEFINE_string(ins_config, "conf/mapping/location/ins_fusion/ins_config.yaml",
              "ins_config");

DEFINE_string(ins_module_imu_topic, "/minieye/imu", "imu topic");
DEFINE_string(ins_module_origin_ins_topic, "/minieye/origin_ins", "origin ins");
DEFINE_string(ins_module_inspva_topic, "/minieye/node_info_inspva",
              "inspva node topic");
DEFINE_string(ins_module_gnssvel_topic, "/minieye/gnssvel", "gnssvel topic");
DEFINE_string(ins_module_output_topic, "/minieye/node_info_ins",
              "output topic");
DEFINE_string(ins_fault_output_topic, "/localization/modules_fault",
              "ins fault output topic");

DEFINE_bool(ins_pub_localization_estimate, false,
            "ins publish localization estimate to dreamview");
DEFINE_string(localization_dv_ins_topic, "/localization/estimate/ins",
              "dv ins topic");
DEFINE_string(viz_addr, "ipc:///tmp/rviz_agent_ins",
              "RvizAgent's working address, this should corresponds to the "
              "address used in RvizBridge. Leaving empty represents not using "
              "RvizAgent for visualization");

namespace hozon {
namespace mp {
namespace loc {

InsFusionComponent::~InsFusionComponent() {
  if (!FLAGS_viz_addr.empty()) {
    mp::util::RvizAgent::Instance().Term();
  }
}

bool InsFusionComponent::Init() {
  if (!FLAGS_viz_addr.empty()) {
    int ret = mp::util::RvizAgent::Instance().Init(FLAGS_viz_addr);
    if (ret < 0) {
      HLOG_WARN << "RvizAgent init failed";
    }
  }
  ins_fusion_ = std::make_unique<InsFusion>();
  if (ins_fusion_->Init(FLAGS_ins_config) != InsInitStatus::OK) {
    HLOG_ERROR << "ins init failed";
    return false;
  }

  origin_ins_reader_ = node_->CreateReader<AlgInsInfo>(
      FLAGS_ins_module_origin_ins_topic,
      [this](const std::shared_ptr<const AlgInsInfo>& msg) {
        OnOriginIns(msg);
      });
  inspva_reader_ = node_->CreateReader<HafNodeInfo>(
      FLAGS_ins_module_inspva_topic,
      [this](const std::shared_ptr<const HafNodeInfo>& msg) { OnInspva(msg); });

  ins_writer_ = node_->CreateWriter<HafNodeInfo>(FLAGS_ins_module_output_topic);
  if (FLAGS_ins_pub_localization_estimate) {
    HLOG_DEBUG << "ins localization topic: " << FLAGS_localization_dv_ins_topic;
    localization_writer_ =
        node_->CreateWriter<hozon::localization::LocalizationEstimate>(
            FLAGS_localization_dv_ins_topic);
  }
  return true;
}

void InsFusionComponent::OnOriginIns(
    const std::shared_ptr<const AlgInsInfo>& msg) {
  ins_fusion_->OnOriginIns(*msg);
}

bool InsFusionComponent::OnInspva(
    const std::shared_ptr<const HafNodeInfo>& msg) {
  if (!ins_fusion_) {
    return false;
  }
  ins_fusion_->OnInspva(*msg);

  adsfi_proto::internal::HafNodeInfo result;
  if (!ins_fusion_->GetResult(&result)) {
    HLOG_ERROR << "ins core get result error";
    return false;
  }

  ins_writer_->Write(
      std::make_shared<adsfi_proto::internal::HafNodeInfo>(result));

  if (FLAGS_ins_pub_localization_estimate) {
    auto estimate_ptr =
        std::make_shared<hozon::localization::LocalizationEstimate>();
    if (ins_fusion_->GetLocalizationEstimate(estimate_ptr.get())) {
      localization_writer_->Write(estimate_ptr);
    }
  }

  return true;
}

}  // namespace loc
}  // namespace mp
}  // namespace hozon
