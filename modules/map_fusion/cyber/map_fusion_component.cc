/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： map_fusion_component.cc
 *   author     ： taoshaoyuan
 *   date       ： 2023.08
 ******************************************************************************/

#include "map_fusion_component.h"

#include <gflags/gflags.h>

#include "map_fusion/map_fusion.h"
#include "util/rviz_agent/rviz_agent.h"

DEFINE_string(mf_config, "conf/mapping/map_fusion/map_fusion.yaml",
              "path to map fusion's config yaml");
DEFINE_string(mf_viz, "ipc:///tmp/rviz_agent_mf",
              "RvizAgent's working address, this should corresponds to the "
              "address used in RvizBridge. Leaving empty represents not using "
              "RvizAgent for visualization");

namespace hozon {
namespace mp {
namespace mf {

MapFusionComponent::~MapFusionComponent() {
  if (!FLAGS_mf_viz.empty() &&
      RVIZ_AGENT.Ok()) {
    RVIZ_AGENT.Term();
  }
}

bool MapFusionComponent::Init() {
  if (!FLAGS_mf_viz.empty() &&
      !RVIZ_AGENT.Ok()) {
    int ret = RVIZ_AGENT.Init(FLAGS_mf_viz);
    if (ret < 0) {
      HLOG_WARN << "RvizAgent init failed";
    }
  }

  mf_ = std::make_shared<MapFusion>();
  mf_->Init(FLAGS_mf_config);

  raw_imu_reader_ = node_->CreateReader<adsfi_proto::hz_Adsfi::AlgIMU>(
      "/imu",
      [this](const std::shared_ptr<adsfi_proto::hz_Adsfi::AlgIMU> &msg) {
        OnImu(msg);
      });
  return true;
}

void MapFusionComponent::OnImu(
    const std::shared_ptr<adsfi_proto::hz_Adsfi::AlgIMU> &msg) {
  return;
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon