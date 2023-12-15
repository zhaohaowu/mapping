/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： topo_assignment_component.cc
 *   author     ： xuliang
 *   date       ： 2023.09
 ******************************************************************************/

#include "onboard/onboard_cyber/map_fusion/topo_assignment_component.h"
#include <gflags/gflags.h>

#include <iomanip>

#include "map_fusion/map_prediction/viz_map.h"
#include "map_fusion/topo_assignment/topo_assignment.h"
#include "util/mapping_log.h"

DEFINE_string(channel_topo_map, "mf/topo_map",
              "channel of topo map from topo assignment");
DEFINE_string(channel_ins_node_info_tac, "/PluginNodeInfo",
              "channel of ins msg from ins fusion");
DEFINE_string(channel_local_map, "/local_map",
              "channel of local map msg from local mapping");

DEFINE_string(viz_addr_topo, "ipc:///tmp/rviz_agent_lm_pv",
              "RvizAgent working address, may like "
              "ipc:///tmp/sample_rviz_agent or "
              "inproc://sample_rviz_agent or "
              "tcp://127.0.0.1:9100");

namespace hozon {
namespace mp {
namespace mf {

bool TopoAssignmentComponent::Init() {
  if (!FLAGS_viz_addr_topo.empty()) {
    int ret = RVIZ_AGENT.Init(FLAGS_viz_addr_topo);
    if (ret < 0) {
      HLOG_WARN << "RvizAgent init failed";
    }
  }
  topo_assign_ = std::make_shared<TopoAssignment>();
  topo_assign_->Init();

  topo_writer_ = node_->CreateWriter<hozon::hdmap::Map>(FLAGS_channel_topo_map);

  ins_reader_ = node_->CreateReader<hozon::localization::HafNodeInfo>(
      FLAGS_channel_ins_node_info_tac,
      [this](const std::shared_ptr<hozon::localization::HafNodeInfo>& msg) {
        OnInsNodeInfo(msg);
      });

  lm_reader_ = node_->CreateReader<hozon::mapping::LocalMap>(
      FLAGS_channel_local_map,
      [this](const std::shared_ptr<hozon::mapping::LocalMap>& msg) {
        OnLocalMap(msg);
      });

  return true;
}

void TopoAssignmentComponent::Clear() {
  if (!FLAGS_viz_addr_topo.empty() && RVIZ_AGENT.Ok()) {
    RVIZ_AGENT.Term();
  }
}

void TopoAssignmentComponent::OnInsNodeInfo(
    const std::shared_ptr<hozon::localization::HafNodeInfo>& msg) {
  if (!msg) {
    HLOG_ERROR << "message ins node info is null";
    return;
  }
  if (!topo_assign_) {
    HLOG_ERROR << "nullptr tppo map assignment";
    return;
  }
  // topo_assign_->OnInsNodeInfo(msg);
}

void TopoAssignmentComponent::OnLocalMap(
    const std::shared_ptr<hozon::mapping::LocalMap>& msg) {
  if (!msg) {
    HLOG_ERROR << "message local map is null";
    return;
  }
  if (!topo_assign_ || !topo_writer_) {
    HLOG_ERROR << "nullptr tppo map assignment or topo map writer";
    return;
  }
  // topo_assign_->OnLocalMap(msg);
  topo_assign_->TopoAssign();
  // 发出拓扑地图
  auto map = topo_assign_->GetTopoMap();
  topo_writer_->Write(map);
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
