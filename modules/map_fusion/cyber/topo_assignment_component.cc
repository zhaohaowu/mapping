/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： topo_assignment_component.cc
 *   author     ： xuliang
 *   date       ： 2023.09
 ******************************************************************************/

#include "cyber/topo_assignment_component.h"

#include <gflags/gflags.h>

#include "map_fusion/topo_assignment/topo_assignment.h"
#include "util/temp_log.h"

DEFINE_string(channel_topo_map, "mf/topo_map",
              "channel of topo map from topo assignment");
DEFINE_string(channel_ins_node_info_tac, "ins_node_info",
              "channel of ins msg from ins fusion");
DEFINE_string(channel_hq_map, "hq_map",
              "channel of hq map msg from prior provider");
DEFINE_string(channel_local_map, "local_map",
              "channel of local map msg from local mapping");

namespace hozon {
namespace mp {
namespace mf {

bool TopoAssignmentComponent::Init() {
  topo_writer_ =
      node_->CreateWriter<hozon::hdmap::Map>(FLAGS_channel_topo_map);

  ins_reader_ = node_->CreateReader<adsfi_proto::internal::HafNodeInfo>(
      FLAGS_channel_ins_node_info_tac,
      [this](const std::shared_ptr<adsfi_proto::internal::HafNodeInfo>& msg) {
        OnInsNodeInfo(msg);
      });

  hq_reader_ = node_->CreateReader<hozon::hdmap::Map>(
      FLAGS_channel_hq_map,
      [this](const std::shared_ptr<hozon::hdmap::Map>& msg) { OnHQMap(msg); });

  lm_reader_ = node_->CreateReader<LocalMap>(
      FLAGS_channel_hq_map,
      [this](const std::shared_ptr<LocalMap>& msg) { OnLocalMap(msg); });

  topo_assign_ = std::make_shared<TopoAssignment>();
  return true;
}

void TopoAssignmentComponent::OnInsNodeInfo(
    const std::shared_ptr<adsfi_proto::internal::HafNodeInfo>& msg) {
  if (!topo_assign_ || !topo_writer_) {
    HLOG_ERROR << "nullptr tppo map assignment or topo map writer";
    return;
  }
  // topo_writer_->OnInsNodeInfo(msg);
}

void TopoAssignmentComponent::OnHQMap(
    const std::shared_ptr<hozon::hdmap::Map>& msg) {
  if (!topo_writer_) {
    HLOG_ERROR << "nullptr tppo map assignment";
    return;
  }
  // topo_writer_->OnHQMap(msg);
}

void TopoAssignmentComponent::OnLocalMap(const std::shared_ptr<LocalMap>& msg) {
  if (!topo_writer_) {
    HLOG_ERROR << "nullptr tppo map assignment";
    return;
  }
  // topo_writer_->OnLocalMap(msg);
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
