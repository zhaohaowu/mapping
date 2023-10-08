/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： fc_component.cc
 *   author     ： lilanxing
 *   date       ： 2023.09
 ******************************************************************************/

#include "modules/location/coord_adapter/cyber/coord_adapter_component.h"

#include <gflags/gflags.h>

DEFINE_string(ca_localmap_in_topic, "/mapping/localmap",
              "local map recv topic");
DEFINE_string(ca_dr_in_topic, "/mapping/loc/dr_fusion", "dr recv topic");
DEFINE_string(ca_init_dr_out_topic, "/mapping/loc/init_dr_fusion",
              "init dr fusion topic");

DEFINE_string(coord_adapter_config,
              "conf/mapping/location/coord_adapter/config.yaml", "ca config");

namespace hozon {
namespace mp {
namespace loc {
namespace ca {

bool CoordAdapterComponent::Init() {
  coord_adapter_ = std::make_unique<CoordAdapter>();
  if (!coord_adapter_->Init(FLAGS_coord_adapter_config)) {
    return false;
  }

  local_map_reader_ = node_->CreateReader<LocalMap>(
      FLAGS_ca_localmap_in_topic,
      [this](const std::shared_ptr<const LocalMap>& msg) { OnLocalMap(msg); });

  dr_reader_ = node_->CreateReader<HafNodeInfo>(
      FLAGS_ca_dr_in_topic,
      [this](const std::shared_ptr<const HafNodeInfo>& msg) {
        OnDrFusion(msg);
      });

  init_dr_writer_ =
      node_->CreateWriter<HafNodeInfo>(FLAGS_ca_init_dr_out_topic);

  return true;
}

void CoordAdapterComponent::OnLocalMap(
    const std::shared_ptr<const LocalMap>& msg) {
  if (!msg) {
    return;
  }
  coord_adapter_->OnLocalMap(*msg);

  if (coord_adapter_->IsCoordInitSucc()) {
    const auto& init_dr = coord_adapter_->GetSysInitDrFusion();
    init_dr_writer_->Write(std::make_shared<HafNodeInfo>(init_dr));
  }
}

void CoordAdapterComponent::OnDrFusion(
    const std::shared_ptr<const HafNodeInfo>& msg) {
  if (!msg) {
    return;
  }
  coord_adapter_->OnDrFusion(*msg);
}

}  // namespace ca
}  // namespace loc
}  // namespace mp
}  // namespace hozon
