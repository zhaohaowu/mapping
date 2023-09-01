/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： map_fusion_component.cc
 *   author     ： taoshaoyuan
 *   date       ： 2023.08
 ******************************************************************************/

#include "map_fusion_component.h"

namespace hozon {
namespace mp {
namespace mf {

bool MapFusionComponent::Init() {
  mf_ = std::make_shared<MapFusion>();
  mf_->Init("");

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