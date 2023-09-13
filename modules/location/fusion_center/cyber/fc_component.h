/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： fc_component.h
 *   author     ： lilanxing
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include "modules/location/fusion_center/cyber/fc_component.h"
#include <cyber/cyber.h>
#include <cyber/component/component.h>
#include <memory>
#include "modules/location/fusion_center/lib/fusion_center.h"

namespace hozon {
namespace mp {
namespace loc {
namespace fc {

using apollo::cyber::Reader;
using apollo::cyber::Writer;

class FcComponent : public apollo::cyber::Component<> {
 public:
  FcComponent() = default;
  ~FcComponent() = default;

  bool Init() override;
  void OnImu(const std::shared_ptr<const AlgIMU>& msg);
  void OnInsFusion(const std::shared_ptr<const HafNodeInfo>& msg);
  void OnDrFusion(const std::shared_ptr<const HafNodeInfo>& msg);
  void OnPoseEstimation(const std::shared_ptr<const HafNodeInfo>& msg);

 private:
  std::shared_ptr<Reader<AlgIMU>> imu_reader_ = nullptr;
  std::shared_ptr<Reader<HafNodeInfo>> ins_reader_ = nullptr;
  std::shared_ptr<Reader<HafNodeInfo>> dr_reader_ = nullptr;
  std::shared_ptr<Reader<HafNodeInfo>> pe_reader_ = nullptr;
  std::shared_ptr<Writer<AlgLocation>> fc_writer_ = nullptr;

  std::shared_ptr<FusionCenter> fc_ = nullptr;
};

CYBER_REGISTER_COMPONENT(FcComponent);

}  // namespace fc
}  // namespace loc
}  // namespace mp
}  // namespace hozon
