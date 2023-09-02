/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： map_fusion_component.h
 *   author     ： taoshaoyuan
 *   date       ： 2023.08
 ******************************************************************************/

#pragma once

#include <adsfi_proto/internal/node_info.pb.h>
#include <adsfi_proto/sensors/sensors_imu.pb.h>
#include <adsfi_proto/sensors/sensors_ins.pb.h>
#include <adsfi_proto/vehicle/chassis_info.pb.h>
#include <cyber/cyber.h>

namespace hozon {
namespace mp {
namespace mf {

class MapFusion;

class MapFusionComponent final : public apollo::cyber::Component<> {
 public:
  MapFusionComponent() = default;
  ~MapFusionComponent();

 public:
  bool Init() override;

 private:
  void OnImu(const std::shared_ptr<adsfi_proto::hz_Adsfi::AlgIMU> &msg);

 private:
  std::shared_ptr<apollo::cyber::Reader<adsfi_proto::hz_Adsfi::AlgIMU>>
      raw_imu_reader_;

  std::shared_ptr<MapFusion> mf_ = nullptr;
};

CYBER_REGISTER_COMPONENT(MapFusionComponent);

}  // namespace mf
}  // namespace mp
}  // namespace hozon