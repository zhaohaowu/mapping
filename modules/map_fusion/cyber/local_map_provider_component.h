/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： local_map_provider_component.h
 *   author     ： xuliang
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <adsfi_proto/internal/node_info.pb.h>
#include <adsfi_proto/location/location.pb.h>
#include <adsfi_proto/perception/lanes.pb.h>
#include <cyber/cyber.h>
#include <depend/proto/local_mapping/local_map.pb.h>

#include <memory>

namespace hozon {
namespace mp {
namespace mf {

class LocalMapProvider;

class LocalMapProviderComponent final : public apollo::cyber::Component<> {
 public:
  LocalMapProviderComponent() = default;
  ~LocalMapProviderComponent() = default;

  bool Init() override;

 private:
  void OnInsNodeInfo(
      const std::shared_ptr<adsfi_proto::internal::HafNodeInfo>& msg);
  void OnLocation(
      const std::shared_ptr<adsfi_proto::hz_Adsfi::AlgLocation>& msg);
  void OnLaneLine(
      const std::shared_ptr<adsfi_proto::hz_Adsfi::AlgLaneDetectionOutArray>&
          msg);
  void OnRoadEdge(
      const std::shared_ptr<adsfi_proto::hz_Adsfi::AlgLaneDetectionOutArray>&
          msg);

  std::shared_ptr<apollo::cyber::Writer<hozon::mapping::LocalMap>> lm_writer_ =
      nullptr;
  std::shared_ptr<apollo::cyber::Reader<adsfi_proto::internal::HafNodeInfo>>
      ins_reader_ = nullptr;
  std::shared_ptr<apollo::cyber::Reader<adsfi_proto::hz_Adsfi::AlgLocation>>
      location_reader_ = nullptr;
  std::shared_ptr<
      apollo::cyber::Reader<adsfi_proto::hz_Adsfi::AlgLaneDetectionOutArray>>
      laneline_reader_ = nullptr;
  std::shared_ptr<
      apollo::cyber::Reader<adsfi_proto::hz_Adsfi::AlgLaneDetectionOutArray>>
      roadedge_reader_ = nullptr;

  std::shared_ptr<LocalMapProvider> lm_provider_ = nullptr;
};

CYBER_REGISTER_COMPONENT(LocalMapProviderComponent);

}  // namespace mf
}  // namespace mp
}  // namespace hozon
