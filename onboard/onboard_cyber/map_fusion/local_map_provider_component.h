/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： local_map_provider_component.h
 *   author     ： xuliang
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <cyber/cyber.h>
#include <depend/proto/local_mapping/local_map.pb.h>
#include <depend/proto/localization/localization.pb.h>
#include <depend/proto/localization/node_info.pb.h>
#include <depend/proto/perception/transport_element.pb.h>

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
      const std::shared_ptr<hozon::localization::HafNodeInfo>& msg);
  void OnLocation(
      const std::shared_ptr<hozon::localization::Localization>& msg);
  void OnLaneLine(
      const std::shared_ptr<hozon::perception::TransportElement>& msg);
  void OnRoadEdge(
      const std::shared_ptr<hozon::perception::TransportElement>& msg);

  std::shared_ptr<apollo::cyber::Writer<hozon::mapping::LocalMap>> lm_writer_ =
      nullptr;
  std::shared_ptr<apollo::cyber::Reader<hozon::localization::HafNodeInfo>>
      ins_reader_ = nullptr;
  std::shared_ptr<apollo::cyber::Reader<hozon::localization::Localization>>
      location_reader_ = nullptr;
  std::shared_ptr<apollo::cyber::Reader<hozon::perception::TransportElement>>
      laneline_reader_ = nullptr;
  std::shared_ptr<apollo::cyber::Reader<hozon::perception::TransportElement>>
      roadedge_reader_ = nullptr;

  std::shared_ptr<LocalMapProvider> lm_provider_ = nullptr;
};

CYBER_REGISTER_COMPONENT(LocalMapProviderComponent);

}  // namespace mf
}  // namespace mp
}  // namespace hozon
