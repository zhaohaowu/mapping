/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： map_fusion_component.h
 *   author     ： taoshaoyuan
 *   date       ： 2023.08
 ******************************************************************************/

#pragma once

#include <cyber/cyber.h>
#include <depend/proto/local_mapping/local_map.pb.h>
#include <depend/proto/localization/localization.pb.h>
#include <depend/proto/localization/node_info.pb.h>
#include <depend/proto/map/map.pb.h>

#include <memory>

namespace hozon {
namespace mp {
namespace mf {

class MapFusion;

class MapFusionComponent final : public apollo::cyber::Component<> {
 public:
  MapFusionComponent() = default;
  ~MapFusionComponent() override = default;

 public:
  bool Init() override;
  void Clear() override;

 private:
  void OnInsNodeInfo(
      const std::shared_ptr<hozon::localization::HafNodeInfo>& msg);
  void OnHQMap(const std::shared_ptr<hozon::hdmap::Map>& msg);
  void OnLocalMap(const std::shared_ptr<hozon::mapping::LocalMap>& msg);
  void OnLocalMapLocation(
      const std::shared_ptr<hozon::localization::Localization>& msg);

 private:
  std::shared_ptr<apollo::cyber::Writer<hozon::hdmap::Map>> map_writer_ =
      nullptr;
  std::shared_ptr<apollo::cyber::Reader<hozon::localization::HafNodeInfo>>
      ins_node_info_reader_ = nullptr;
  std::shared_ptr<apollo::cyber::Reader<hozon::hdmap::Map>> hq_map_reader_ =
      nullptr;
  std::shared_ptr<apollo::cyber::Reader<hozon::mapping::LocalMap>>
      local_map_reader_ = nullptr;
  std::shared_ptr<apollo::cyber::Reader<hozon::localization::Localization>>
      local_map_location_reader_ = nullptr;

  std::shared_ptr<MapFusion> mf_ = nullptr;
};

CYBER_REGISTER_COMPONENT(MapFusionComponent);

}  // namespace mf
}  // namespace mp
}  // namespace hozon
