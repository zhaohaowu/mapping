/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： map_prediction_component.h
 *   author     ： zhangshuo
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <adsfi_proto/internal/node_info.pb.h>
#include <adsfi_proto/map/local_map.pb.h>
#include <cyber/cyber.h>
#include <depend/proto/map/map.pb.h>

#include <memory>

namespace hozon {
namespace mp {
namespace mf {

class MapPrediction;

class MapPredictionComponent final : public apollo::cyber::Component<> {
 public:
  MapPredictionComponent() = default;
  ~MapPredictionComponent() = default;

  bool Init() override;

 private:
  void OnInsNodeInfo(
      const std::shared_ptr<adsfi_proto::internal::HafNodeInfo>& msg);
  //   void OnLocalMap(const std::shared_ptr<const hozon::hdmap::Map>&
  //   localMap);
  void OnHqMap(const std::shared_ptr<const hozon::hdmap::Map>& hqMap);
  void OnLocalMap(const std::shared_ptr<hozon::mapping::LocalMap>& msg);

  std::shared_ptr<apollo::cyber::Reader<adsfi_proto::internal::HafNodeInfo>>
      ins_reader_ = nullptr;
  //   std::shared_ptr<apollo::cyber::Reader<hozon::hdmap::Map>>
  //   localMap_reader_ =
  //       nullptr;
  std::shared_ptr<apollo::cyber::Reader<hozon::hdmap::Map>> hqMap_reader_ =
      nullptr;
  std::shared_ptr<apollo::cyber::Reader<hozon::mapping::LocalMap>> lm_reader_ =
      nullptr;
  std::shared_ptr<MapPrediction> prediction_ = nullptr;
};

CYBER_REGISTER_COMPONENT(MapPredictionComponent);

}  // namespace mf
}  // namespace mp
}  // namespace hozon
