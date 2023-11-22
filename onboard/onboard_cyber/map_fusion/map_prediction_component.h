/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： map_prediction_component.h
 *   author     ： zhangshuo
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <adsfi_proto/internal/node_info.pb.h>
#include <cyber/cyber.h>
#include <depend/proto/local_mapping/local_map.pb.h>
#include <depend/proto/localization/node_info.pb.h>
#include <depend/proto/map/map.pb.h>

#include <memory>

#include "map_fusion/topo_assignment/topo_assignment.h"

namespace hozon {
namespace mp {
namespace mf {

class MapPrediction;

class MapPredictionComponent final : public apollo::cyber::Component<> {
 public:
  MapPredictionComponent() = default;
  ~MapPredictionComponent() = default;

  bool Init() override;
  void Clear() override;

 private:
  void OnInsNodeInfo(
      const std::shared_ptr<hozon::localization::HafNodeInfo>& msg);
  void OnHqMap(const std::shared_ptr<hozon::hdmap::Map>& msg);
  void OnTopoMap(const std::shared_ptr<hozon::hdmap::Map>& msg);

  std::shared_ptr<apollo::cyber::Writer<hozon::hdmap::Map>> pred_writer_ =
      nullptr;
  std::shared_ptr<apollo::cyber::Reader<hozon::localization::HafNodeInfo>>
      ins_reader_ = nullptr;
  std::shared_ptr<apollo::cyber::Reader<hozon::hdmap::Map>> hqMap_reader_ =
      nullptr;
  std::shared_ptr<apollo::cyber::Reader<hozon::hdmap::Map>> topo_reader_ =
      nullptr;
  std::shared_ptr<MapPrediction> prediction_ = nullptr;
};

CYBER_REGISTER_COMPONENT(MapPredictionComponent);

}  // namespace mf
}  // namespace mp
}  // namespace hozon
