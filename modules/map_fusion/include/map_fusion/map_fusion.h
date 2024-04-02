/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： map_fusion.h
 *   author     ： taoshaoyuan
 *   date       ： 2023.08
 ******************************************************************************/

#pragma once

#include <depend/proto/local_mapping/local_map.pb.h>
#include <depend/proto/localization/localization.pb.h>
#include <depend/proto/localization/node_info.pb.h>
#include <depend/proto/map/map.pb.h>
#include <depend/proto/planning/planning.pb.h>
#include <depend/proto/routing/routing.pb.h>
#include <yaml-cpp/yaml.h>

#include <memory>
#include <string>
#include <thread>

#include "map_fusion/map_service/map_service_fault.h"

namespace hozon {
namespace mp {
namespace mf {

class TopoAssignment;
class MapPrediction;
class MapService;
class MapTable;
class RoadRecognition;

class MapFusion {
 public:
  MapFusion() = default;
  ~MapFusion() = default;
  int Init(const YAML::Node& conf);
  void Stop();

  int ProcService(
      const std::shared_ptr<hozon::localization::HafNodeInfo>& curr_node_info,
      const std::shared_ptr<hozon::planning::ADCTrajectory>& curr_planning,
      hozon::routing::RoutingResponse* routing);
  int ProcFusion(
      const std::shared_ptr<hozon::localization::Localization>& curr_loc,
      const std::shared_ptr<hozon::mapping::LocalMap>& curr_local_map,
      bool need_update_global_hd,
      std::shared_ptr<hozon::hdmap::Map>& fusion_map,  // NOLINT
      hozon::routing::RoutingResponse* routing);
  MapServiceFault GetMapServiceFault();
  int ProcPercep(
      const std::shared_ptr<hozon::localization::Localization>& curr_loc,
      const std::shared_ptr<hozon::mapping::LocalMap>& curr_local_map,
      hozon::hdmap::Map* fusion_map, hozon::routing::RoutingResponse* routing);

 private:
  std::shared_ptr<MapService> map_service_ = nullptr;
  std::shared_ptr<TopoAssignment> topo_ = nullptr;
  std::shared_ptr<MapPrediction> pred_ = nullptr;
  std::shared_ptr<MapTable> map_table_ = nullptr;
  std::shared_ptr<RoadRecognition> recog_ = nullptr;
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
