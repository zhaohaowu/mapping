/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： road_recognition.h
 *   author     ： zhangzhike
 *   date       ： 2023.12
 ******************************************************************************/

#pragma once

#include <yaml-cpp/yaml.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "Eigen/src/Core/Matrix.h"
#include "map_fusion/fusion_common/element_map.h"
#include "map_fusion/map_service/map_proto_maker.h"
#include "map_fusion/map_service/map_table.h"
#include "map_fusion/road_recognition/group_map.h"
#include "map_fusion/topo_assignment/topo_assignment.h"
#include "util/geo.h"
#include "util/rviz_agent/rviz_agent.h"

namespace hozon {
namespace mp {
namespace mf {

class GeoOptimization;
class TopoGeneration;
class RoutingGeneration;

class RoadRecognition {
 public:
  RoadRecognition() = default;
  ~RoadRecognition() = default;
  bool Init(const YAML::Node& conf);

  void OnLocalization(
      const std::shared_ptr<hozon::localization::Localization>& msg);
  void OnLocalMap(
      const std::shared_ptr<hozon::mapping::LocalMap>& msg,
      const std::shared_ptr<hozon::perception::PerceptionObstacles>& obj_msg,
      const std::pair<double, double>& map_speed_limit);
  std::shared_ptr<hozon::hdmap::Map> GetPercepMap();
  std::shared_ptr<hozon::routing::RoutingResponse> GetRouting();
  std::shared_ptr<hozon::mp::mf::em::ElementMapOut> GetElementMap();

 private:
  void AddRoadEdge(const std::shared_ptr<hozon::mapping::LocalMap>& msg,
                   const std::shared_ptr<hozon::hdmap::Map>& percep_map);
  // void OnElemtMap(const std::shared_ptr<>)
  std::shared_ptr<hozon::mp::mf::em::ElementMapOut> ele_map_ = nullptr;
  std::shared_ptr<GeoOptimization> geo_ = nullptr;
  std::shared_ptr<TopoGeneration> topo_ = nullptr;
  std::shared_ptr<hozon::hdmap::Map> percep_map_ = nullptr;
  std::shared_ptr<RoutingGeneration> rout_ = nullptr;
  std::shared_ptr<hozon::routing::RoutingResponse> routingresponse_ = nullptr;
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
