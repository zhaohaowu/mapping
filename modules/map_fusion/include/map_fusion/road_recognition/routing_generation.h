/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： routing_generation.h
 *   author     ： zhangzhike
 *   date       ： 2024.1
 ******************************************************************************/

#pragma once
#include <proto/map/map.pb.h>
#include <yaml-cpp/yaml.h>

#include <memory>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "Eigen/src/Core/Matrix.h"
#include "map_fusion/fusion_common/element_map.h"
#include "map_fusion/topo_assignment/topo_assignment.h"
#include "util/geo.h"
#include "util/rviz_agent/rviz_agent.h"

namespace hozon {
namespace mp {
namespace mf {

class RoutingGeneration {
 public:
  RoutingGeneration() = default;
  ~RoutingGeneration() = default;
  int Init();
  void Generate(const std::shared_ptr<hozon::hdmap::Map>& percep_map);
  std::shared_ptr<hozon::routing::RoutingResponse> GetRouting();

 private:
  std::shared_ptr<hozon::routing::RoutingResponse> routing_ = nullptr;
  std::shared_ptr<hozon::hdmap::Map> percep_map_ = nullptr;
  double viz_lifetime_ = 0;
  void SetRouting();

  template <
      typename T,
      typename std::enable_if<
          std::is_base_of<google::protobuf::Message, T>::value, int>::type = 0>
  void FillHeader(const std::string& module_name, T* msg);
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
