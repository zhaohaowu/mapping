/***************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： geo_optimization_rviz.h
 *   author     ： ouyanghailin
 *   date       ： 2024.08
 ******************************************************************************/
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "depend/proto/local_mapping/local_map.pb.h"
#include "modules/map_fusion_02/base/element_map.h"
#include "modules/util/include/util/rviz_agent/rviz_agent.h"

namespace hozon {
namespace mp {
namespace mf {

class GeoOptimizationViz {
 public:
  GeoOptimizationViz() = default;
  ~GeoOptimizationViz() = default;

  // void VizLocalMap(bool road_recognition_rviz_switch,
  //                  const std::shared_ptr<hozon::mapping::LocalMap>&
  //                  local_map, const std::string& viz_topic, const
  //                  Eigen::Isometry3d& T_U_V);
  void VizElementMap(ElementMap::Ptr element_map,
                                       const Eigen::Affine3d& T);

 private:
  void PointsToMarker(const double stamp,
                      const std::vector<Eigen::Vector3d>& points,
                      const std::vector<float>& color_type,
                      adsfi_proto::viz::Marker* marker);
  void LineIdToMarker(const double stamp, const Eigen::Vector3d& point,
                      const std::string& id, adsfi_proto::viz::Marker* marker);

 private:
  const std::unordered_map<char, std::vector<float>> color_palette = {
      {'r', {1.0, 0.0, 0.0}}, {'g', {0.0, 1.0, 0.0}},   {'b', {0.0, 0.0, 1.0}},
      {'w', {1.0, 1.0, 1.0}}, {'o', {1.0, 0.647, 0.0}}, {'c', {0.0, 1.0, 1.0}},
      {'y', {1.0, 1.0, 0.0}}};
  const std::string kTopicRoadRecognitionTf = "/roadr/tf";
  const std::string kTopicRoadRecognitionLocation = "/roadr/location";
  const std::string KTopicRoadRecognitionLocalMap = "/roadr/local_map";
  const std::string KTopicRoadRecognitionTopoMapRoad = "/roadr/topo_map_road";
  const std::string KTopicRoadRecognitionTopoMapLane = "/roadr/topo_map_lane";
  const std::string KTopicRoadRecognitionElementMap = "/roadr/element_line";
  const std::string KTopicRoadRecognitionLineLable = "/roadr/line_lable";
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
