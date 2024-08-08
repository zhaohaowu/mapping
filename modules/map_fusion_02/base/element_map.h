/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： element_map.h
 *   author     ： chenlongxi
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <map>
#include <string>

#include "modules/map_fusion_02/base/element_base.h"
#include "modules/map_fusion_02/base/group.h"

namespace hozon {
namespace mp {
namespace mf {

// TODO(a): 各模块需要的定位信息往这里添加
struct LocInfo {
  double timestamp;
  Eigen::Vector3d position;
  Eigen::Quaterniond quaternion;
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  Eigen::Vector3d euler_angle;
  Eigen::Vector3d linear_vrf;
  Eigen::Vector3d angular_vrf;
  float heading;

  DEFINE_PTR(LocInfo)
  DEFINE_CONST_PTR(LocInfo)
};

struct MapInfo {
  std::string frame_id;
  double stamp = 0.;
  Eigen::Vector3d local_enu_center;
  int utm_zone = 0;
};

// 数据传入后首先转成ElementMap供下游使用
struct ElementMap {
  MapInfo map_info;

  std::map<Id, Arrow::Ptr> arrows;
  std::map<Id, BoundaryNode::Ptr> boundary_nodes;
  std::map<Id, Boundary::Ptr> boundaries;
  std::map<Id, CenterLine::Ptr> center_lines;
  std::map<Id, CrossWalk::Ptr> cross_walks;
  std::map<Id, Lane::Ptr> lanes;
  std::map<Id, Road::Ptr> roads;
  std::map<Id, StopLine::Ptr> stop_lines;
  std::map<Id, Symbol::Ptr> symbols;
  std::map<Id, TrafficLight::Ptr> traffic_lights;
  std::map<Id, OccRoad::Ptr> occ_roads;

  DEFINE_PTR(ElementMap)
  DEFINE_CONST_PTR(ElementMap)
};

// struct ElementMapOut {
//   MapInfo map_info;
//
//   std::map<Id, Arrow::Ptr> arrows;
//   std::map<Id, BoundaryDetailed::Ptr> boundaries;
//   std::map<Id, CenterLineDetailed::Ptr> center_lines;
//   std::map<Id, CrossWalk::Ptr> cross_walks;
//   std::map<Id, Lane::Ptr> lanes;
//   std::map<Id, Road::Ptr> roads;
//   std::map<Id, StopLine::Ptr> stop_lines;
//   std::map<Id, Symbol::Ptr> symbols;
//   std::map<Id, TrafficLight::Ptr> traffic_lights;
//   std::map<Id, OccRoad::Ptr> occ_roads;
//   std::map<Id, Obj::Ptr> objs;
//
//   DEFINE_PTR(ElementMapOut)
//
//   static Ptr Create() { return std::make_shared<ElementMapOut>(); }
// };

}  // namespace mf
}  // namespace mp
}  // namespace hozon
