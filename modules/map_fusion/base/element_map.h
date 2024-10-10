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

#include "modules/map_fusion/base/element_base.h"
#include "modules/map_fusion/base/group.h"

namespace hozon {
namespace mp {
namespace mf {

// TODO(a): 各模块需要的定位信息往这里添加
struct LocInfo {
  DEFINE_PTR(LocInfo)
  DEFINE_CONST_PTR(LocInfo)

  LocInfo Interpolate(const double scale, const LocInfo& end,
                      double timestamp) const {
    LocInfo res;
    res.timestamp = timestamp;
    res.translation =
        this->translation + (end.translation - this->translation) * scale;
    res.quaternion = this->quaternion.slerp(scale, end.quaternion);
    res.pose =
        Eigen::Translation3d(res.translation) * Eigen::Affine3d(res.quaternion);

    if (scale >= 0.5) {
      res.linear_velocity = end.linear_velocity;
      res.angular_velocity = end.angular_velocity;
      res.acceleration = end.acceleration;
      res.gear = end.gear;
    } else {
      res.linear_velocity = this->linear_velocity;
      res.angular_velocity = this->angular_velocity;
      res.acceleration = this->acceleration;
      res.gear = this->gear;
    }
    return res;
  }

 public:
  double timestamp = 0.0;
  Eigen::Vector3d translation;                         // 平移
  Eigen::Quaterniond quaternion;                       // 旋转
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();  // 平移+旋转
  Eigen::Vector3d euler_angle;
  Eigen::Vector3d linear_velocity;   // 线速度
  Eigen::Vector3d angular_velocity;  // 角速度
  Eigen::Vector3d acceleration;      // 加速度
  float heading = 0.0;
  int gear = 100;
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
  // boundaries 改成lane_boundaries
  std::map<Id, Boundary::Ptr> lane_boundaries;
  std::map<Id, CenterLine::Ptr> center_lines;
  std::map<Id, CrossWalk::Ptr> cross_walks;
  std::map<Id, LaneIDSet::Ptr> lanes;
  std::map<Id, RoadEdge::Ptr> road_edges;
  std::map<Id, StopLine::Ptr> stop_lines;
  std::map<Id, Symbol::Ptr> symbols;
  std::map<Id, TrafficLight::Ptr> traffic_lights;
  std::map<Id, OccRoad::Ptr> occ_roads;
  void Clear() {
    arrows.clear();
    boundary_nodes.clear();
    lane_boundaries.clear();
    center_lines.clear();
    cross_walks.clear();
    road_edges.clear();
    stop_lines.clear();
    symbols.clear();
    traffic_lights.clear();
    occ_roads.clear();
  }
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
