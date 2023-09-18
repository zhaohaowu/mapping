/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： hd_map.cc
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/

#include "modules/location/pose_estimation/lib/hd_map/hd_map.h"

#include "modules/util/include/util/temp_log.h"

namespace hozon {
namespace mp {
namespace loc {

template <typename Map_Type>
void Map<Map_Type>::set_ref_point(const V3& ref_point) {
  ref_point_ = ref_point;
}
template <typename Map_Type>
V3 Map<Map_Type>::get_ref_point(void) {
  HLOG_INFO << "get_ref_point_ = [" << ref_point_.x() << ", " << ref_point_.y()
            << ", " << ref_point_.z() << "]";
  return ref_point_;
}
template <typename Map_Type>
void Map<Map_Type>::Crop(const SE3& T_W_V, double front, double width) {
  BoxUpdate(T_W_V, front, width);
  for (auto& elment : elment_) {
    switch (elment->type_) {
      case HD_MAP_LANE_BOUNDARY_LINE: {
        auto p = std::static_pointer_cast<MapBoundaryLine>(elment);
        p->Crop(T_W_V, front, width);
        break;
      }
      // case HD_MAP_LANE_CENTER_LINE: {
      //   auto p = std::static_pointer_cast<MapCentorLine>(elment);
      //   p->Crop(T_W_V, front, width);
      //   break;
      // }
      // case HD_MAP_ROAD_EDGE: {
      //   auto p = std::static_pointer_cast<MapRoadEdge>(elment);
      //   p->Crop(T_W_V, front, width);
      //   break;
      // }
      // case HD_MAP_POLE: {
      //   auto p = std::static_pointer_cast<MapPole>(elment);
      //   p->Crop(T_W_V, front, width);
      //   break;
      // }
      // case HD_MAP_TRAFFIC_SIGN: {
      //   auto p = std::static_pointer_cast<MapTrafficSign>(elment);
      //   p->Crop(T_W_V, front, width);
      //   break;
      // }
      default:
        break;
    }
  }
}
template <typename Map_Type>
void Map<Map_Type>::BoxUpdate(const SE3& T_W_V, double front, double width) {
  box_.clear();
  const SE3 T_V_W = T_W_V.inverse();
  V3 a(front, width, 0);
  V3 b(front, -width, 0);
  V3 c(-front, -width, 0);
  V3 d(-front, width, 0);

  box_.emplace_back(T_W_V * a);
  box_.emplace_back(T_W_V * b);
  box_.emplace_back(T_W_V * c);
  box_.emplace_back(T_W_V * d);
  box_.emplace_back(T_W_V * a);
}
template <typename Map_Type>
void Map<Map_Type>::SetMap(const Map_Type& hd_map) {
  MapBoundaryLine::Ptr map_boundary_line = std::make_shared<MapBoundaryLine>();
  map_boundary_line->Set(hd_map, ref_point_);
  elment_.emplace_back(map_boundary_line);
  // MapCentorLine::Ptr map_center_line = std::make_shared<MapCentorLine>();
  // map_center_line->Set(hd_map, ref_point_);
  // elment_.emplace_back(map_center_line);
  // MapRoadEdge::Ptr road_edge = std::make_shared<MapRoadEdge>();
  // road_edge->Set(hd_map, ref_point_);
  // elment_.emplace_back(road_edge);
  // MapPole::Ptr map_pole = std::make_shared<MapPole>();
  // map_pole->Set(hd_map, ref_point_);
  // elment_.emplace_back(map_pole);
  // MapTrafficSign::Ptr map_traffic_sign = std::make_shared<MapTrafficSign>();
  // map_traffic_sign->Set(hd_map, ref_point_);
  // elment_.emplace_back(map_traffic_sign);
}
template <typename Map_Type>
MapElement::Ptr Map<Map_Type>::GetElement(int type) const {
  for (const auto& elment : elment_) {
    if (elment->type_ == type) {
      return elment;
    }
  }
  return nullptr;
}
template <typename Map_Type>
void Map<Map_Type>::Clear(void) {
  box_.clear();
  elment_.clear();
  box_ = std::vector<V3>();
  elment_ = std::vector<MapElement::Ptr>();
}
template class Map<adsfi_proto::internal::SubMap>;

}  // namespace loc
}  // namespace mp
}  // namespace hozon
