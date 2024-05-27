/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： hd_map_road_edge.cc
 *   author     ： luoning
 *   date       ： 2024.04
 ******************************************************************************/
#include "modules/location/pose_estimation/lib/hd_map/hd_map_road_edge.h"

namespace hozon {
namespace mp {
namespace loc {
void MapRoadEdge::Set(const std::vector<hozon::hdmap::LaneInfoConstPtr>& lane_ptr_vec,
                      const V3& ref_point) {
  edge_line_.clear();
  std::unordered_set<std::string> road_ids;
  std::unordered_set<std::string> sec_ids;
  for (const auto& lane_ptr : lane_ptr_vec) {
    if (!lane_ptr) {
      continue;
    }
    road_ids.insert(lane_ptr->road_id().id());
    sec_ids.insert(lane_ptr->section_id().id());
  }
  for (const auto& road_id : road_ids) {
    hozon::hdmap::Id road_ptr;
    road_ptr.set_id(road_id);
    auto road = GLOBAL_HD_MAP->GetRoadById(road_ptr)->road();
    for (const auto& section : road.section()) {
      if (sec_ids.find(section.id().id()) == sec_ids.end()) {
        continue;
      }
      int road_edge_count = 0;
      for (const auto& line :
           section.boundary().outer_polygon().edge()) {
        if (line.type() == hdmap::BoundaryEdge::UNKNOWN) {
          continue;
        }
        if (line.type() == hdmap::BoundaryEdge::NORMAL) {
          continue;
        }
        if (line.is_plane()) {
          continue;
        }
        auto edge_curve = line.curve();
        auto edge_curve_id = line.id().id();
        if (edge_line_.find(edge_curve_id) != edge_line_.end()) {
          continue;
        }
        auto& el = edge_line_[edge_curve_id];
        el.id_edge = edge_curve_id;
        const int zone = 51;
        for (const auto& curve_segment : edge_curve.segment()) {
          auto edge_line_segment = curve_segment.line_segment();
          for (const auto& point : edge_line_segment.point()) {
            double x = point.x();
            double y = point.y();
            auto ret =
                hozon::common::coordinate_convertor::UTM2GCS(zone, &x, &y);
            if (!ret) {
              HLOG_ERROR << "utm2gcs failed";
              continue;
            }
            Eigen::Vector3d p_gcj(y, x, 0);
            Eigen::Vector3d p_enu = util::Geo::Gcj02ToEnu(p_gcj, ref_point);
            ControlPoint cpoint(0, {0, 0, 0});
            cpoint.line_type = -1;
            cpoint.point = p_enu;
            el.control_point.emplace_back(cpoint);
          }
        }
        road_edge_count = road_edge_count + 1;
      }
    }
  }
  this->type_ = HD_MAP_ROAD_EDGE;
}

void MapRoadEdge::Crop(const SE3& T_W_V, double front, double width) {
  const SE3 T_V_W = T_W_V.inverse();
  for (auto& line : edge_line_) {
    auto& one_line = line.second;
    // 裁剪地图
    std::vector<ControlPoint> control_points;
    for (const auto& p : one_line.control_point) {
      V3 temp = p.point;
      temp.z() = 0;
      temp = T_V_W * temp;
      // in corp
      bool in_x = (temp.x() > (-front)) && (temp.x() < front);
      bool in_y = (temp.y() > (-width)) && (temp.y() < width);

      if (in_x && in_y) {
        control_points.emplace_back(p);
      }
    }
    // 交换数据
    line.second.control_point = control_points;
  }
}
}  // namespace loc
}  // namespace mp
}  // namespace hozon
