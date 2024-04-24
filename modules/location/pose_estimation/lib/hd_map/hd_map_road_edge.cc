/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： hd_map_road_edge.cc
 *   author     ： ouyanghailin
 *   date       ： 2024.04
 ******************************************************************************/

#include "modules/location/pose_estimation/lib/hd_map/hd_map_road_edge.h"
#include "modules/location/pose_estimation/lib/util/euler.h"

#include "modules/location/pose_estimation/lib/util/globals.h"
#include "common/utm_projection/coordinate_convertor.h"

namespace hozon {
namespace mp {
namespace loc {

void MapRoadEdge::Set(const hozon::common::PointENU& position,
                          const Eigen::Matrix3d& rotation,
                          const double& distance, const V3& ref_point) {
  edge_line_.clear();
  static std::string map_road_edge = "00000";
  Eigen::Vector3d euler = hozon::mp::loc::RotToEuler312(rotation);
  euler = euler - ((euler.array() > M_PI).cast<double>() * 2.0 * M_PI).matrix();
  double heading = 90.0 - euler.z() / M_PI * 180;
  if (heading < 0.0) {
    heading += 360.0;
  }
  heading = hozon::mp::loc::CalHeading(heading);
  std::vector<hozon::hdmap::RoadRoiPtr> road_ptr_vec;
  std::vector<hozon::hdmap::JunctionInfoConstPtr> junction_ptr_vec;
  const int ret = GLOBAL_HD_MAP->GetRoadBoundaries(position, distance, &road_ptr_vec, &junction_ptr_vec);
  if (ret != 0 || road_ptr_vec.empty()) {
    HLOG_ERROR << "get nearest road_edge failed";
    return;
  }
  HLOG_ERROR << "road_ptr_vec.size = " << road_ptr_vec.size();
  size_t l_count = 0, r_count = 0;
  for (const auto& road_ptr : road_ptr_vec) {
    auto left_road_edge = road_ptr->left_boundary;
    auto left_road_edge_id = map_road_edge;
    if (AddMapRoadEdge(left_road_edge, ref_point, left_road_edge_id)) {
        map_road_edge = std::to_string(std::stoi(map_road_edge) + 1);
        l_count++;
    }
    auto right_road_edge = road_ptr->right_boundary;
    auto right_road_edge_id = map_road_edge;
    if (AddMapRoadEdge(right_road_edge, ref_point, right_road_edge_id)) {
        map_road_edge = std::to_string(std::stoi(map_road_edge) + 1);
        r_count++;
    }
  }
  HLOG_ERROR << "l_count = " << l_count << " , r_count = " << r_count
             << ", edge_line_ size: " << edge_line_.size();
  // Print(edge_line_);
  this->type_ = HD_MAP_ROAD_EDGE;
}

bool MapRoadEdge::AddMapRoadEdge(
    const hozon::hdmap::LineBoundary& road_edge, const V3& ref_point, const std::string road_edge_id) {
  if (road_edge.line_points.empty()) {
    return false;
  }
  const auto el_points = road_edge.line_points;
//   const auto road_edge_id = road_edge_id;
  if (edge_line_.find(road_edge_id) != edge_line_.end()) {
    return false;
  }
  auto& el = edge_line_[road_edge_id];
  el.id_edge = road_edge_id;
  el.edge_type = 0;
  for (auto& el_point : el_points) {  // utm frame point
    auto utm_x = el_point.x();
    auto utm_y = el_point.y();
    hozon::common::coordinate_convertor::UTM2GCS(51, &utm_x, &utm_y);
    Eigen::Vector3d el_point_gcj(utm_y, utm_x, 0);
    Eigen::Vector3d el_point_enu = util::Geo::Gcj02ToEnu(el_point_gcj, ref_point);
    ControlPoint cpoint(0, {0, 0, 0});
    cpoint.point = el_point_enu;
    el.control_point.emplace_back(cpoint);
  }
  return true;
}

// void MapBoundaryLine::Print(
//     const std::unordered_map<std::string, BoundaryLine>& boundarylines) {
//   HLOG_DEBUG << "boundarylines.size = " << boundarylines.size();
//   for (auto line : boundarylines) {
//     auto id = line.first;
//     auto cpt = line.second.control_point;
//     HLOG_DEBUG << "id = " << id;
//     HLOG_DEBUG << "cpt.size = " << cpt.size();
//   }
// }

// void MapBoundaryLine::Crop(const SE3& T_W_V, double front, double width) {
//   const SE3 T_V_W = T_W_V.inverse();
//   for (auto& line : edge_line_) {
//     auto& one_line = line.second;
//     std::vector<ControlPoint> control_points;
//     for (const auto& p : one_line.control_point) {
//       V3 temp = p.point;
//       temp.z() = 0;
//       temp = T_V_W * temp;
//       bool in_x = (temp.x() > (-front)) && (temp.x() < front);
//       bool in_y = (temp.y() > (-width)) && (temp.y() < width);
//       if (in_x && in_y) {
//         control_points.emplace_back(p);
//       }
//     }
//     line.second.control_point = control_points;
//   }
// }

}  // namespace loc
}  // namespace mp
}  // namespace hozon
