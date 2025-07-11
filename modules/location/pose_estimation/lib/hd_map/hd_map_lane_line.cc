/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： hd_map_lane_line.cc
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/

#include "modules/location/pose_estimation/lib/hd_map/hd_map_lane_line.h"

#include "modules/location/pose_estimation/lib/util/euler.h"
#include "modules/location/pose_estimation/lib/util/globals.h"

namespace hozon {
namespace mp {
namespace loc {

void MapBoundaryLine::Set(
    const std::vector<hozon::hdmap::LaneInfoConstPtr>& lane_ptr_vec,
    const V3& ref_point) {
  boundary_line_.clear();
  HLOG_DEBUG << "lane_ptr_vec.size = " << lane_ptr_vec.size();
  size_t l_count = 0;
  size_t r_count = 0;
  bool is_big_curvature_lane = false;
  for (const auto& lane_ptr : lane_ptr_vec) {
    auto lane = (*lane_ptr).lane();
    if (lane.has_extra_left_boundary()) {
      const auto& extra_left_lane_boundary = lane.extra_left_boundary();
      if (AddMapLine(extra_left_lane_boundary, ref_point)) {
        l_count++;
      }
    } else if (lane.has_left_boundary()) {
      const auto& left_lane_boundary = lane.left_boundary();
      if (AddMapLine(left_lane_boundary, ref_point)) {
        l_count++;
      }
    }
    if (lane.has_extra_right_boundary()) {
      const auto& extra_right_lane_boundary = lane.extra_right_boundary();
      if (AddMapLine(extra_right_lane_boundary, ref_point)) {
        r_count++;
      }
    } else if (lane.has_right_boundary()) {
      const auto& right_lane_boundary = lane.right_boundary();
      if (AddMapLine(right_lane_boundary, ref_point)) {
        r_count++;
      }
    }
  }
  HLOG_DEBUG << "l_count = " << l_count << " , r_count = " << r_count
             << ", boundary_line_ size: " << boundary_line_.size();
  // Print(boundary_line_);
  this->type_ = HD_MAP_LANE_BOUNDARY_LINE;
}

bool MapBoundaryLine::AddMapLine(
    const hozon::hdmap::LaneBoundary& lane_boundary, const V3& ref_point) {
  if (lane_boundary.id().empty()) {
    return false;
  }
  if (lane_boundary.virtual_()) {
    return false;
  }
  if (lane_boundary.boundary_type().empty() ||
      (!lane_boundary.boundary_type().empty() &&
       lane_boundary.boundary_type()[0].types().empty())) {
    HLOG_WARN << "lane boundary type is empty";
    return false;
  }
  if (lane_boundary.boundary_type()[0].types()[0] ==
          hozon::hdmap::LaneBoundaryType::Type::LaneBoundaryType_Type_CURB ||
      lane_boundary.boundary_type()[0].types()[0] ==
          hozon::hdmap::LaneBoundaryType::Type::LaneBoundaryType_Type_BARRIER ||
      lane_boundary.boundary_type()[0].types()[0] ==
          hozon::hdmap::LaneBoundaryType::Type::LaneBoundaryType_Type_UNKNOWN) {
    HLOG_WARN << "it's not lane boundary!!";
    return false;
  }
  auto lane_boundary_id = lane_boundary.id(0).id();
  if (boundary_line_.find(lane_boundary_id) != boundary_line_.end()) {
    return false;
  }
  auto& bl = boundary_line_[lane_boundary_id];
  bl.id_boundary = lane_boundary_id;
  bl.line_type = 0;
  auto lane_boundary_curve = lane_boundary.curve();
  for (const auto& curve_segment : lane_boundary_curve.segment()) {
    auto line_segment = curve_segment.line_segment();
    for (const auto& p : line_segment.original_point()) {
      // 地图是经纬度和ins相反
      Eigen::Vector3d p_gcj(p.y(), p.x(), 0);
      Eigen::Vector3d p_enu = util::Geo::Gcj02ToEnu(p_gcj, ref_point);
      ControlPoint cpoint(0, {0, 0, 0});
      cpoint.point = p_enu;
      bl.control_point.emplace_back(cpoint);
    }
  }
  return true;
}

void MapBoundaryLine::Print(
    const std::unordered_map<std::string, BoundaryLine>& boundarylines) {
  HLOG_DEBUG << "boundarylines.size = " << boundarylines.size();
  for (auto line : boundarylines) {
    auto id = line.first;
    auto cpt = line.second.control_point;
    HLOG_DEBUG << "id = " << id;
    HLOG_DEBUG << "cpt.size = " << cpt.size();
  }
}

void MapBoundaryLine::Crop(const SE3& T_W_V, double front, double width) {
  const SE3 T_V_W = T_W_V.inverse();
  for (auto& line : boundary_line_) {
    auto& one_line = line.second;
    std::vector<ControlPoint> control_points;
    for (const auto& p : one_line.control_point) {
      V3 temp = p.point;
      temp.z() = 0;
      temp = T_V_W * temp;
      bool in_x = (temp.x() > (-front)) && (temp.x() < front);
      bool in_y = (temp.y() > (-width)) && (temp.y() < width);
      if (in_x && in_y) {
        control_points.emplace_back(p);
      }
    }
    line.second.control_point = control_points;
  }
}

}  // namespace loc
}  // namespace mp
}  // namespace hozon
