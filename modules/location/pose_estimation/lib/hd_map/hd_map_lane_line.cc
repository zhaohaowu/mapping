/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： hd_map_lane_line.cc
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/

#include "modules/location/pose_estimation/lib/hd_map/hd_map_lane_line.h"

#include <vector>

namespace hozon {
namespace mp {
namespace loc {

void MapBoundaryLine::Set(const adsfi_proto::internal::SubMap& hd_map,
                          const V3& ref_point) {
  for (const auto& line : hd_map.lines()) {
    auto global_id = line.global_id();
    auto& bl = boundary_line_[global_id];
    bl.id_boundary = global_id;
    bl.line_type = line.linetype();
    for (const auto& point : line.points()) {
      const auto& p = point.wgs84_point();
      Eigen::Vector3d blh(p.latitude(), p.longitude(), p.altitude());
      Eigen::Vector3d enu = hozon::mp::util::Geo::Gcj02ToEnu(blh, ref_point);
      ControlPoint cpoint;
      cpoint.line_type = -1;
      cpoint.point = enu;
      bl.control_point.emplace_back(cpoint);
    }
  }
  this->type_ = HD_MAP_LANE_BOUNDARY_LINE;
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
