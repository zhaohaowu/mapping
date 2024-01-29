/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： hd_map_lane_line.cc
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/

#include "modules/location/pose_estimation/lib/hd_map/hd_map_lane_line.h"

#include "modules/location/pose_estimation/lib/util/euler.h"

namespace hozon {
namespace mp {
namespace loc {

void MapBoundaryLine::Set(const hozon::common::PointENU& position,
                          const Eigen::Matrix3d& rotation,
                          const double& distance, const V3& ref_point) {
  Eigen::Vector3d euler = hozon::mp::loc::RotToEuler312(rotation);
  euler = euler - ((euler.array() > M_PI).cast<double>() * 2.0 * M_PI).matrix();
  double heading = 90.0 - euler.z() / M_PI * 180;
  if (heading < 0.0) {
    heading += 360.0;
  }
  heading = hozon::mp::loc::CalHeading(heading);
  std::vector<hozon::hdmap::LaneInfoConstPtr> lane_ptr_vec;
  const int ret = GLOBAL_HD_MAP->GetLanesWithHeading(
      position, distance, heading, M_PI / 4.0, &lane_ptr_vec);
  if (ret != 0 || lane_ptr_vec.empty()) {
    HLOG_ERROR << "get nearest lane failed";
    return;
  }
  HLOG_INFO << "lane_ptr_vec.size = " << lane_ptr_vec.size();
  size_t l_count = 0, r_count = 0;
  for (const auto& lane_ptr : lane_ptr_vec) {
    auto lane = (*lane_ptr).lane();
    if (lane.has_extra_left_boundary()) {
      auto left_lane_boundary = lane.extra_left_boundary();
      if (left_lane_boundary.id().empty()) {
        continue;
      }
      auto left_lane_boundary_id = left_lane_boundary.id(0).id();
      if (boundary_line_.find(left_lane_boundary_id) != boundary_line_.end()) {
        continue;
      }
      auto& bl = boundary_line_[left_lane_boundary_id];
      bl.id_boundary = left_lane_boundary_id;
      bl.line_type = 0;
      auto left_lane_boundary_curve = left_lane_boundary.curve();
      for (const auto& curve_segment : left_lane_boundary_curve.segment()) {
        auto line_segment = curve_segment.line_segment();
        for (const auto& p : line_segment.original_point()) {
          // 地图是经纬度和ins相反
          Eigen::Vector3d p_gcj(p.y(), p.x(), 0);
          Eigen::Vector3d p_enu = util::Geo::Gcj02ToEnu(p_gcj, ref_point);
          ControlPoint cpoint(0, {0, 0, 0});
          if (bl.line_type == DoubleLineType::DoubleSolidLine ||
              bl.line_type == DoubleLineType::DoubleDashedLine ||
              bl.line_type == DoubleLineType::LeftSolidRightDashed ||
              bl.line_type == DoubleLineType::RightSolidLeftDashed) {
            cpoint.line_type = 1;
          } else {
            cpoint.line_type = 0;
          }
          cpoint.point = p_enu;
          bl.control_point.emplace_back(cpoint);
        }
      }
      l_count++;
    } else if (lane.has_left_boundary()) {
      HLOG_ERROR << "left xu ni xian";
      auto left_lane_boundary = lane.left_boundary();
      if (left_lane_boundary.id().empty()) {
        continue;
      }
      auto left_lane_boundary_id = left_lane_boundary.id(0).id();
      if (boundary_line_.find(left_lane_boundary_id) != boundary_line_.end()) {
        continue;
      }
      auto& bl = boundary_line_[left_lane_boundary_id];
      bl.id_boundary = left_lane_boundary_id;
      bl.line_type = 0;
      auto left_lane_boundary_curve = left_lane_boundary.curve();
      for (const auto& curve_segment : left_lane_boundary_curve.segment()) {
        auto line_segment = curve_segment.line_segment();
        for (const auto& p : line_segment.original_point()) {
          // 地图是经纬度和ins相反
          Eigen::Vector3d p_gcj(p.y(), p.x(), 0);
          Eigen::Vector3d p_enu = util::Geo::Gcj02ToEnu(p_gcj, ref_point);
          ControlPoint cpoint(0, {0, 0, 0});
          if (bl.line_type == DoubleLineType::DoubleSolidLine ||
              bl.line_type == DoubleLineType::DoubleDashedLine ||
              bl.line_type == DoubleLineType::LeftSolidRightDashed ||
              bl.line_type == DoubleLineType::RightSolidLeftDashed) {
            cpoint.line_type = 1;
          } else {
            cpoint.line_type = 0;
          }
          cpoint.point = p_enu;
          bl.control_point.emplace_back(cpoint);
        }
      }
      l_count++;
    }
    if (lane.has_extra_right_boundary()) {
      auto right_lane_boundary = lane.extra_right_boundary();
      if (right_lane_boundary.id().empty()) {
        continue;
      }
      auto right_lane_boundary_id = right_lane_boundary.id(0).id();
      if (boundary_line_.find(right_lane_boundary_id) != boundary_line_.end()) {
        continue;
      }
      auto& br = boundary_line_[right_lane_boundary_id];
      br.id_boundary = right_lane_boundary_id;
      br.line_type = 0;
      auto right_lane_boundary_curve = right_lane_boundary.curve();
      for (const auto& curve_segment : right_lane_boundary_curve.segment()) {
        auto line_segment = curve_segment.line_segment();
        for (const auto& p : line_segment.original_point()) {
          // 地图是经纬度和ins相反
          Eigen::Vector3d p_gcj(p.y(), p.x(), 0);
          Eigen::Vector3d p_enu = util::Geo::Gcj02ToEnu(p_gcj, ref_point);
          ControlPoint cpoint(0, {0, 0, 0});
          if (br.line_type == DoubleLineType::DoubleSolidLine ||
              br.line_type == DoubleLineType::DoubleDashedLine ||
              br.line_type == DoubleLineType::LeftSolidRightDashed ||
              br.line_type == DoubleLineType::RightSolidLeftDashed) {
            cpoint.line_type = 1;
          } else {
            cpoint.line_type = 0;
          }
          cpoint.point = p_enu;
          br.control_point.emplace_back(cpoint);
        }
      }
      r_count++;
    } else if (lane.has_right_boundary()) {
      HLOG_ERROR << "right xu ni xian";
      auto right_lane_boundary = lane.right_boundary();
      if (right_lane_boundary.id().empty()) {
        continue;
      }
      auto right_lane_boundary_id = right_lane_boundary.id(0).id();
      if (boundary_line_.find(right_lane_boundary_id) != boundary_line_.end()) {
        continue;
      }
      auto& br = boundary_line_[right_lane_boundary_id];
      br.id_boundary = right_lane_boundary_id;
      br.line_type = 0;
      auto right_lane_boundary_curve = right_lane_boundary.curve();
      for (const auto& curve_segment : right_lane_boundary_curve.segment()) {
        auto line_segment = curve_segment.line_segment();
        for (const auto& p : line_segment.original_point()) {
          // 地图是经纬度和ins相反
          Eigen::Vector3d p_gcj(p.y(), p.x(), 0);
          Eigen::Vector3d p_enu = util::Geo::Gcj02ToEnu(p_gcj, ref_point);
          ControlPoint cpoint(0, {0, 0, 0});
          if (br.line_type == DoubleLineType::DoubleSolidLine ||
              br.line_type == DoubleLineType::DoubleDashedLine ||
              br.line_type == DoubleLineType::LeftSolidRightDashed ||
              br.line_type == DoubleLineType::RightSolidLeftDashed) {
            cpoint.line_type = 1;
          } else {
            cpoint.line_type = 0;
          }
          cpoint.point = p_enu;
          br.control_point.emplace_back(cpoint);
        }
      }
      r_count++;
    }
  }
  HLOG_INFO << "l_count = " << l_count << " , r_count = " << r_count;
  // Print(boundary_line_);
  this->type_ = HD_MAP_LANE_BOUNDARY_LINE;
}

void MapBoundaryLine::Print(
    const std::unordered_map<std::string, BoundaryLine>& boundarylines) {
  HLOG_ERROR << "boundarylines.size = " << boundarylines.size();
  for (auto line : boundarylines) {
    auto id = line.first;
    auto cpt = line.second.control_point;
    HLOG_ERROR << "id = " << id;
    HLOG_ERROR << "cpt.size = " << cpt.size();
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
