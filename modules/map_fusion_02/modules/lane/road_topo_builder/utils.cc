/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： utils.cc
 *   author     ： likuan
 *   date       ： 2024.08
 ******************************************************************************/
#include "modules/map_fusion_02/modules/lane/road_topo_builder/utils.h"

namespace hozon {
namespace mp {
namespace mf {

float TopoUtils::LaneDist(const Lane::Ptr& lane_in_curr,
                          const Lane::Ptr& lane_in_next) {
  size_t sizet = lane_in_curr->center_line_pts.size();
  Eigen::Vector3f lane_in_next_norm(0.0, 0.0, 0.0);
  size_t next_lane_cp_idx = 1;
  while (next_lane_cp_idx < lane_in_next->center_line_pts.size() &&
         lane_in_next_norm.norm() < 2.0) {
    lane_in_next_norm = lane_in_next->center_line_pts[next_lane_cp_idx].pt -
                        lane_in_next->center_line_pts[0].pt;
    next_lane_cp_idx++;
  }
  if (lane_in_next_norm.norm() > 2.0) {
    lane_in_next_norm = lane_in_next_norm.normalized();
    Eigen::Vector3f lane_curr_next_vec =
        lane_in_curr->center_line_pts[sizet - 1].pt -
        lane_in_next->center_line_pts[0].pt;
    float dis = (lane_curr_next_vec.cross(lane_in_next_norm)).norm();
    return dis;
  }
  return 10.0;
}

float TopoUtils::CalculateDistPt(const Lane::Ptr& lane_in_next,
                                 const Lane::Ptr& lane_in_curr, size_t sizet) {
  return pow(lane_in_next->center_line_pts[0].pt.y() -
                 lane_in_curr->center_line_pts[sizet - 1].pt.y(),
             2) +
         pow(lane_in_next->center_line_pts[0].pt.x() -
                 lane_in_curr->center_line_pts[sizet - 1].pt.x(),
             2);
}

float TopoUtils::CalculatePoint2CenterLine(const Lane::Ptr& lane_in_next,
                                           const Lane::Ptr& lane_in_curr) {
  return abs(lane_in_next->center_line_pts[0].pt.y() -
             (lane_in_curr->center_line_param[0] +
              lane_in_curr->center_line_param[1] *
                  lane_in_next->center_line_pts[0].pt.x())) /
         sqrt(1 + pow(lane_in_curr->center_line_param[1], 2));
}

float TopoUtils::Calculate2CenterlineAngle(const Lane::Ptr& lane_in_next,
                                           const Lane::Ptr& lane_in_curr,
                                           size_t sizet) {
  return atan((lane_in_next->center_line_pts[0].pt.y() -
               lane_in_curr->center_line_pts[sizet - 1].pt.y()) /
              (lane_in_next->center_line_pts[0].pt.x() -
               lane_in_curr->center_line_pts[sizet - 1].pt.x())) -
         atan(lane_in_curr->center_line_param[1]);
}

double TopoUtils::CalcLaneLength(const Lane::Ptr& lane) {
  if (lane == nullptr || lane->center_line_pts.size() < 2) {
    return 0;
  }
  double len = 0;
  for (int i = 0; i < static_cast<int>(lane->center_line_pts.size()) - 1; ++i) {
    Eigen::Vector3f p0(lane->center_line_pts.at(i).pt.x(),
                       lane->center_line_pts.at(i).pt.y(), 0);
    Eigen::Vector3f p1(lane->center_line_pts.at(i + 1).pt.x(),
                       lane->center_line_pts.at(i + 1).pt.y(), 0);
    len += Dist(p0, p1);
  }
  return len;
}

float TopoUtils::PointToLineDis(const LineSegment& line, float line_front_x,
                                float line_front_y) {
  if (line.pts.empty()) {
    return 0.0;
  }
  if (line.pts.size() == 1) {
    return std::hypot(line.pts[0].pt.x() - line_front_x,
                      line.pts[0].pt.y() - line_front_y);
  }
  int index_left = 0;
  if (index_left < static_cast<int>(line.pts.size()) - 1) {
    int index_right = index_left + 1;
    while (index_right < static_cast<int>(line.pts.size()) &&
           line.pts[index_right].pt.x() - line.pts[index_left].pt.x() < 0.1) {
      index_right++;
    }
    if (index_right < static_cast<int>(line.pts.size())) {
      const auto& p_left = line.pts[index_left].pt;
      const auto& p_right = line.pts[index_right].pt;
      float k = (p_right.y() - p_left.y()) / (p_right.x() - p_left.x());
      float b = p_right.y() - k * p_right.x();
      return abs(line_front_x * k + b - line_front_y) / sqrt(1 + k * k);
    }
  }
  return 0.0;
}

void TopoUtils::FitCenterLine(Lane::Ptr lane) {
  std::vector<hozon::common::math::Vec2d> left_pts;
  std::vector<hozon::common::math::Vec2d> right_pts;
  left_pts.reserve(lane->left_boundary->pts.size());
  right_pts.reserve(lane->right_boundary->pts.size());
  for (const auto& pt : lane->left_boundary->pts) {
    left_pts.emplace_back(pt.pt.x(), pt.pt.y());
  }
  for (const auto& pt : lane->right_boundary->pts) {
    right_pts.emplace_back(pt.pt.x(), pt.pt.y());
  }
  std::vector<hozon::common::math::Vec2d> cent_pts;
  math::GenerateCenterPoint(left_pts, right_pts, &cent_pts);
  for (const auto& pt : cent_pts) {
    Point cpt(PointType::RAW, static_cast<float>(pt.x()),
              static_cast<float>(pt.y()), 0);
    lane->center_line_pts.emplace_back(cpt);
  }
  if (lane->center_line_pts.size() > 3) {
    lane->center_line_param = math::FitLaneline(lane->center_line_pts);
    lane->center_line_param_front =
        math::FitLanelinefront(lane->center_line_pts);
  }
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
