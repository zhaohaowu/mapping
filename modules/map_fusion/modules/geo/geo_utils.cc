/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： geo_utils.cc
 *   author     ： hozon
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include "modules/map_fusion/modules/geo/geo_utils.h"

#include <algorithm>
#include <limits>

#include "modules/map_fusion/common/calc_util.h"
#include "modules/map_fusion/data_manager/location_data_manager.h"

namespace hozon {
namespace mp {
namespace mf {
double OccLineFitError(OccRoad::Ptr occ) {
  double avg_error = std::numeric_limits<double>::max();
  const auto& curve_param = occ->curve_params.coefs;
  if (curve_param.size() != 4) {
    return avg_error;
  }
  avg_error = 0.0;
  double c0 = curve_param[0];
  double c1 = curve_param[1];
  double c2 = curve_param[2];
  double c3 = curve_param[3];
  for (const auto& pt : occ->ori_detect_points) {
    double x = pt.x();
    double y = pt.y();
    double error = std::abs(c3 * x * x * x + c2 * x * x + c1 * x + c0 - y);
    avg_error += error;
  }
  return avg_error / (static_cast<int>(occ->ori_detect_points.size()) + 1);
}

bool CheckOppositeLineByObj(const std::vector<Eigen::Vector3f>& line_points,
                            const std::vector<Eigen::Vector3f>& obj_points) {
  int line_size = line_points.size();
  for (auto& obj_point : obj_points) {
    if (obj_point.x() > line_points[line_size - 1].x() ||
        obj_point.x() < line_points[0].x()) {
      continue;
    }
    int num_thresh = 0;
    int num_calculate = 0;
    for (int line_index = 0; line_index < line_size - 1; line_index++) {
      num_calculate++;
      if (math::IsRight(obj_point, line_points[line_index],
                        line_points[line_index + 1])) {
        num_thresh++;
      }
    }
    if (num_thresh >= 1 &&
        (num_calculate != 0 && (static_cast<double>(num_thresh) /
                                static_cast<double>(num_calculate)) > 0.5)) {
      return true;
    }
  }
  return false;
}

void ComputeLaneLineHeading(const std::vector<Eigen::Vector3f>& line_pts,
                            Eigen::Vector3f* avg_heading) {
  if (line_pts.empty() || line_pts.size() < 2 || avg_heading == nullptr) {
    return;
  }
  Eigen::Vector3f sum_heading(0.0, 0.0, 0.0);
  for (int i = 0; i < static_cast<int>(line_pts.size()) - 1; ++i) {
    Eigen::Vector3f point1(line_pts[i].x(), line_pts[i].y(), line_pts[i].z());
    Eigen::Vector3f point2(line_pts[i + 1].x(), line_pts[i + 1].y(),
                           line_pts[i + 1].z());
    Eigen::Vector3f dir_heading = point2 - point1;
    sum_heading += dir_heading;
  }
  *avg_heading = sum_heading / (line_pts.size() - 1);
  return;
}

void ComputeAngleBetweenVectors(const Eigen::Vector3f& v1,
                                const Eigen::Vector3f& v2, float* angle_deg) {
  if (angle_deg == nullptr) {
    return;
  }
  float dot_product = v1.dot(v2);
  float v1_norm = v1.norm();
  float v2_norm = v2.norm();

  if (v1_norm == 0.0 || v2_norm == 0.0) {
    HLOG_WARN << "Error: Zero vector encountered.";
    return;
  }
  float cos_theta = dot_product / (v1_norm * v2_norm);
  cos_theta = std::max(static_cast<float>(1.0),
                       std::min(static_cast<float>(1.0), cos_theta));
  float angle_rad = std::acos(cos_theta);
  *angle_deg = angle_rad * 180.0 / M_PI;
  return;
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
