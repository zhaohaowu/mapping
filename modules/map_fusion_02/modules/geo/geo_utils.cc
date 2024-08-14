/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： geo_utils.cc
 *   author     ： hozon
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include "modules/map_fusion_02/modules/geo/geo_utils.h"

#include <limits>

#include "modules/map_fusion_02/common/calc_util.h"
#include "modules/map_fusion_02/data_manager/dr_data_manager.h"

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

bool CheckOppositeLineByObj(
    const std::vector<Eigen::Vector3d>& line_points,
    const boost::circular_buffer<std::shared_ptr<Object>>& objects) {
  std::vector<Eigen::Vector3d> obj_points;
  const auto& cur_T_w_v_ = DR_MANAGER->GetCurrentPose();
  for (const auto& object : objects) {
    Eigen::Vector3d p_local(object->position);
    Eigen::Vector3d p_veh = cur_T_w_v_.inverse() * p_local;
    obj_points.emplace_back(p_veh);
  }
  if (obj_points.empty()) {
    return false;
  }
  int line_size = static_cast<int>(line_points.size());
  if (line_size < 2 || line_points.at(0).x() < 0) {
    return false;
  }
  for (auto& obj_point : obj_points) {
    if (obj_point.x() > line_points[line_size - 1].x() ||
        obj_point.x() < line_points[0].x()) {
      continue;
    }
    int num_thresh = 0;
    int num_calculate = 0;
    for (int line_index = 0; line_index < line_size - 1; line_index++) {
      Eigen::Vector3d pt1(line_points[line_index].x(),
                          line_points[line_index].y(),
                          line_points[line_index].z());
      Eigen::Vector3d pt2(line_points[line_index + 1].x(),
                          line_points[line_index + 1].y(),
                          line_points[line_index + 1].z());
      num_calculate++;
      if (math::IsRight(obj_point, pt1, pt2)) {
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

}  // namespace mf
}  // namespace mp
}  // namespace hozon
