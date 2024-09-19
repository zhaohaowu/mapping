/******************************************************************************
 *   Copyright (C) 2024 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： cpt_util.h
 *   author     ： mahaijun
 *   date       ： 2024.05
 ******************************************************************************/

#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cstddef>
#include <iostream>
#include <limits>
#include <utility>
#include <vector>

#include <Sophus/se3.hpp>
#include <Sophus/so3.hpp>

#include "modules/map_fusion_02/common/calc_util.h"
#include "modules/map_fusion_02/modules/lane/road_builder/cut_point.h"
#include "modules/util/include/util/mapping_log.h"

namespace hozon {
namespace mp {
namespace mf {

Point3D ToPoint(const Eigen::Vector3d& p);

// template <typename D>
// Point3D operator*(const Eigen::Matrix<D, 3, 3>& R, const Point3D& q) {
//     Point3D r(0, 0, 0, q.weight);
//     r.x = R(0, 0) * q.x + R(0, 1) * q.y + R(0, 2) * q.z;
//     r.y = R(1, 0) * q.x + R(1, 1) * q.y + R(1, 2) * q.z;
//     r.z = R(2, 0) * q.x + R(2, 1) * q.y + R(2, 2) * q.z;
//     return r;
// }

template <typename D>
Point3D operator*(const Eigen::Matrix<D, 3, 3>& R, const Point3D& q) {
  Point3D r(0, 0, 0, q.weight);
  r.x = R(0, 0) * q.x + R(0, 1) * q.y + R(0, 2) * q.z;
  r.y = R(1, 0) * q.x + R(1, 1) * q.y + R(1, 2) * q.z;
  r.z = R(2, 0) * q.x + R(2, 1) * q.y + R(2, 2) * q.z;
  return r;
}

inline Point3D operator*(const Sophus::SO3d& R, const Point3D& p) {
  Point3D result = R.matrix() * p;
  result.weight = p.weight;
  return result;
}

inline Point3D operator*(const Sophus::SE3d& T, const Point3D& p) {
  Point3D result = T.so3().matrix() * p + ToPoint(T.translation());
  result.weight = p.weight;
  return result;
}

/**
 * @brief Is there an overlap between the two lines
 * @param segment1 points of the line
 * @param segment2 points of the line
 */
bool IsOverLapTwoLine(const std::vector<Point3D>& segment1,
                      const std::vector<Point3D>& segment2);

// This function always return SUCCESS. overlap_status:
// 0: exits lateral overlap between AB, 1: lineA totally front of lineB, 2:
// lineA totally back of lineB
SMStatus DistBetweenTwoLine(const std::vector<Point3D>& points_a,
                            const std::vector<Point3D>& points_b,
                            double* mean_d, double* min_d, int* overlap_status);

SMStatus Point2LineProject3D(const Point3D& pt, const Point3D& line_start_pt,
                             const Point3D& line_end_pt, Point3D* project_pt,
                             float64_t* coef = nullptr);

double FindMinDist(const Point3D& query_point,
                   const std::vector<Point3D>& source_points, size_t start,
                   size_t end);

float64_t InnerProd3d(const Point3D& v1, const Point3D& v2);

SMStatus ComputeLinePointsDist(const std::vector<Point3D>& points_a,
                               const std::vector<Point3D>& points_b,
                               double* mean_d, double* min_d);

// Find the project_point of ref_point in geometric points
// points can be world-coordinate or vehicle-coordiante
// next_idx: next geometric point index in points, indicating the position
// of project_point in points. when project_point is the same as i-th geometric
// point, next_idx is i.
// pos_flag: indicates the relation between ref_point and points.
// 1, ref_point above the points; 0, ref_point projects in the points; -1,
// ref_point bottom the points
SMStatus GetProjectPoint(const std::vector<Point3D>& points,
                         const Point3D& ref_point, Point3D* _project_point,
                         size_t* _next_idx, int* _pos_flag);

SMStatus Point2LineProject2D(const Point3D& pt, const Point3D& line_start_pt,
                             const Point3D& line_end_pt, Point3D* project_pt,
                             float64_t* coef = nullptr);

float64_t InnerProd2d(const Point3D& v1, const Point3D& v2);

// Find the foot of ref_point in target line_points, if the foot is not in the
// range of line_points set, return INVALID
SMStatus GetFootPointInLine(const Point3D& ref_point,
                            const std::vector<Point3D>& points,
                            Point3D* _foot_point);

}  // namespace mf
}  // namespace mp
}  // namespace hozon
