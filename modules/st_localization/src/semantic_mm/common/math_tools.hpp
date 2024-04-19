/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */

#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Sophus/se2.hpp>
#include <Sophus/se3.hpp>

#include "localization/common/log.hpp"
#include "localization/data_type/base.hpp"
#include "localization/data_type/percept_data.hpp"

namespace senseAD {
namespace localization {
namespace smm {

///////////////////////////// data type convert ////////////////////////////////

Eigen::Vector2d PtInner2Eigen(const Point2D_t& p);
Eigen::Vector3d PtInner2Eigen(const Point3D_t& p);

Point2D_t PtEigen2Inner(const Eigen::Vector2d& p);
Point3D_t PtEigen2Inner(const Eigen::Vector3d& p);

Eigen::Quaterniond QuatInner2Eigen(const Quaternion_t& q);
Quaternion_t QuatEigen2Inner(const Eigen::Quaterniond& q);

Eigen::Matrix4d TInner2Eigen(const Quaternion_t& quaternion,
                             const Point3D_t& position);
SE3d TInner2SE3(const Quaternion_t& quaternion, const Point3D_t& position);

///////////////////////////// operator related /////////////////////////////////

// plus one 2d point
Point2D_t operator+(const Point2D_t& a, const Eigen::Vector2d& b);
Eigen::Vector2d operator+(const Eigen::Vector2d& a, const Point2D_t& b);

// plus one 2d point
Point3D_t operator+(const Point3D_t& a, const Eigen::Vector3d& b);
Eigen::Vector3d operator+(const Eigen::Vector3d& a, const Point3D_t& b);

// rotate one 2d point in plane
Point2D_t operator*(const Eigen::Matrix<double, 2, 2>& R, const Point2D_t& q);

// homography convert one 2d point in plane to another plane
Point2D_t operator*(const Eigen::Matrix<double, 3, 3>& H, const Point2D_t& p);

// rotate one 3d point in space
Point3D_t operator*(const Eigen::Matrix<double, 3, 3>& R, const Point3D_t& q);

// transform one 3d point in space
Point3D_t operator*(const Eigen::Matrix<double, 4, 4>& T, const Point3D_t& p);

// transform one 2d point in plane
Point2D_t operator*(const Sophus::SE2<double>& T, const Point2D_t& p);

// transform one 3d point in space
Point3D_t operator*(const SE3d& T, const Point3D_t& p);

/////////////////////////// math/algorithm related /////////////////////////////

// get 2D line main direction by pca
Point2D_t GetLineMainDirection2D(const std::vector<Point3D_t>& points,
                                 int i_th = 1);

// voxel downsample 2D points by covariance
std::vector<int> VoxelDownSample2D(const std::vector<Point3D_t>& points,
                                   const std::vector<Eigen::Matrix2d>& covs,
                                   double sample_dist, double min_x = -100,
                                   double max_x = 100, double min_y = -100,
                                   double max_y = 100);

bool PolyLineLeastSquareFitting(const std::vector<Point3D_t>& points,
                                const std::vector<double>& weights,
                                size_t order, Eigen::VectorXd* line_param,
                                double* residual_mean, double* redisual_max);

bool AdaptedPolyfitLineWithLength(const std::vector<Point3D_t>& points,
                                  const std::vector<double>& weights,
                                  Eigen::Vector4d* line_param);
// triangulate a 3D point given multiple poses and observations in camera
// coordinate (RDF)
bool TriangulatePoint(const std::vector<Point3D_t>& pts,
                      const std::vector<SE3d>& proj_mats, Point3D_t* point_3d);

bool SortPointAndCovFromNearToFar(std::vector<Point3D_t>* points,
                                  std::vector<Eigen::Matrix2d>* covs);

// normalize angle difference in [-pi, pi]
double NormalizeAngleDiff(double diff);

bool BBoxIntersection(const BoundingBox2D& bbox1, const BoundingBox2D& bbox2,
                      BoundingBox2D* roi);

double BoundingBoxIOU(const BoundingBox2D& bbox1, const BoundingBox2D& bbox2);

bool BBoxInterp(const BoundingBox2D& start_bbox, const BoundingBox2D& end_bbox,
                double factor, BoundingBox2D* interp_bbox);

std::pair<int, int> BinarySearchTwoNearestMapLinesId(
    double search_y, const std::vector<Eigen::Vector4d>& map_line_params);

double FourDegreePolyFunc(const Eigen::Vector4d& line_param, double x);

bool CheckTwoPolyFitLineParallel(const Eigen::Vector4d& line1_param,
                                 const Eigen::Vector4d& line2_param);

bool CheckPerceptPointsMapLineParallel(
    const std::vector<Point2DWithCov>& percept_points,
    const Eigen::Vector4d& map_line_param,
    Sophus::SE2<double> transform = Sophus::SE2<double>());

double CalculateDistbtwTwoPolyFitLine(const Eigen::Vector4d& line1_param,
                                      const Eigen::Vector4d& line2_param);

double CalculateDistbtwPerceptPointsMapLine(
    const std::vector<Point2DWithCov>& percept_points,
    const Eigen::Vector4d& map_line_param,
    Sophus::SE2<double> transform = Sophus::SE2<double>());

double CalculateDistbtwTwoPerceptPoints(
    const std::vector<Point2DWithCov>& percept_points1,
    const std::vector<Point2DWithCov>& percept_points2);

bool OptimizeYawByPerceptPointsMapLine(
    const std::vector<std::vector<Point2DWithCov>>& percept_points,
    const std::vector<Eigen::Vector4d>& percept_line_params,
    const std::vector<Eigen::Vector4d>& map_line_params,
    double* optimize_heading);

// project check whether one point in line range
template <typename T>
bool IsPointInLineRange(const T& pt, const T& start_pt, const T& end_pt) {
  T direct = end_pt - start_pt;
  direct = direct / direct.Norm();
  double proj1 = (pt - start_pt).Dot(direct);
  double proj2 = (pt - end_pt).Dot(direct);
  return proj1 >= 0 && proj2 <= 0;
}

template <typename T>
float64_t Point2PointManhattanDist2D(const T& p1, const T& p2,
                                     const Eigen::Matrix2d& info) {
  Eigen::Vector2d p1_2d(p1.x, p1.y);
  Eigen::Vector2d p2_2d(p2.x, p2.y);
  Eigen::Vector2d p1_p2 = p1_2d - p2_2d;
  return std::sqrt(p1_p2.transpose() * info * p1_p2);
}

// point to line distance
template <typename T>
float64_t Point2LineDist(const T& p, const T& q1, const T q2,
                         bool signed_dist = false) {
  T line_q1q2 = q2 - q1;
  T line_q1p = p - q1;
  line_q1q2 = line_q1q2 / line_q1q2.Norm();
  float64_t project_dist = line_q1p.Dot(line_q1q2);
  if (line_q1p.Norm() <= project_dist) return 0.0;

  double sign = 1.0;
  if (signed_dist) {
    if (line_q1q2.Cross(line_q1p) < 0) sign = -1;
  }
  return sign * std::sqrt(line_q1p.SquaredNorm() - project_dist * project_dist);
}
// point to line 2D(X-Y) distance
template <typename T>
float64_t Point2LineDist2D(const T& p, const T& q1, const T q2,
                           bool signed_dist = false) {
  Point2D_t p_2d(p.x, p.y);
  Point2D_t q1_2d(q1.x, q1.y);
  Point2D_t q2_2d(q2.x, q2.y);
  return Point2LineDist(p_2d, q1_2d, q2_2d, signed_dist);
}
// point to line 2D(X-Y) manhattan distance
template <typename T>
float64_t Point2LineMahDist2D(const T& p, const T& q1, const T q2,
                              const Eigen::Matrix2d& info) {
  Point2D_t line_q1q2 = Point2D_t(q2.x, q2.y) - Point2D_t(q1.x, q1.y);
  Point2D_t line_q1p = Point2D_t(p.x, p.y) - Point2D_t(q1.x, q1.y);
  line_q1q2 = line_q1q2 / line_q1q2.Norm();
  float64_t project_dist = line_q1p.Dot(line_q1q2);
  Point2D_t line_q1proj = line_q1q2 * project_dist;
  Point2D_t line_pproj = line_q1proj - line_q1p;
  Eigen::Vector2d vec_pproj(line_pproj.x, line_pproj.y);
  return std::sqrt(vec_pproj.transpose() * info * vec_pproj);
}
// point to line segment distance
template <typename T>
float64_t Point2LineSegmentDist2D(const T& p, const T& q1, const T q2,
                                  bool signed_dist = false) {
  if (IsPointInLineRange(p, q1, q2)) {
    return Point2LineDist2D(p, q1, q2, signed_dist);
  } else {
    Point2D_t line_q1q2 = Point2D_t(q2.x, q2.y) - Point2D_t(q1.x, q1.y);
    Point2D_t line_q1p = Point2D_t(p.x, p.y) - Point2D_t(q1.x, q1.y);
    Point2D_t line_q2p = Point2D_t(p.x, p.y) - Point2D_t(q2.x, q2.y);
    float64_t dis_q1p = line_q1p.Norm();
    float64_t dis_q2p = line_q2p.Norm();
    if (dis_q1p < dis_q2p) {
      double sign = 1.0;
      if (signed_dist) {
        if (line_q1q2.Cross(line_q1p) < 0) sign = -1;
      }
      return sign * dis_q1p;
    } else {
      double sign = 1.0;
      if (signed_dist) {
        if (line_q1q2.Cross(line_q2p) < 0) sign = -1;
      }
      return sign * dis_q2p;
    }
  }
}

// transform points
template <typename T1, typename T2>
std::vector<T1> TransformPoints(const std::vector<T1>& points, const T2& pose) {
  std::vector<T1> trans_pts;
  trans_pts.reserve(points.size());
  for (const auto& pt : points) trans_pts.emplace_back(pose * pt);
  return trans_pts;
}

// binary search two nearest points in one line
template <typename T>
bool BinarySearchTwoNearestPoints(const T& search_pt,
                                  const std::vector<T>& points, size_t* index1,
                                  size_t* index2) {
  if (points.size() < 2 || !index1 || !index2) return false;

  T direct = points.back() - points.front();
  direct = direct / direct.Norm();

  double pt_proj = (search_pt - points.front()).Dot(direct);

  int left = 0, right = static_cast<int>(points.size() - 1);
  while (right - left > 1) {
    int mid = left + (right - left) / 2;
    double proj = (points[mid] - points.front()).Dot(direct);
    if (proj < pt_proj) {
      left = mid;
    } else {
      right = mid;
    }
  }
  *index1 = left;
  *index2 = right;
  return true;
}

// point to line 2d(X-Y) distance
template <typename T>
float64_t Point2LineDistance2D(const T& pt, const std::vector<T>& tgt_line,
                               bool* valid, bool signed_dist = false) {
  if (tgt_line.empty() || !valid) return std::numeric_limits<double>::max();

  // check whether in target line view range
  if (!IsPointInLineRange(pt, tgt_line.front(), tgt_line.back())) {
    *valid = false;
    return std::numeric_limits<double>::max();
  }

  // serach the nearest two points in target line
  size_t idx1, idx2;
  if (!BinarySearchTwoNearestPoints(pt, tgt_line, &idx1, &idx2)) {
    *valid = false;
    return std::numeric_limits<double>::max();
  }

  *valid = true;
  double distance =
      Point2LineDist2D(pt, tgt_line[idx1], tgt_line[idx2], signed_dist);
  return distance;
}

// line to line 2d(X-Y) distance
template <typename T>
float64_t Line2LineDistance2D(const std::vector<T>& src_line,
                              const std::vector<T>& tgt_line,
                              int* valid_num = nullptr,
                              bool signed_dist = false) {
  if (src_line.empty() || tgt_line.empty()) {
    if (valid_num) *valid_num = 0;
    return std::numeric_limits<double>::max();
  }

  int counter = 0;
  double ave_distance = 0.0;
  for (const auto& pt : src_line) {
    // check whether in target line view range
    if (!IsPointInLineRange(pt, tgt_line.front(), tgt_line.back())) {
      continue;
    }
    // serach the nearest two points in target line
    size_t idx1, idx2;
    if (!BinarySearchTwoNearestPoints(pt, tgt_line, &idx1, &idx2)) {
      continue;
    }
    ave_distance +=
        Point2LineDist2D(pt, tgt_line[idx1], tgt_line[idx2], signed_dist);
    ++counter;
  }
  if (valid_num) *valid_num = counter;
  ave_distance = counter == 0 ? std::numeric_limits<double>::max()
                              : ave_distance / counter;
  return ave_distance;
}

// line to line manhattan 2d(X-Y) distance
template <typename T>
float64_t Line2LineManhattanDistance2D(
    const std::vector<T>& src_line,
    const std::vector<Eigen::Matrix2d>& src_line_cov,
    const std::vector<T>& tgt_line, int* valid_num = nullptr) {
  if (src_line.empty() || tgt_line.empty()) {
    if (valid_num) *valid_num = 0;
    return std::numeric_limits<double>::max();
  }

  int counter = 0;
  double ave_distance = 0.0;
  for (size_t i = 0; i < src_line.size(); ++i) {
    const auto& pt = src_line[i];
    Eigen::Matrix2d pt_info = src_line_cov[i].inverse();
    // check whether in target line view range
    if (!IsPointInLineRange(pt, tgt_line.front(), tgt_line.back())) {
      continue;
    }
    // serach the nearest two points in target line
    size_t idx1, idx2;
    if (!BinarySearchTwoNearestPoints(pt, tgt_line, &idx1, &idx2)) {
      continue;
    }
    ave_distance +=
        Point2LineMahDist2D(pt, tgt_line[idx1], tgt_line[idx2], pt_info);
    ++counter;
  }
  if (valid_num) *valid_num = counter;
  ave_distance = counter == 0 ? std::numeric_limits<double>::max()
                              : ave_distance / counter;
  return ave_distance;
}

// upsample points
template <typename T>
bool UpSamplePoints(const std::vector<T>& raw_pts, double step,
                    std::vector<T>* out_pts) {
  if (!out_pts) return false;
  if (raw_pts.empty()) return false;

  const float EPSINON = 0.00001;
  int sample_pt_num =
      static_cast<int>((raw_pts.back() - raw_pts.front()).Norm() / step + 0.5);
  out_pts->reserve(sample_pt_num);
  for (size_t i = 0; i < raw_pts.size() - 1; ++i) {
    out_pts->emplace_back(raw_pts[i]);
    T start_point = raw_pts[i];
    T end_point = raw_pts[i + 1];
    float64_t len = (end_point - start_point).Norm();
    int insert_cnt = static_cast<int>(len / step);
    if (std::fabs(len - insert_cnt) < EPSINON) insert_cnt -= 1;

    for (int k = 1; k <= insert_cnt; ++k) {
      float64_t coef_ = static_cast<float64_t>(k) / (insert_cnt + 1.0);
      T new_point = (end_point - start_point) * coef_ + start_point;
      out_pts->emplace_back(new_point);
    }
  }
  out_pts->emplace_back(raw_pts.back());
  return true;
}

// RamerDouglasPeucker downsample points
template <typename T>
bool DownSamplePointsRDPMethod(const std::vector<T>& raw_pts, double epsilon,
                               std::vector<T>* out_pts) {
  if (!out_pts) return false;
  if (raw_pts.size() < 2) return false;

  // Find point with the maximum distance from line between start and end
  double dist_max = 0.0;
  size_t index = 0, end = raw_pts.size() - 1;
  for (size_t i = 1; i < end; i++) {
    float64_t dist = Point2LineDist(raw_pts[i], raw_pts[0], raw_pts[end]);
    if (dist > dist_max) {
      index = i;
      dist_max = dist;
    }
  }
  // If max distance is greater than epsilon, recursively simplify
  if (dist_max > epsilon) {
    // Recursive call
    std::vector<T> res1;
    std::vector<T> res2;
    std::vector<T> first_line(raw_pts.begin(), raw_pts.begin() + index + 1);
    std::vector<T> last_line(raw_pts.begin() + index, raw_pts.end());
    DownSamplePointsRDPMethod(first_line, epsilon, &res1);
    DownSamplePointsRDPMethod(last_line, epsilon, &res2);
    // Build the result list
    out_pts->assign(res1.begin(), res1.end() - 1);
    out_pts->insert(out_pts->end(), res2.begin(), res2.end());
    if (out_pts->size() < 2)
      throw std::runtime_error("Problem assembling output");
  } else {
    // Just return start and end points
    out_pts->clear();
    out_pts->push_back(raw_pts[0]);
    out_pts->push_back(raw_pts[end]);
  }
  return true;
}

// sample points from a line param by distance
template <typename T>
bool SampleLineByDistance(const Point3D_t& start_pt, const Point3D_t& end_pt,
                          const T& poly_coef, const double interval,
                          std::vector<Point3D_t>* sample_pts) {
  if (!sample_pts) return false;
  Point3D_t interp_vec = end_pt - start_pt;
  for (auto x = start_pt.x; x < end_pt.x - interval / 2; x += interval) {
    double y = poly_coef[3] + poly_coef[2] * x + poly_coef[1] * x * x +
               poly_coef[0] * x * x * x;
    double z = (x - start_pt.x) / interp_vec.x * interp_vec.z + start_pt.z;
    sample_pts->emplace_back(x, y, z);
  }

  // insert end point
  sample_pts->emplace_back(end_pt);
  return true;
}

// Get more than 1 order curve function(y = a*x^{order} + ...
template <typename T,
          std::enable_if_t<std::is_same<T, Point2D_t>::value ||
                           std::is_same<T, Point3D_t>::value>* = nullptr>
bool PolyLineFit(const std::vector<T>& points, const size_t order,
                 Eigen::VectorXd* line) {
  if (line == nullptr || points.size() < 2) {
    LC_LERROR(PolyLineFit) << "Wrong parameter with points: " << points.size();
    return false;
  }
  if (order > points.size() - 1) {
    LC_LERROR(PolyLineFit) << "points number-" << points.size()
                           << " is too less to fit " << order << "-order line";
    return false;
  }

  Eigen::MatrixXd A(points.size(), order + 1);
  for (size_t i = 0; i < points.size(); ++i) {
    A(i, order) = 1.0;
  }

  Eigen::VectorXd b(points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    for (int j = order - 1; j >= 0; --j) {
      A(i, j) = A(i, j + 1) * points[i].x;
    }
    b(i) = points[i].y;
  }
  *line = A.householderQr().solve(b);
  return true;
}

template <typename T>
bool CheckLineSegmentCircleIntersection(double radius, const T& center,
                                        const T& start, const T& end) {
  double dist_to_line = Point2LineDist2D(center, start, end);
  if (dist_to_line > radius) return false;
  double dist_to_start = (center - start).Norm2D();
  double dist_to_end = (center - end).Norm2D();
  bool in_range = IsPointInLineRange(center, start, end);
  if (!in_range && dist_to_start > radius && dist_to_end > radius) return false;
  return true;
}

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
