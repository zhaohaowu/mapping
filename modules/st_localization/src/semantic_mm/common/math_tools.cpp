
/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */

#include "semantic_mm/common/math_tools.hpp"

#include "semantic_mm/common/voxel_index.hpp"

namespace senseAD {
namespace localization {
namespace smm {

Eigen::Vector2d PtInner2Eigen(const Point2D_t& p) {
  return Eigen::Vector2d(p.x, p.y);
}

Eigen::Vector3d PtInner2Eigen(const Point3D_t& p) {
  return Eigen::Vector3d(p.x, p.y, p.z);
}

Point2D_t PtEigen2Inner(const Eigen::Vector2d& p) {
  return Point2D_t(p.x(), p.y());
}

Point3D_t PtEigen2Inner(const Eigen::Vector3d& p) {
  return Point3D_t(p.x(), p.y(), p.z());
}

Eigen::Quaterniond QuatInner2Eigen(const Quaternion_t& q) {
  return {q.qw, q.qx, q.qy, q.qz};
}

Quaternion_t QuatEigen2Inner(const Eigen::Quaterniond& q) {
  return {q.w(), q.x(), q.y(), q.z()};
}

Eigen::Matrix4d TInner2Eigen(const Quaternion_t& quaternion,
                             const Point3D_t& position) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = QuatInner2Eigen(quaternion).matrix();
  T.block<3, 1>(0, 3) = PtInner2Eigen(position);
  return T;
}

SE3d TInner2SE3(const Quaternion_t& quaternion, const Point3D_t& position) {
  return {QuatInner2Eigen(quaternion), PtInner2Eigen(position)};
}

Point2D_t operator+(const Point2D_t& a, const Eigen::Vector2d& b) {
  return Point2D_t(a.x + b.x(), a.y + b.y());
}

Eigen::Vector2d operator+(const Eigen::Vector2d& a, const Point2D_t& b) {
  return Eigen::Vector2d(a.x() + b.x, a.y() + b.y);
}

Point3D_t operator+(const Point3D_t& a, const Eigen::Vector3d& b) {
  return Point3D_t(a.x + b.x(), a.y + b.y(), a.z + b.z());
}

Eigen::Vector3d operator+(const Eigen::Vector3d& a, const Point3D_t& b) {
  return Eigen::Vector3d(a.x() + b.x, a.y() + b.y, a.z() + b.z);
}

Point2D_t operator*(const Eigen::Matrix<double, 2, 2>& R, const Point2D_t& q) {
  Point2D_t r;
  r.x = R(0, 0) * q.x + R(0, 1) * q.y;
  r.y = R(1, 0) * q.x + R(1, 1) * q.y;
  return r;
}

Point2D_t operator*(const Eigen::Matrix<double, 3, 3>& H, const Point2D_t& p) {
  double x = H(0, 0) * p.x + H(0, 1) * p.y + H(0, 2);
  double y = H(1, 0) * p.x + H(1, 1) * p.y + H(1, 2);
  double z = H(2, 0) * p.x + H(2, 1) * p.y + H(2, 2);
  return Point2D_t(x / z, y / z);
}

Point3D_t operator*(const Eigen::Matrix<double, 3, 3>& R, const Point3D_t& q) {
  Point3D_t r(0, 0, 0);
  r.x = R(0, 0) * q.x + R(0, 1) * q.y + R(0, 2) * q.z;
  r.y = R(1, 0) * q.x + R(1, 1) * q.y + R(1, 2) * q.z;
  r.z = R(2, 0) * q.x + R(2, 1) * q.y + R(2, 2) * q.z;
  return r;
}

Point3D_t operator*(const Eigen::Matrix<double, 4, 4>& T, const Point3D_t& p) {
  const Eigen::Matrix3d R = T.block<3, 3>(0, 0);
  const Eigen::Vector3d t = T.block<3, 1>(0, 3);
  return R * p + t;
}

Point2D_t operator*(const Sophus::SE2<double>& T, const Point2D_t& p) {
  return PtEigen2Inner(T * PtInner2Eigen(p));
}

Point3D_t operator*(const SE3d& T, const Point3D_t& p) {
  return PtEigen2Inner(T * PtInner2Eigen(p));
}

Point2D_t GetLineMainDirection2D(const std::vector<Point3D_t>& points,
                                 int i_th) {
  Eigen::Vector2d mean_point(0, 0);
  for (size_t m = 0; m < points.size(); ++m) {
    mean_point(0) += points[m].x;
    mean_point(1) += points[m].y;
  }
  mean_point /= points.size();
  Eigen::Matrix2d center_cov = Eigen::Matrix2d::Zero();
  for (size_t m = 0; m < points.size(); m++) {
    Eigen::Vector2d temp =
        Eigen::Vector2d(points[m].x, points[m].y) - mean_point;
    center_cov += temp * temp.transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> saes(center_cov);
  Eigen::Vector2d main_direction = saes.eigenvectors().col(i_th);
  return PtEigen2Inner(main_direction);
}

std::vector<int> VoxelDownSample2D(const std::vector<Point3D_t>& points,
                                   const std::vector<Eigen::Matrix2d>& covs,
                                   double sample_dist, double min_x,
                                   double max_x, double min_y, double max_y) {
  std::unordered_map<VoxelIndex, int, VoxelIndexHash> voxels;
  VoxelIndexConverter voxel_converter(sample_dist);
  for (int i = 0; i < points.size(); ++i) {
    const auto& pt = points[i];
    if (pt.x < min_x || pt.x > max_x || pt.y < min_y || pt.y > max_y) continue;
    auto index = voxel_converter.PointInnerToVoxelIndex(Point2D_t(pt.x, pt.y));
    auto iter = voxels.find(index);
    if (iter == voxels.end()) {
      voxels.insert({index, i});
    } else {
      int i_ori = iter->second;
      double cov_original = covs[i_ori].diagonal().norm();
      double cov_other = covs[i].diagonal().norm();
      if (cov_other < cov_original) {
        iter->second = i;
      }
    }
  }
  std::vector<int> sampled_idxs;
  sampled_idxs.reserve(voxels.size());
  for (const auto& item : voxels) {
    sampled_idxs.emplace_back(item.second);
  }
  return sampled_idxs;
}

bool PolyLineLeastSquareFitting(const std::vector<Point3D_t>& points,
                                const std::vector<double>& weights,
                                size_t order, Eigen::VectorXd* line_param,
                                double* residual_mean, double* redisual_max) {
  if (points.size() < 2) return false;
  if (order > points.size() - 1) return false;
  if (points.size() != weights.size()) return false;

  Eigen::MatrixXd A(points.size(), order + 1);
  Eigen::VectorXd b(points.size());

  // construct A and b, fitting polyline: y = a*x^3 + b*x^2 + c*x + d
  for (size_t i = 0; i < points.size(); ++i) {
    A(i, order) = weights[i];
  }
  for (size_t i = 0; i < points.size(); ++i) {
    for (int j = order - 1; j >= 0; --j) {
      A(i, j) = std::pow(points[i].x, order - j) * weights[i];
    }
    b(i) = points[i].y * weights[i];
  }

  *line_param = (A.transpose() * A).llt().solve(A.transpose() * b);

  // evaluate residuals if needed
  auto PolyFunc = [](const Eigen::VectorXd& param, double x) {
    double y = 0;
    int param_size = param.size();
    for (int i = 0; i < param_size; ++i) {
      y += param[i] * std::pow(x, param_size - 1 - i);
    }
    return y;
  };
  if (residual_mean != nullptr || redisual_max != nullptr) {
    double mean_residual = 0;
    double max_residual = 0;
    for (const auto& pt : points) {
      double res = std::fabs(pt.y - PolyFunc(*line_param, pt.x));
      mean_residual += res;
      if (res > max_residual) max_residual = res;
    }
    mean_residual /= points.size();
    if (residual_mean != nullptr) *residual_mean = mean_residual;
    if (redisual_max != nullptr) *redisual_max = max_residual;
  }
  return true;
}

bool AdaptedPolyfitLineWithLength(const std::vector<Point3D_t>& points,
                                  const std::vector<double>& weights,
                                  Eigen::Vector4d* line_param) {
  if (points.size() < 2 || !line_param) return false;
  if (points.size() != weights.size()) return false;

  line_param->setZero();
  double line_length = (points.back() - points.front()).Norm2D();

  int order = 0;
  double mean_residual = 0, max_residual = 0;
  bool has_fit = false;
  if (points.size() > 5 && line_length > 20.) {
    // cubic line
    order = 3;
    Eigen::VectorXd cubic_line_param;
    has_fit =
        PolyLineLeastSquareFitting(points, weights, order, &cubic_line_param,
                                   &mean_residual, &max_residual);
    if (has_fit) *line_param = cubic_line_param;
  } else if (points.size() > 2 && line_length > 5.) {
    // straight line
    order = 1;
    Eigen::VectorXd straight_line_param;
    has_fit =
        PolyLineLeastSquareFitting(points, weights, order, &straight_line_param,
                                   &mean_residual, &max_residual);
    if (has_fit) {
      (*line_param)(2) = straight_line_param(0);
      (*line_param)(3) = straight_line_param(1);
    }
  }
  return has_fit;
}

bool TriangulatePoint(const std::vector<Point3D_t>& pts,
                      const std::vector<SE3d>& proj_mats, Point3D_t* point_3d) {
  if (pts.size() != proj_mats.size()) return false;
  if (pts.size() < 2) return false;
  Eigen::MatrixXd svd_A(2 * pts.size(), 4);
  int svd_idx = 0;

  for (size_t i = 0; i < pts.size(); ++i) {
    const auto& pt_i = pts[i];
    Eigen::Matrix4d P_i = proj_mats[i].matrix();
    svd_A.row(svd_idx++) = pt_i.x * P_i.row(2) - pt_i.z * P_i.row(0);
    svd_A.row(svd_idx++) = pt_i.y * P_i.row(2) - pt_i.z * P_i.row(1);
  }

  Eigen::Vector4d svd_V =
      Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV)
          .matrixV()
          .rightCols<1>();
  Eigen::Vector3d pt_3d_eigen = svd_V.segment<3>(0) / svd_V[3];
  Point3D_t pt_3d = PtEigen2Inner(pt_3d_eigen);

  // check triangulate quality
  if (!std::isfinite(pt_3d.x) || !std::isfinite(pt_3d.y) ||
      !std::isfinite(pt_3d.z))
    return false;

  for (size_t i = 0; i < pts.size(); ++i) {
    Point3D_t pt_i_proj = proj_mats[i] * pt_3d;
    if (pt_i_proj.z < 0) return false;
    pt_i_proj = pt_i_proj / pt_i_proj.Norm();
    double cos_angle_err = pts[i].Dot(pt_i_proj);
    double dis_err = (pts[i] - pt_i_proj).Norm();
    if (cos_angle_err < 0.9999 || dis_err > 0.1) return false;
  }

  *point_3d = pt_3d;
  return true;
}

bool SortPointAndCovFromNearToFar(std::vector<Point3D_t>* points,
                                  std::vector<Eigen::Matrix2d>* covs) {
  if (points == nullptr || covs == nullptr) return false;
  if (points->size() != covs->size()) return false;

  std::vector<int> sort_ids(points->size());
  for (int i = 0; i < points->size(); ++i) {
    sort_ids[i] = i;
  }
  std::sort(sort_ids.begin(), sort_ids.end(), [&points](int i, int j) {
    double xi = (*points)[i].x;
    double xj = (*points)[j].x;
    return xi < xj;
  });
  std::vector<Point3D_t> sort_pts(points->size());
  std::vector<Eigen::Matrix2d> sort_pt_covs(points->size());
  for (int i = 0; i < points->size(); ++i) {
    sort_pts[i] = (*points)[sort_ids[i]];
    sort_pt_covs[i] = (*covs)[sort_ids[i]];
  }
  *points = std::move(sort_pts);
  *covs = std::move(sort_pt_covs);
  return true;
}

double NormalizeAngleDiff(double diff) {
  if (diff > M_PI) diff -= 2 * M_PI;
  if (diff < -M_PI) diff += 2 * M_PI;
  return diff;
}

bool BBoxIntersection(const BoundingBox2D& bbox1, const BoundingBox2D& bbox2,
                      BoundingBox2D* roi) {
  double left = std::max(bbox1.center.x - bbox1.width / 2,
                         bbox2.center.x - bbox2.width / 2);
  double right = std::min(bbox1.center.x + bbox1.width / 2,
                          bbox2.center.x + bbox2.width / 2);
  double top = std::max(bbox1.center.y - bbox1.length / 2,
                        bbox2.center.y - bbox2.length / 2);
  double bottom = std::min(bbox1.center.y + bbox1.length / 2,
                           bbox2.center.y + bbox2.length / 2);

  if (left > right || top > bottom) {
    // no intersection
    return false;
  } else {
    // intersection exists
    roi->center.x = (left + right) / 2;
    roi->center.y = (top + bottom) / 2;
    roi->width = right - left;
    roi->length = bottom - top;
    return true;
  }
}

double BoundingBoxIOU(const BoundingBox2D& bbox1, const BoundingBox2D& bbox2) {
  double area1 = bbox1.width * bbox1.length;
  double area2 = bbox2.width * bbox2.length;
  BoundingBox2D intersection;
  if (BBoxIntersection(bbox1, bbox2, &intersection)) {
    double intersection_area = intersection.width * intersection.length;
    double union_area = area1 + area2 - intersection_area;
    double iou = intersection_area / (union_area + 1e-8);
    return iou;
  } else {
    return 0.0;
  }
}

bool BBoxInterp(const BoundingBox2D& start_bbox, const BoundingBox2D& end_bbox,
                double factor, BoundingBox2D* interp_bbox) {
  if (!interp_bbox) return false;

  interp_bbox->center =
      start_bbox.center + (end_bbox.center - start_bbox.center) * factor;
  interp_bbox->width =
      start_bbox.width + (end_bbox.width - start_bbox.width) * factor;
  interp_bbox->length =
      start_bbox.length + (end_bbox.length - start_bbox.length) * factor;

  return true;
}

std::pair<int, int> BinarySearchTwoNearestMapLinesId(
    double search_y, const std::vector<Eigen::Vector4d>& map_line_params) {
  int param_size = 4;
  int left = 0, right = map_line_params.size() - 1;
  if (search_y >= map_line_params[right][param_size - 1]) {
    left = right;
    right = -1;
  } else if (search_y <= map_line_params[left][param_size - 1]) {
    right = left;
    left = -1;
  } else {
    while (right > left + 1) {
      int mid = left + (right - left) / 2;
      double mid_y = map_line_params[mid][param_size - 1];
      if (mid_y < search_y) {
        left = mid;
      } else {
        right = mid;
      }
    }
  }
  return {left, right};
}

double FourDegreePolyFunc(const Eigen::Vector4d& line_param, double x) {
  double y = 0;
  Eigen::Vector4d xs(x * x * x, x * x, x, 1);
  y = line_param.transpose() * xs;
  return y;
}

bool CheckTwoPolyFitLineParallel(const Eigen::Vector4d& line1_param,
                                 const Eigen::Vector4d& line2_param) {
  double y_var = 0., y_mean = 0.;
  double threshold = 10.0, step = 2.0;
  int points_size = 2 * static_cast<int>(threshold / step) + 1;
  std::vector<double> y_diffs;
  y_diffs.reserve(points_size);
  for (double pt_x = -threshold; pt_x <= threshold; pt_x += step) {
    double pt_y1 = FourDegreePolyFunc(line1_param, pt_x);
    double pt_y2 = FourDegreePolyFunc(line2_param, pt_x);
    double y_diff = pt_y1 - pt_y2;
    y_diffs.emplace_back(y_diff);
    y_mean += y_diff;
  }
  y_mean /= points_size;
  for (const auto& y_diff : y_diffs) {
    y_var += (y_diff - y_mean) * (y_diff - y_mean);
  }
  y_var /= points_size;
  double y_std = std::sqrt(y_var);
  if (y_std > 0.2) {
    return false;
  }
  return true;
}

bool CheckPerceptPointsMapLineParallel(
    const std::vector<Point2DWithCov>& percept_points,
    const Eigen::Vector4d& map_line_param, Sophus::SE2<double> transform) {
  if (percept_points.empty()) return false;
  int points_size = percept_points.size();
  std::vector<double> y_diffs, weights;
  y_diffs.reserve(points_size);
  weights.reserve(points_size);

  double y_var = 0., y_mean = 0., w_sum = 0.;
  for (const auto& percept_point : percept_points) {
    if (percept_point.point.x < -10. || percept_point.point.x > 20.) continue;
    Point2D_t transformed_point;
    transformed_point = transform * percept_point.point;
    double map_y = FourDegreePolyFunc(map_line_param, transformed_point.x);
    double y_diff = transformed_point.y - map_y;
    double weight = 1.0 / std::max(percept_point.cov.diagonal().norm(), 1e-4);
    y_diffs.emplace_back(y_diff);
    weights.emplace_back(weight);
    w_sum += weight;
  }
  if (y_diffs.empty()) return false;

  for (auto& weight : weights) weight /= w_sum;
  for (int i = 0; i < y_diffs.size(); i++) {
    y_mean += weights[i] * y_diffs[i];
  }
  for (int i = 0; i < y_diffs.size(); i++) {
    y_var += weights[i] * (y_diffs[i] - y_mean) * (y_diffs[i] - y_mean);
  }
  double y_std = std::sqrt(y_var);
  if (y_std > 0.2) {
    return false;
  }
  return true;
}

double CalculateDistbtwTwoPolyFitLine(const Eigen::Vector4d& line1_param,
                                      const Eigen::Vector4d& line2_param) {
  double y_mean = 0.;
  double threshold = 10.0, step = 2.0;
  int points_size = std::floor((2 * threshold) / step) + 1;
  std::vector<double> y_diffs;
  y_diffs.reserve(points_size);
  for (double pt_x = -threshold; pt_x <= threshold; pt_x += step) {
    double pt_y1 = FourDegreePolyFunc(line1_param, pt_x);
    double pt_y2 = FourDegreePolyFunc(line2_param, pt_x);
    double y_diff = pt_y1 - pt_y2;
    y_diffs.emplace_back(y_diff);
    y_mean += y_diff;
  }
  y_mean /= points_size;
  return y_mean;
}

double CalculateDistbtwPerceptPointsMapLine(
    const std::vector<Point2DWithCov>& percept_points,
    const Eigen::Vector4d& map_line_param, Sophus::SE2<double> transform) {
  if (percept_points.empty()) {
    return 1e9;
  }

  int points_size = percept_points.size();
  std::vector<double> y_diffs, weights;
  y_diffs.reserve(points_size);
  weights.reserve(points_size);

  double y_mean = 0., w_sum = 0.;
  for (const auto& percept_point : percept_points) {
    if (percept_point.point.x < -10. || percept_point.point.x > 20.) continue;
    Point2D_t transformed_point;
    transformed_point = transform * percept_point.point;
    double map_y = FourDegreePolyFunc(map_line_param, transformed_point.x);
    double y_diff = transformed_point.y - map_y;
    double weight = 1.0 / std::max(percept_point.cov.diagonal().norm(), 1e-4);
    y_diffs.emplace_back(y_diff);
    weights.emplace_back(weight);
    w_sum += weight;
  }
  if (y_diffs.empty()) {
    return 1e9;
  }

  for (auto& weight : weights) weight /= w_sum;
  for (int i = 0; i < y_diffs.size(); i++) {
    y_mean += weights[i] * y_diffs[i];
  }
  return y_mean;
}

double CalculateDistbtwTwoPerceptPoints(
    const std::vector<Point2DWithCov>& percept_points1,
    const std::vector<Point2DWithCov>& percept_points2) {
  int percept_points1_size = percept_points1.size();
  int percept_points2_size = percept_points2.size();
  std::vector<Point2D_t> src_points;
  std::vector<Point2D_t> tgt_points;
  src_points.reserve(percept_points1_size);
  tgt_points.reserve(percept_points2_size);
  for (const auto& percept_point : percept_points1) {
    src_points.emplace_back(percept_point.point);
  }
  for (const auto& percept_point : percept_points2) {
    tgt_points.emplace_back(percept_point.point);
  }
  int valid_num = 0;
  double dist = Line2LineDistance2D(src_points, tgt_points, &valid_num, true);
  if (valid_num == 0) return 1e10;
  return dist;
}

bool OptimizeYawByPerceptPointsMapLine(
    const std::vector<std::vector<Point2DWithCov>>& percept_points,
    const std::vector<Eigen::Vector4d>& percept_line_params,
    const std::vector<Eigen::Vector4d>& map_line_params,
    double* solve_heading) {
  if (percept_points.size() != percept_line_params.size() ||
      percept_line_params.size() != map_line_params.size()) {
    LC_LINFO(debug)
        << "OptimizeYawByPerceptPointsMapLine input doesn't satisfy";
    return false;
  }
  double optimize_heading = 0., delta_heading = 1.;
  int iter = 0;
  while (iter < 3 && std::fabs(delta_heading) > 1e-3) {
    double H = 0., b = 0., residual_sum = 0.;
    Eigen::Matrix2d rotation;
    double cosh = std::cos(optimize_heading), sinh = std::sin(optimize_heading);
    rotation << cosh, -sinh, sinh, cosh;
    for (int k = 0; k < percept_points.size(); k++) {
      const auto& percept_line_param = percept_line_params[k];
      const auto& map_line_param = map_line_params[k];
      for (const auto& percept_point : percept_points[k]) {
        if (percept_point.point.x < -10. || percept_point.point.x > 20.)
          continue;
        // estimate point cov
        double percept_y =
            FourDegreePolyFunc(percept_line_param, percept_point.point.x);
        double sigma = 0.05;
        double weight = std::exp(
            -std::pow((percept_y - percept_point.point.y) / sigma, 2.0));

        // estimete point residual and jacobian
        Point2D_t rotated_point = rotation * percept_point.point;
        double map_y = FourDegreePolyFunc(map_line_param, rotated_point.x);

        int n = map_line_param.size();
        double residual = map_y - rotated_point.y -
                          (map_line_param(n - 1) - percept_line_param(n - 1));

        double J_residual_x = 0;
        for (int i = 0; i < n - 1; i++) {
          J_residual_x += map_line_param(i) * (n - 1 - i) *
                          std::pow(rotated_point.x, n - 2 - i);
        }
        double J_residual_y = -1;
        double J_x_heading =
            -sinh * percept_point.point.x - cosh * percept_point.point.y;
        double J_y_heading =
            cosh * percept_point.point.x - sinh * percept_point.point.y;
        double J = J_residual_x * J_x_heading + J_residual_y * J_y_heading;

        double threshold = 0.5;
        if (residual > threshold) {
          residual = threshold;
          J = 0.;
        } else if (residual < -threshold) {
          residual = -threshold;
          J = 0.;
        }

        H += J * J * weight;
        b += -J * residual * weight;
        residual_sum += residual * residual;
      }
    }
    delta_heading = b / std::max(H, 1e-4);
    optimize_heading += delta_heading;

    if (optimize_heading > M_PI) optimize_heading -= M_PI;
    if (optimize_heading < -M_PI) optimize_heading += M_PI;

    iter++;
  }
  *solve_heading = optimize_heading;
  return true;
}
}  // namespace smm
}  // namespace localization
}  // namespace senseAD
