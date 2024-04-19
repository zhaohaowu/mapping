/*
 * Copyright (C) 2021 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */
#pragma once

#include <Eigen/Core>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <opencv2/opencv.hpp>

#include "localization/data_type/base.hpp"
#include "localization/data_type/semantic_type.hpp"
#include "localization/data_type/smart_ptr.hpp"

namespace senseAD {
namespace localization {

// PeceptBaseType
struct PeceptBaseType {
  DEFINE_PTR(PeceptBaseType)
  DEFINE_PTR_CONTAINER(PeceptBaseType)

  int id;
  float32_t confidence;
};

// PerceptLaneLine
struct PerceptLaneLine : PeceptBaseType {
  DEFINE_PTR(PerceptLaneLine)
  DEFINE_PTR_CONTAINER(PerceptLaneLine)

  PerceptLaneLine() = default;
  PerceptLaneLine(const PerceptLaneLine& other)
      : line_width(other.line_width),
        color(other.color),
        line_type(other.line_type),
        line_style(other.line_style),
        start_point(other.start_point),
        end_point(other.end_point),
        poly_coef(other.poly_coef),
        img_pts(other.img_pts),
        linked_id(other.linked_id),
        processed_bv_points(other.processed_bv_points),
        processed_bv_point_covs(other.processed_bv_point_covs),
        processed_img_points(other.processed_img_points) {}
  ~PerceptLaneLine() = default;

  float32_t line_width;                       // line width (meter)
  Color color;                                // line color
  LineType line_type = LineType::Unknown;     // line type
  LineStyle line_style = LineStyle::Unknown;  // line style

  // raw laneline perception in BV view
  Point2D_t start_point, end_point;  // start and end point of laneline
  std::vector<float64_t> poly_coef;  // polyline coef param, [a,b,c,d]

  // raw laneline perception in image view
  std::vector<Point2D_t> img_pts;  // points in image view

  // preprocessed linked laneline id, using for roadside line
  id_t linked_id{-1};

  // preprocessed points and uncertainty cov in BV view(m)
  std::vector<Point3D_t> processed_bv_points;
  std::vector<Eigen::Matrix2d> processed_bv_point_covs;
  // preprocessed points in image view(pixel)
  std::vector<Point2D_t> processed_img_points;
};

// BoundingBox2D
struct BoundingBox2D {
  DEFINE_PTR(BoundingBox2D)
  DEFINE_PTR_CONTAINER(BoundingBox2D)

  Point2D_t center;
  double length;
  double width;
};

// PerceptTrafficSign
struct PerceptTrafficSign : PeceptBaseType {
  DEFINE_PTR(PerceptTrafficSign)
  DEFINE_PTR_CONTAINER(PerceptTrafficSign)

  BoundingBox2D rect;    // bounding box 2d in image
  TrafficSignType type;  // traffic sign type

  Point3D_t processed_center;            // center by tracking and triangulation
  Eigen::Matrix3d processed_center_cov;  // covariance of triangulated center
  size_t observ_cnt;                     // observation count in tracking
  Point2D_t move_velocity{0.0, 0.0};  // move velocity in images (pixel/100ms)
};

// PerceptPole
struct PerceptPole : PeceptBaseType {
  DEFINE_PTR(PerceptPole)
  DEFINE_PTR_CONTAINER(PerceptPole)

  BoundingBox2D rect;  // bounding box 2d in image
  PoleType type;       // pole type

  Point3D_t processed_center;            // center by tracking and triangulation
  Eigen::Matrix3d processed_center_cov;  // covariance of triangulated center
  size_t observ_cnt;                     // observation count in tracking
  Point2D_t move_velocity{0.0, 0.0};  // move velocity in images (pixel/100ms)
};

// PerceptData
class PerceptData {
 public:
  DEFINE_PTR(PerceptData)

  bool Empty() const {
    return ((lane_lines.empty() && traffic_signs.empty()) && poles.empty());
  }

 public:
  uint64_t timestamp_ns;
  std::string camera_name;
  std::unordered_map<id_t, PerceptLaneLine> lane_lines;
  std::unordered_map<id_t, PerceptTrafficSign> traffic_signs;
  std::unordered_map<id_t, PerceptPole> poles;
  cv::Mat raw_image;  // for visualization only
};

struct Point2DWithCov {
  Point2DWithCov() = default;
  Point2DWithCov(const Point2D_t& pt, const Eigen::Matrix2d& cov)
      : point(pt), cov(cov) {}
  Point2D_t point;
  Eigen::Matrix2d cov;
};

}  // namespace localization
}  // namespace senseAD
