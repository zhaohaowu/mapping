/*
 * Copyright (C) 2021 by SenseTime Group Limited. All rights reserved.
 * Zhu Kaiying <zhukaiying@sensetime.com>
 */
#pragma once

#include <string>
#include <utility>
#include <vector>

#include <opencv2/opencv.hpp>

#include "localization/data_type/base.hpp"
#include "localization/data_type/semantic_type.hpp"

namespace senseAD {
namespace localization {

class VisualizerUtil {
 public:
  enum class Color {
    RED = 0,
    GREEN,
    BLUE,
    YELLOW,
    WHITE,
    GRAY,
    PINK,
    ORANGE,
    CHARTREUSE
  };

  enum class DrawLineStyle { Point = 0, Solid, Dashed };

  struct Metric2PixelPara {
    double lateral_scale_factor;
    double longitudinal_scale_factor;
    Point2D_t zero_pt_in_pixel;
    Point2D_t width_boundary;
    Point2D_t height_boundary;
  };

  // @brief: color render
  static cv::Scalar ColorRender(Color color);

  // @brief: convert one local point to image coordinate
  static bool CvtLocalPoint2Image(const Point2D_t& local_pt,
                                  const Metric2PixelPara& convert_para,
                                  cv::Point2d* pixel_pt);

  // @brief: calculate main axis of ellipse for displaying covariance
  static bool CvtPositionCov2Ellipse(const Eigen::Matrix3d& Rvw,
                                     const Eigen::Matrix3d& global_cov,
                                     const Metric2PixelPara& convert_para,
                                     double* half_ellipse_x_axis,
                                     double* half_ellipse_y_axis);

  // @brief: draw lines in image
  static bool DrawLine(const std::vector<cv::Point2d>& draw_pts,
                       const DrawLineStyle& draw_style, const cv::Scalar& color,
                       const double& line_width, cv::Mat* draw_image);

  // @brief: draw coordinate tick marks
  static bool DrawTickMark(
      const std::vector<double>& longitudinal_tick_mark_metric,
      const std::vector<double>& lateral_tick_mark_metric,
      const Metric2PixelPara& convert_para, cv::Mat* draw_image);
};

}  // namespace localization
}  // namespace senseAD
