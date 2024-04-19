/*
 * Copyright (C) 2021 by SenseTime Group Limited. All rights reserved.
 * Zhu Kaiying <zhukaiying@sensetime.com>
 */
#include "common/visualizer_util.hpp"

#include <algorithm>

namespace senseAD {
namespace localization {

cv::Scalar VisualizerUtil::ColorRender(Color color) {
  switch (color) {
    case Color::RED:
      return cv::Scalar(0, 0, 255);
    case Color::GREEN:
      return cv::Scalar(0, 255, 0);
    case Color::BLUE:
      return cv::Scalar(255, 0, 0);
    case Color::YELLOW:
      return cv::Scalar(0, 255, 255);
    case Color::WHITE:
      return cv::Scalar(255, 255, 255);
    case Color::GRAY:
      return cv::Scalar(100, 100, 100);
    case Color::PINK:
      return cv::Scalar(203, 192, 255);
    case Color::ORANGE:
      return cv::Scalar(0, 165, 255);
    case Color::CHARTREUSE:
      return cv::Scalar(0, 255, 127);
    default:
      return cv::Scalar(255, 255, 255);
  }
  return cv::Scalar(0, 0, 0);
}

bool VisualizerUtil::CvtLocalPoint2Image(const Point2D_t& local_pt,
                                         const Metric2PixelPara& convert_para,
                                         cv::Point2d* pixel_pt) {
  if (!pixel_pt) return false;

  // convert from local pt to image coordinate
  auto pixel_x = convert_para.zero_pt_in_pixel.x -
                 local_pt.y / convert_para.lateral_scale_factor;
  auto pixel_y = convert_para.zero_pt_in_pixel.y -
                 local_pt.x / convert_para.longitudinal_scale_factor;
  *pixel_pt = cv::Point2d(pixel_x, pixel_y);

  // check if in the boundary
  if (pixel_x <= convert_para.width_boundary.x ||
      pixel_x >= convert_para.width_boundary.y ||
      pixel_y <= convert_para.height_boundary.x ||
      pixel_y >= convert_para.height_boundary.y) {
    return false;
  }
  return true;
}

bool VisualizerUtil::CvtPositionCov2Ellipse(
    const Eigen::Matrix3d& Rvw, const Eigen::Matrix3d& global_cov,
    const Metric2PixelPara& convert_para, double* half_ellipse_x_axis,
    double* half_ellipse_y_axis) {
  if (!half_ellipse_x_axis || !half_ellipse_y_axis) return false;

  Eigen::Matrix3d local_cov = Rvw * global_cov * Rvw.transpose();
  double local_position_x_std = std::sqrt(std::max(local_cov(0, 0), 0.0));
  double local_position_y_std = std::sqrt(std::max(local_cov(1, 1), 0.0));

  *half_ellipse_x_axis =
      local_position_y_std / (convert_para.lateral_scale_factor + 1e-8);
  *half_ellipse_y_axis =
      local_position_x_std / (convert_para.longitudinal_scale_factor + 1e-8);

  return true;
}

bool VisualizerUtil::DrawLine(const std::vector<cv::Point2d>& draw_pts,
                              const DrawLineStyle& draw_style,
                              const cv::Scalar& color, const double& line_width,
                              cv::Mat* draw_image) {
  if (!draw_image || draw_pts.size() < 2) return false;

  if (draw_style == DrawLineStyle::Solid) {
    for (size_t i = 1; i < draw_pts.size(); ++i) {
      cv::line(*draw_image, draw_pts[i - 1], draw_pts[i], color, line_width);
    }
  } else if (draw_style == DrawLineStyle::Dashed) {
    for (size_t i = 1; i < draw_pts.size(); i += 2) {
      cv::line(*draw_image, draw_pts[i - 1], draw_pts[i], color, line_width);
    }
  } else if (draw_style == DrawLineStyle::Point) {
    for (size_t i = 0; i < draw_pts.size(); ++i) {
      cv::circle(*draw_image, draw_pts[i], line_width, color);
    }
  }
  return true;
}

bool VisualizerUtil::DrawTickMark(
    const std::vector<double>& longitudinal_tick_mark_metric,
    const std::vector<double>& lateral_tick_mark_metric,
    const Metric2PixelPara& convert_para, cv::Mat* draw_image) {
  if (!draw_image) return false;

  static const cv::Scalar mark_color = ColorRender(Color::WHITE);

  for (auto metric : longitudinal_tick_mark_metric) {
    cv::Point2d cv_pt;
    if (CvtLocalPoint2Image(Point2D_t(metric, 0), convert_para, &cv_pt)) {
      cv::line(*draw_image, cv::Point(draw_image->cols - 10, cv_pt.y),
               cv::Point(draw_image->cols - 1, cv_pt.y), mark_color, 5);
      std::string number = std::to_string(static_cast<int>(metric));
      cv::putText(*draw_image, number,
                  cv::Point(draw_image->cols - 30, cv_pt.y + 15),
                  cv::FONT_HERSHEY_SIMPLEX, 0.4, mark_color);
    }
  }

  // draw lateral tick mark to image
  for (auto metric : lateral_tick_mark_metric) {
    cv::Point2d cv_pt;
    if (CvtLocalPoint2Image(Point2D_t(0, metric), convert_para, &cv_pt)) {
      cv::line(*draw_image, cv::Point(cv_pt.x, draw_image->rows - 10),
               cv::Point(cv_pt.x, draw_image->rows - 1), mark_color, 5);
      std::string number = std::to_string(static_cast<int>(metric));
      cv::putText(*draw_image, number,
                  cv::Point(cv_pt.x, draw_image->rows - 20),
                  cv::FONT_HERSHEY_SIMPLEX, 0.4, mark_color);
    }
  }
  return true;
}

}  // namespace localization
}  // namespace senseAD
