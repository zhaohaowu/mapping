/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */

#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/opencv.hpp>

#include "common/visualizer_util.hpp"
#include "localization/data_type/base.hpp"
#include "localization/data_type/smart_ptr.hpp"

namespace senseAD {
namespace localization {
namespace smm {

class FramePackage;
class MapManager;
class TrackingManager;

// class holds the visualization, contains draw frame percetion results, and map
// roadstructure, matching results etc.
class Visualization {
 public:
  DEFINE_SMART_PTR(Visualization)

  Visualization() = default;
  ~Visualization() = default;

  // @brief: draw frame package and map
  cv::Mat Draw(const std::shared_ptr<FramePackage>& frame_package,
               const std::shared_ptr<MapManager>& map_manager,
               const std::shared_ptr<TrackingManager>& tracking_manager);

 private:
  // @brief: draw road line points in bird view
  void DrawRoadLineDataOptimPose(
      const std::shared_ptr<FramePackage>& frame_package,
      const std::shared_ptr<MapManager>& map_manager,
      const std::shared_ptr<TrackingManager>& tracking_manager,
      cv::Mat* draw_image) const;

  void DrawRoadLineDataCalibHomo(
      const std::shared_ptr<FramePackage>& frame_package,
      cv::Mat* draw_image) const;

  // @brief: draw covariance ellipse in bird view
  void DrawPoseAndCovEllipse(const std::shared_ptr<FramePackage>& frame_package,
                             cv::Mat* draw_image) const;

  // @brief: generate smm info to display
  void MakeSMMInfo(const std::shared_ptr<FramePackage>& frame_package);

  // @brief: draw smm info
  void DrawSMMInfo(cv::Mat* draw_image) const;

  // @brief: draw vehicle in bird view
  void DrawVehicle(const cv::Point2d& position, const double heading_angle,
                   const cv::Scalar& color, cv::Mat* draw_image) const;

  // @brief: draw polyfit line
  void DrawPerceptPolyfitLine(
      const std::shared_ptr<FramePackage>& frame_package,
      cv::Mat* draw_image) const;

  void DrawMapPolyfitLine(const std::shared_ptr<FramePackage>& frame_package,
                          const SE3d& transform, cv::Mat* draw_image) const;

  void GeneratePolyfitLinePoints(const Eigen::Vector4d& line_param,
                                 const SE3d& transform,
                                 std::vector<cv::Point2d>* cv_points) const;

  // @breif: draw heading valid array
  void DrawSearchHeadingValidArray(
      const std::shared_ptr<FramePackage>& frame_package, const SE3d& transform,
      cv::Mat* draw_image) const;

  // @brief: save draw image
  void SaveImage(const cv::Mat& draw_image);

 private:
  // window size parameters, unit: pixel
  const int image_width_ = 800;
  const int image_height_ = 1000;

  // parameters for converting metric point to pixel
  const VisualizerUtil::Metric2PixelPara bv_view_matric_{
      40.0 / image_width_, 80.0 / image_height_,
      Point2D_t(image_width_ * 0.5 + 60.0, image_height_ * 0.5 + 140.0),
      Point2D_t(100.0, image_width_), Point2D_t(50.0, image_height_)};

  // for save image
  bool save_image_{false};
  double lateral_err_;
  double heading_err_;

  bool draw_pt_cov_{false};

  std::vector<std::pair<cv::Scalar, std::string>> smm_info_;
};

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
