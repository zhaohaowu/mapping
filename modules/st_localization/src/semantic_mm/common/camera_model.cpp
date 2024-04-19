/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */
#include "semantic_mm/common/camera_model.hpp"

#include "semantic_mm/common/math_tools.hpp"

namespace senseAD {
namespace localization {
namespace smm {

CameraModel::CameraModel(uint32_t width, uint32_t height, float64_t fx,
                         float64_t fy, float64_t cx, float64_t cy)
    : camera_width_(width),
      camera_height_(height),
      fx_(fx),
      fy_(fy),
      cx_(cx),
      cy_(cy) {}

void CameraModel::SetCameraIntrinsic(float64_t fx, float64_t fy, float64_t cx,
                                     float64_t cy) {
  fx_ = fx;
  fy_ = fy;
  cx_ = cx;
  cy_ = cy;
}

float64_t CameraModel::GetFx() const { return fx_; }

float64_t CameraModel::CameraModel::GetFy() const { return fy_; }

float64_t CameraModel::GetCx() const { return cx_; }

float64_t CameraModel::GetCy() const { return cy_; }

Eigen::Matrix3d CameraModel::GetCameraIntrinsic() const {
  Eigen::Matrix3d K;
  K << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1;
  return K;
}

void CameraModel::SetCameraSize(uint32_t camera_width, uint32_t camera_height) {
  camera_width_ = camera_width;
  camera_height_ = camera_height;
}

uint32_t CameraModel::GetCameraWidth() const { return camera_width_; }

uint32_t CameraModel::GetCameraHeight() const { return camera_height_; }

bool CameraModel::Pixel2Camera(const Point2D_t& pixel_pt,
                               Point3D_t* cam_pt) const {
  cam_pt->x = (pixel_pt.x - cx_) / fx_;
  cam_pt->y = (pixel_pt.y - cy_) / fy_;
  cam_pt->z = 1.0;
  return true;
}

bool CameraModel::Camera2Pixel(const Point3D_t& cam_pt,
                               Point2D_t* pixel_pt) const {
  pixel_pt->x = fx_ * cam_pt.x / cam_pt.z + cx_;
  pixel_pt->y = fy_ * cam_pt.y / cam_pt.z + cy_;

  return !(pixel_pt->x < 0 || pixel_pt->y < 0 ||
           pixel_pt->x > camera_width_ - 1 || pixel_pt->y > camera_height_ - 1);
}

bool CameraModel::World2Pixel(const Point3D_t& wpt, const Eigen::Matrix4d& Tcw,
                              Point2D_t* pixel_pt) const {
  // convert world pt to camera pt
  const Eigen::Matrix3d& Rcw = Tcw.block<3, 3>(0, 0);
  const Eigen::Vector3d& tcw = Tcw.block<3, 1>(0, 3);
  Point3D_t cam_pt = Rcw * wpt + tcw;

  // convert camera pt to pixel
  return Camera2Pixel(cam_pt, pixel_pt);
}

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
