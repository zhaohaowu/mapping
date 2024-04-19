/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */

#pragma once

#include <Eigen/Core>
#include <vector>

#include "localization/data_type/base.hpp"
#include "localization/data_type/smart_ptr.hpp"

namespace senseAD {
namespace localization {
namespace smm {

// class holds the camera model, contains camera intrinsic, homography, and a
// serial of projection interface etc.
class CameraModel {
 public:
  DEFINE_SMART_PTR(CameraModel)

  CameraModel() = default;
  CameraModel(uint32_t width, uint32_t height, float64_t fx, float64_t fy,
              float64_t cx, float64_t cy);

  ~CameraModel() = default;

  // set and get interface
  void SetCameraIntrinsic(float64_t fx, float64_t fy, float64_t cx,
                          float64_t cy);
  float64_t GetFx() const;
  float64_t GetFy() const;
  float64_t GetCx() const;
  float64_t GetCy() const;
  Eigen::Matrix3d GetCameraIntrinsic() const;

  void SetCameraSize(uint32_t camera_width, uint32_t camera_height);
  uint32_t GetCameraWidth() const;
  uint32_t GetCameraHeight() const;

  // convert between pixel and camera coordinate (depth normalized)
  bool Pixel2Camera(const Point2D_t& pixel_pt, Point3D_t* cam_pt) const;
  bool Camera2Pixel(const Point3D_t& cam_pt, Point2D_t* pixel_pt) const;

  // convert from world coordinate to pixel
  bool World2Pixel(const Point3D_t& wpt, const Eigen::Matrix4d& Tcw,
                   Point2D_t* pixel_pt) const;

 private:
  // camera image param
  uint32_t camera_width_;
  uint32_t camera_height_;

  // camera intrinsic, has been undistorted
  float64_t fx_, fy_, cx_, cy_;
};

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
