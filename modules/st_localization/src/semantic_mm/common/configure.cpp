/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */
#include "semantic_mm/common/configure.hpp"

#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include "common/transform_config.hpp"
#include "localization/common/log.hpp"
#include "semantic_mm/common/camera_model.hpp"
#include "semantic_mm/common/math_tools.hpp"

namespace senseAD {
namespace localization {
namespace smm {

Configure* Configure::GetInstance() {
  static Configure instance;
  return &instance;
}

adLocStatus_t Configure::Init(const LocalizationParam& param) {
  param_ = param;

  // load ground to car-center extrinsic
  if (!TransformConfig::GetTVehGround(&Tveh_ground_)) {
    LC_LERROR(COMMON) << "load ground to car-center extrinsic failed.";
    return LOC_LOCALIZATION_ERROR;
  }

  // set camera related intrinsic and extrinsic
  for (const auto& camera_name : param_.smm_param.enable_camera_names) {
    // intrinsic
    TransformConfig::CamIntrinsic intrinsic;
    if (!TransformConfig::GetCameraIntrinsic(camera_name, &intrinsic)) {
      LC_LERROR(COMMON) << "load camera intrinsic failed.";
      return LOC_LOCALIZATION_ERROR;
    }
    cv::Mat K = intrinsic.K;
    camera_models_[camera_name] = std::make_shared<CameraModel>(
        intrinsic.cam_width, intrinsic.cam_height, K.at<float>(0, 0),
        K.at<float>(1, 1), K.at<float>(0, 2), K.at<float>(1, 2));

    // homography
    cv::Mat H;
    if (!TransformConfig::GetCameraHomography(camera_name, &H)) {
      LC_LERROR(COMMON) << "load camera homography failed.";
      return LOC_LOCALIZATION_ERROR;
    }
    Eigen::Matrix3d H_eigen;
    cv::cv2eigen(H, H_eigen);
    camera_Hs_[camera_name] = H_eigen;

    // extrinsic
    SE3d extrinsic;
    if (!TransformConfig::GetTVehCam(camera_name, &extrinsic)) {
      LC_LERROR(COMMON) << "load camera extrinsic failed.";
      return LOC_LOCALIZATION_ERROR;
    }
    camera_extrinsics_[camera_name] = extrinsic;

    // camera to ground extrincis
    SE3d Tveh_cam = extrinsic;
    SE3d Tground_cam = Tveh_ground_.inverse() * Tveh_cam;
    camera_to_ground_extrinsics_[camera_name] = Tground_cam;

    // homography base from Tground_cam
    Eigen::Matrix3d K_inv =
        camera_models_[camera_name]->GetCameraIntrinsic().inverse();
    Eigen::Matrix3d Rgc = Tground_cam.rotationMatrix();
    Eigen::Vector3d tgc = Tground_cam.translation();
    Eigen::Vector3d shift(0, 0, 1);
    double d_g = tgc(2);          // cam to ground distance
    Eigen::Vector3d n(0, 0, -1);  // ground norm vector in global frame
    Eigen::Matrix3d H_base =
        Rgc + 1.0 / d_g * (tgc + shift) * n.transpose() * Rgc;

    // check consistency
    Eigen::Matrix3d H_from_base = H_base * K_inv;
    double homo_inconsistency_error =
        (H_from_base / H_from_base(2, 2) - H_eigen / H_eigen(2, 2)).norm();
    if (homo_inconsistency_error > 1e-2) {
      LC_LERROR(COMMON) << "inconsistency between homography and camera "
                           "intrinsic/extrinsic! camera name: "
                        << camera_name
                        << ", error: " << homo_inconsistency_error;
      // return LOC_LOCALIZATION_ERROR;
    }
    camera_H_bases_[camera_name] = std::make_pair(H_base, K_inv);
  }

  return LOC_SUCCESS;
}

const LocalizationParam& Configure::GetLocalizationParam() const {
  return param_;
}

const LocalizationSMMParam& Configure::GetLocalizationSMMParam() const {
  return param_.smm_param;
}

SE3d Configure::GetTGround2Vehicle() const { return Tveh_ground_; }

bool Configure::HasCameraModel(const std::string& camera_name) const {
  return camera_models_.find(camera_name) != camera_models_.end();
}

adLocStatus_t Configure::GetCameraModel(const std::string& camera_name,
                                        CameraModel::Ptr* camera_model) const {
  if (!camera_model) return LOC_NULL_PTR;
  auto iter = camera_models_.find(camera_name);
  if (iter == camera_models_.end()) return LOC_OUT_OF_RANGE;
  *camera_model = iter->second;
  return LOC_SUCCESS;
}

const std::map<std::string, CameraModel::Ptr>& Configure::GetCameraModels()
    const {
  return camera_models_;
}

bool Configure::HasCameraHomography(const std::string& camera_name) const {
  return camera_Hs_.find(camera_name) != camera_Hs_.end();
}

adLocStatus_t Configure::GetCameraHomography(const std::string& camera_name,
                                             Eigen::Matrix3d* homo) const {
  if (!homo) return LOC_NULL_PTR;
  auto iter = camera_Hs_.find(camera_name);
  if (iter == camera_Hs_.end()) return LOC_OUT_OF_RANGE;
  *homo = iter->second;
  return LOC_SUCCESS;
}

const std::map<std::string, Eigen::Matrix3d>& Configure::GetCameraHomographys()
    const {
  return camera_Hs_;
}

adLocStatus_t Configure::GetCameraToGroundExtrinsic(
    const std::string& camera_name, SE3d* Tgc) const {
  if (!Tgc) return LOC_NULL_PTR;
  auto iter = camera_to_ground_extrinsics_.find(camera_name);
  if (iter == camera_to_ground_extrinsics_.end()) return LOC_OUT_OF_RANGE;
  *Tgc = iter->second;
  return LOC_SUCCESS;
}

const std::map<std::string, std::pair<Eigen::Matrix3d, Eigen::Matrix3d>>&
Configure::GetCameraHomographyBaseComponents() const {
  return camera_H_bases_;
}

adLocStatus_t Configure::GetCameraHomographyBaseComponent(
    const std::string& camera_name, Eigen::Matrix3d* H_base,
    Eigen::Matrix3d* K_inv) const {
  if (!H_base || !K_inv) return LOC_NULL_PTR;
  auto iter = camera_H_bases_.find(camera_name);
  if (iter == camera_H_bases_.end()) return LOC_OUT_OF_RANGE;
  *H_base = iter->second.first;
  *K_inv = iter->second.second;
  return LOC_SUCCESS;
}

bool Configure::HasCameraExtrinsic(const std::string& camera_name) const {
  return camera_extrinsics_.find(camera_name) != camera_extrinsics_.end();
}

adLocStatus_t Configure::GetCameraExtrinsic(const std::string& camera_name,
                                            SE3d* extrinsic) const {
  if (!extrinsic) return LOC_NULL_PTR;
  auto iter = camera_extrinsics_.find(camera_name);
  if (iter == camera_extrinsics_.end()) return LOC_OUT_OF_RANGE;
  *extrinsic = iter->second;
  return LOC_SUCCESS;
}

const std::map<std::string, SE3d>& Configure::GetCameraExtrinsics() const {
  return camera_extrinsics_;
}

void Configure::UpdateMultiCameraDiffPitch(double pitch_fov30,
                                           double pitch_fov120) {
  double diff = pitch_fov30 - pitch_fov120;
  diff = NormalizeAngleDiff(diff);
  ++count_;
  double v1 = diff - diff_h_pitch_;
  diff_h_pitch_ += v1 / count_;

  double v2 = diff - diff_h_pitch_;
  diff_h_pitch_var_ = ((count_ - 1) * diff_h_pitch_var_ + v1 * v2) / count_;
}

void Configure::GetMultiCameraDiffPitch(double* mean, double* var,
                                        int* cnt) const {
  *mean = diff_h_pitch_;
  *var = diff_h_pitch_var_;
  *cnt = count_;
}

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
