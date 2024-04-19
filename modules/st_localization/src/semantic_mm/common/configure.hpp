/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */

#pragma once

#include <map>
#include <memory>
#include <string>
#include <utility>

#include <Sophus/se3.hpp>

#include "localization/data_type/base.hpp"
#include "localization/localization_param.hpp"

namespace senseAD {
namespace localization {
namespace smm {

class CameraModel;

class Configure {
 public:
  ~Configure() {}
  static Configure* GetInstance();

  // @brief: init
  adLocStatus_t Init(const LocalizationParam& param);

  // @brief: get localization param
  const LocalizationParam& GetLocalizationParam() const;
  const LocalizationSMMParam& GetLocalizationSMMParam() const;

  // @brief: get transformation from ground to car-center
  SE3d GetTGround2Vehicle() const;

  // @brief: get camera model related
  bool HasCameraModel(const std::string& camera_name) const;
  adLocStatus_t GetCameraModel(
      const std::string& camera_name,
      std::shared_ptr<CameraModel>* camera_model) const;
  const std::map<std::string, std::shared_ptr<CameraModel>>& GetCameraModels()
      const;

  // @brief: get camera homography related
  bool HasCameraHomography(const std::string& camera_name) const;
  adLocStatus_t GetCameraHomography(const std::string& camera_name,
                                    Eigen::Matrix3d* homo) const;
  const std::map<std::string, Eigen::Matrix3d>& GetCameraHomographys() const;

  adLocStatus_t GetCameraToGroundExtrinsic(const std::string& camera_name,
                                           SE3d* Tgc) const;

  adLocStatus_t GetCameraHomographyBaseComponent(const std::string& camera_name,
                                                 Eigen::Matrix3d* H_base,
                                                 Eigen::Matrix3d* K_inv) const;

  const std::map<std::string, std::pair<Eigen::Matrix3d, Eigen::Matrix3d>>&
  GetCameraHomographyBaseComponents() const;

  // @brief: update/get multi camera homography pitch difference estimation
  void UpdateMultiCameraDiffPitch(double pitch_fov30, double pitch_fov120);
  void GetMultiCameraDiffPitch(double* mean, double* var, int* cnt) const;

  // @brief: get camera extrinsic related
  bool HasCameraExtrinsic(const std::string& camera_name) const;
  adLocStatus_t GetCameraExtrinsic(const std::string& camera_name,
                                   SE3d* extrinsic) const;
  const std::map<std::string, SE3d>& GetCameraExtrinsics() const;

 private:
  // @brief: disable construction
  Configure() = default;
  Configure(const Configure&) = delete;
  Configure& operator=(const Configure&) = delete;

 private:
  // localization param
  LocalizationParam param_;

  // extrinsic from ground to car-center
  SE3d Tveh_ground_;

  // camera models
  std::map<std::string, std::shared_ptr<CameraModel>> camera_models_;

  // camera homographys (convert from image to bird view)
  std::map<std::string, Eigen::Matrix3d> camera_Hs_;

  // camera extrinsics (camera -> car-center)
  std::map<std::string, SE3d> camera_extrinsics_;

  // base components for homographys, <camera, <H_base, K_inv>>
  std::map<std::string, std::pair<Eigen::Matrix3d, Eigen::Matrix3d>>
      camera_H_bases_;

  // camera extrinsics (camera -> ground)
  std::map<std::string, SE3d> camera_to_ground_extrinsics_;

  // for homography diff estimation
  double diff_h_pitch_{0};
  double diff_h_pitch_var_{0};
  double count_{0};
};

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
