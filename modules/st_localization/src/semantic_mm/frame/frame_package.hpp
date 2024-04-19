/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */

#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Sophus/se3.hpp>

#include "eval/evaluator_smm.hpp"
#include "localization/data_type/base.hpp"
#include "localization/data_type/smart_ptr.hpp"

namespace senseAD {
namespace localization {
namespace smm {

class Frame;

// class holds the frame package, contains multi synced frames
class FramePackage {
 public:
  DEFINE_SMART_PTR(FramePackage);

  FramePackage() = default;
  explicit FramePackage(const std::vector<std::shared_ptr<Frame>>& frames);
  ~FramePackage() = default;

  // @brief: set and get interface
  void SetId(uint64_t id);
  uint64_t GetId() const;

  void SetInitPoseState(const SE3d& pose_state);
  SE3d GetInitPoseState() const;

  void SetInitialPoseCorrection(const NavState& pose_state);
  NavState GetInitialPoseCorrection() const;

  void SetPoseState(const SE3d& pose_state);
  SE3d GetPoseState() const;

  void SetPoseCov(const Eigen::Matrix<double, 6, 6>& pose_cov);
  Eigen::Matrix<double, 6, 6> GetPoseCov() const;

  void AddFrame(const std::shared_ptr<Frame>& frame);
  void SetFrames(const std::vector<std::shared_ptr<Frame>>& frames);
  adLocStatus_t GetFrame(const std::string& name,
                         std::shared_ptr<Frame>* frame) const;
  const std::vector<std::shared_ptr<Frame>>& GetFrames() const;

  void SetGTState(const NavState& gt_state);
  adLocStatus_t GetGTState(NavState* state) const;

  void SetGnssState(const NavState& gnss_state);
  adLocStatus_t GetGnssState(NavState* state) const;

  void SetSMMStatus(bool smm_success);
  bool GetSMMStatus() const;

  std::unordered_map<std::string, SE3d> GetTransformSubToMain(
      bool use_odom_state) const;

  std::unordered_map<std::string, double> GetHomoPitch() const;

  // @brief: preprocess for frames
  adLocStatus_t PreProcess();

  // @brief: get if in relocalization mode
  void SetRelocalizationMode(bool relocal_mode);
  bool GetRelocalizationMode() const;

  // @brief: check quality of perception data in bird view
  adLocStatus_t CheckPerceptionData(SMMEvalData* eval_data) const;

  // @brief: set/get matched data for visualization
  void SetMatchedLanelineData(const std::vector<Point3D_t>& matched_points);
  const std::vector<Point3D_t>& GetMatchedLanelineData() const;

  void SetMatchedMapLanelineData(
      const std::vector<Point3D_t>& matched_map_points);
  const std::vector<Point3D_t>& GetMatchedMapLanelineData() const;

  void SetMatchedMapLanelineDataForHomoCalib(
      const std::vector<Point3D_t>& matched_map_points);
  const std::vector<Point3D_t>& GetMatchedMapLanelineDataForHomoCalib() const;

  void SetRelocalDistributions(
      const std::vector<std::vector<std::pair<double, double>>>& distribs);
  const std::vector<std::vector<std::pair<double, double>>>&
  GetRelocalDistributions() const;

  // @brief: set/get percept laneline polyfit param for visualization
  void SetPerceptPolyfitParam(const Eigen::Vector4d& left_param,
                              const Eigen::Vector4d& right_param, bool valid);
  bool GetPerceptPolyfitParam(Eigen::Vector4d* left_param,
                              Eigen::Vector4d* right_param) const;

  // @brief: set/get map laneline polyfit param for visualization
  void SetMapLinePolyfitParam(
      const std::vector<Eigen::Vector4d>& map_line_params);
  const std::vector<Eigen::Vector4d>& GetMapLinePolyfitParam() const;

  // @brief: set/get heading valid array for visualization
  void SetSearchHeadingValidArray(
      const std::vector<std::pair<double, bool>>& heading_valid_array);
  const std::vector<std::pair<double, bool>>& GetSearchHeadingValidArray()
      const;

  // @brief: set module time cost
  void SetModuleTimeCost(
      const std::vector<std::pair<std::string, double>>& time_cost);
  std::vector<std::pair<std::string, double>> GetModuleTimeCost() const;

 private:
  // @brief: check multi-camera homography calib consistency
  void CheckMultiCameraHCalibConsistency();

 private:
  uint64_t id_;
  static std::atomic<uint64_t> next_id_;

  bool smm_success_ = false;

  // reference state estimated from other sensor
  NavState gt_state_;
  NavState gnss_state_;

  // localized pose state accociated main frame (car-center-frame)
  SE3d init_pose_state_;
  SE3d pose_state_;
  Eigen::Matrix<double, 6, 6> pose_cov_;  // covariance in vehicle coordinate
  NavState init_pose_state_correction_;   // from relocalization
  bool relocalization_mode_{false};

  // contains all synced frames, the 0-index is the main frame
  std::vector<std::shared_ptr<Frame>> frames_;

  // matched data in bird view for visualization
  std::vector<Point3D_t> matched_laneline_points_;
  std::vector<Point3D_t> matched_map_laneline_points_;
  std::vector<Point3D_t> matched_map_laneline_points_homo_calib_;
  std::vector<std::vector<std::pair<double, double>>> relocal_distribs_;

  // fov120 percept laneline polyfit param
  Eigen::Vector4d left_percept_line_param_ = Eigen::Vector4d::Zero();
  Eigen::Vector4d right_percept_line_param_ = Eigen::Vector4d::Zero();
  bool percept_line_param_valid_{false};

  // map laneline polyfit param
  std::vector<Eigen::Vector4d> map_line_params_;

  // search_y - heading valid
  std::vector<std::pair<double, bool>> heading_valid_array_;

  // different module process time cost in ms
  std::vector<std::pair<std::string, double>> time_cost_;
};

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
