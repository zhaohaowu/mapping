/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */

#pragma once

#include <Eigen/Core>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Sophus/se3.hpp>

#include "eval/evaluator_smm.hpp"
#include "localization/data_type/base.hpp"
#include "localization/data_type/percept_data.hpp"
#include "localization/data_type/smart_ptr.hpp"
#include "semantic_mm/common/match_index.hpp"

namespace senseAD {
namespace localization {
namespace smm {

// class holds the frame, contains perception data, pose state, preprocess, and
// related query API interface
class Frame {
 public:
  DEFINE_SMART_PTR(Frame);

  Frame() = default;
  Frame(uint64_t timestamp, const std::string& camera_name,
        const NavState& nav_state, const OdomState& odom_state,
        const PerceptData::Ptr& percept_data);
  ~Frame() = default;

  // @brief: set and get interface
  void SetTimestamp(uint64_t timestamp);
  uint64_t GetTimestamp() const;

  void SetCameraName(const std::string& camera_name);
  std::string GetCameraName() const;

  void SetNavState(const NavState& nav_state);
  NavState GetNavState() const;

  void SetOdomState(const OdomState& odom_state);
  OdomState GetOdomState() const;

  void SetPerceptionData(const PerceptData::Ptr& percept_data);
  PerceptData::Ptr GetPerceptionData() const;

  void SetVanishingPoint(const Eigen::Vector2d& p);
  Eigen::Vector2d GetVanishingPoint() const;

  void SetHomoPitch(double pitch);
  double GetHomoPitch() const;
  Eigen::Matrix3d GetAdaptedHomography() const;

  void SetOptimHomoChi2Errors(std::pair<double, double> chi2_errors);
  std::pair<double, double> GetOptimHomoChi2Errors() const;

  void SetMatchIndex(const MatchIndex::Ptr& match);
  MatchIndex::Ptr GetMatchIndex() const;

  // @brief: preprocess for frame perception data
  adLocStatus_t PreProcess();

  // @brief: check quality of perception data in bird view
  adLocStatus_t CheckPerceptionData(SMMEvalData* eval_data) const;

 public:
  // @brief: static preprocess algorithm for base data structure
  static adLocStatus_t ProjectLaneLineBVPoints(
      const Eigen::Matrix3d& H, const NavState& nav_state,
      double max_longitudinal_range, double sample_dist,
      double homo_proj_cov_coeff,
      std::unordered_map<id_t, PerceptLaneLine>* percept_lls);

 private:
  // @brief: preprocess for perception laneline
  adLocStatus_t PreProcessLaneLine();

  adLocStatus_t CalibHomographyPitchByVanishingPoint();

  adLocStatus_t EstimateLaneLineVanishingPoint();

  // TODO(xx): support extended, other semantic elements ...

 private:
  uint64_t timestamp_;
  std::string camera_name_;

  // predicted/refined nav state associated this frame (car-center-frame)
  NavState nav_state_;
  // odom state from dead reckoning (car-center-frame)
  OdomState odom_state_;

  // raw/preprocessed perception results
  PerceptData::Ptr perception_data_;

  // adapted homography
  Eigen::Matrix3d H_base_, K_inv_;
  Eigen::Matrix3d H_adapted_ = Eigen::Matrix3d::Zero();
  double h_pitch_{0};
  // homography optim chi2 mean/max error
  std::pair<double, double> chi2_errors_;

  // estimated vanishing point
  Eigen::Vector2d vanishing_point_;

  // map matching related
  MatchIndex::Ptr match_indices_{nullptr};
};

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
