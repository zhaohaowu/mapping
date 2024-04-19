/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "factor_optimizer/factor_optimizer.hpp"
#include "localization/data_type/base.hpp"
#include "localization/data_type/percept_data.hpp"

namespace senseAD {
namespace localization {
namespace smm {

class LaneLine;
class Pole;
class TrafficSign;
class FramePackage;
class MapManager;
class MatchingManager;
class TrackingManager;
class MatchIndex;

// class holds the optimization manager, build optimization task and optimize it
class OptimManager {
 public:
  ~OptimManager() {}
  static OptimManager* GetInstance();

  // @brief: optimize pose, using laneline constrain, matching in space
  adLocStatus_t OptimizePoseOnlyLaneLineInSpace(
      const std::shared_ptr<FramePackage>& frame_package,
      const std::shared_ptr<MapManager>& map_manager,
      const std::shared_ptr<MatchingManager>& matching_manager,
      const std::shared_ptr<TrackingManager>& tracking_manager);

  // TODO(xx): support extended, optimize pose using other methods ...

  // @brief: optimize homography, using laneline constrain, matching in space
  adLocStatus_t OptimizeHomographyOnlyLaneLineInSpace(
      const std::shared_ptr<FramePackage>& frame_package,
      const std::shared_ptr<MapManager>& map_manager,
      const std::shared_ptr<MatchingManager>& matching_manager);

 private:
  // struct holds point-to-point correlation
  struct P2PCorr {
    id_t id_map;          // id in map
    SemanticType type;    // semantic type
    Point3D_t p;          // percept point
    Point3D_t p_map;      // map point
    Eigen::Matrix2d cov;  // percept point cov
    double error = 0.0;   // point-point euclidean distance, negtive if
                          // percept point is at right of map points
    bool inlier{true};    // whether is inlier
  };

  // struct holds point-to-line correlation
  struct P2LCorr {
    id_t id_map;            // id in map
    LineType type;          // line type
    Point3D_t p;            // percept point
    Point3D_t p_map_s;      // map line start
    Point3D_t p_map_e;      // map line end
    Point3D_t opt_p_map_s;  // optimized map line start
    Point3D_t opt_p_map_e;  // optimized map line end
    Eigen::Matrix2d cov;    // percept point cov
    double error = 0.0;     // point-line euclidean distance, negtive if
                            // percept point is at right of map points
    bool inlier{true};      // whether is inlier
  };

  // @brief: find point-to-line associations btw perception and map lanelines
  void FindLaneLineP2LAssociations(
      const PerceptLaneLine::PtrUMap& percept_lls,
      const std::unordered_map<id_t, std::shared_ptr<LaneLine>>& map_lls,
      const std::vector<std::pair<id_t, id_t>>& match_pairs,
      const SE3d& Tmap_percept, std::vector<P2LCorr>* p2l_corrs,
      bool in_image_space = false);

  adLocStatus_t FindPoleP2PAssociations(
      const std::unordered_map<std::string, PerceptPole::PtrUMap>&
          multi_cam_percept_poles,
      const std::unordered_map<id_t, std::shared_ptr<Pole>>& map_poles,
      const std::unordered_map<std::string, std::shared_ptr<MatchIndex>>&
          multi_cam_match_index,
      std::vector<P2PCorr>* p2p_corrs) const;

  adLocStatus_t FindTrafficSignP2PAssociations(
      const std::unordered_map<std::string, PerceptTrafficSign::PtrUMap>&
          multi_cam_percept_signs,
      const std::unordered_map<id_t, std::shared_ptr<TrafficSign>>& map_signs,
      const std::unordered_map<std::string, std::shared_ptr<MatchIndex>>&
          multi_cam_match_index,
      std::vector<P2PCorr>* p2p_corrs) const;

  // @brief: estimate SMM covariance
  adLocStatus_t EstimateSMMCovariance(
      const FactorOptimizer::Ptr& optimizer,
      const std::vector<P2LCorr>& laneline_p2l_corrs,
      const std::vector<P2PCorr>& pole_p2p_corrs,
      const std::vector<P2PCorr>& sign_p2p_corrs,
      Eigen::Matrix<double, 6, 6>* pose_cov);

 private:
  // @brief: disable construction
  OptimManager() = default;
  OptimManager(const OptimManager&) = delete;
  OptimManager& operator=(const OptimManager&) = delete;

  // chi-square table
  double chi2_2d_[5] = {4.605, 5.991, 7.378, 9.210, 10.597};
};

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
