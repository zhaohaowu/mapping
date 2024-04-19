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

#include <Sophus/se3.hpp>

#include "localization/data_type/base.hpp"
#include "localization/data_type/percept_data.hpp"
#include "localization/localization_param.hpp"
#include "semantic_mm/common/camera_model.hpp"
#include "semantic_mm/tracking/tracker/depth_tracker.hpp"

namespace senseAD {
namespace localization {
namespace smm {

class LaneLine;
class Pole;
class TrafficSign;
class FramePackage;
class TrackingManager;
class MapManager;
class MatcherMultiSemanticGrid;

// class holds the matching manager, execute matching task and cache matching
// results for reuse
class MatchingManager {
 public:
  DEFINE_SMART_PTR(MatchingManager)

  MatchingManager();
  ~MatchingManager() = default;

  // @brief: map matching for frame package
  adLocStatus_t ProcessMatchingFrmpkg(
      const std::shared_ptr<FramePackage>& frame_package,
      const std::shared_ptr<MapManager>& map_manager);

  // @brief: map matching for tracker
  adLocStatus_t ProcessMatchingTracker(
      const std::shared_ptr<FramePackage>& frame_package,
      const std::shared_ptr<TrackingManager>& tracking_manager,
      const std::shared_ptr<MapManager>& map_manager);

 public:
  // @brief: static matching algorithm for base data structure
  static adLocStatus_t MatchingLaneLine(
      const SE3d& pose,
      const std::unordered_map<id_t, PerceptLaneLine>& percept_data,
      const std::unordered_map<id_t, std::shared_ptr<LaneLine>>& map_data,
      bool relocalization_mode,
      std::vector<std::pair<id_t, id_t>>* matching_indices);

  static adLocStatus_t MatchingPole(
      const std::unordered_map<id_t, std::shared_ptr<PerceptPole>>&
          percept_data,
      const std::unordered_map<id_t, std::shared_ptr<Pole>>& map_data,
      bool relocalization_mode,
      std::vector<std::pair<id_t, id_t>>* matching_indices);

  static adLocStatus_t MatchingTrafficSign(
      const std::unordered_map<id_t, std::shared_ptr<PerceptTrafficSign>>&
          percept_data,
      const std::unordered_map<id_t, std::shared_ptr<TrafficSign>>& map_data,
      bool relocalization_mode,
      std::vector<std::pair<id_t, id_t>>* matching_indices);

  static adLocStatus_t MatchingTrafficSignBbox(
      const uint64_t timestamp, const SE3d& pose,
      const std::unordered_map<id_t, PerceptTrafficSign>& percept_data,
      const std::unordered_map<id_t, std::shared_ptr<PerceptTrafficSign>>&
          tracking_data,
      const DepthTracker::PtrUMap& trackers,
      const std::shared_ptr<CameraModel>& camera_model,
      std::unordered_map<id_t, id_t>* matching_indices);

  static adLocStatus_t MatchingPoleBbox(
      const uint64_t timestamp, const SE3d& pose,
      const std::unordered_map<id_t, PerceptPole>& percept_data,
      const std::unordered_map<id_t, std::shared_ptr<PerceptPole>>&
          tracking_data,
      const DepthTracker::PtrUMap& trackers,
      const std::shared_ptr<CameraModel>& camera_model,
      std::unordered_map<id_t, id_t>* matching_indices);

  // TODO(xx): support extended, matching other semantic elements ...

 private:
  // @brief: map matching for tracker in localization mode
  adLocStatus_t MatchingTrackerLocalizationMode(
      const std::shared_ptr<TrackingManager>& tracking_manager,
      const std::shared_ptr<MapManager>& map_manager);

  // @brief: map matching for tracker in relocalization mode
  adLocStatus_t MatchingTrackerRelocalizationMode(
      const std::shared_ptr<TrackingManager>& tracking_manager,
      const std::shared_ptr<MapManager>& map_manager);
  adLocStatus_t TrackerRelocalizationGridMatch(
      const std::shared_ptr<FramePackage>& frame_package,
      const std::shared_ptr<TrackingManager>& tracking_manager,
      const std::shared_ptr<MapManager>& map_manager);
  adLocStatus_t TrackerRelocalizationObjectMatch(
      const std::shared_ptr<FramePackage>& frame_package,
      const std::shared_ptr<TrackingManager>& tracking_manager,
      const std::shared_ptr<MapManager>& map_manager);

 private:
  // instance for relocalization
  std::shared_ptr<MatcherMultiSemanticGrid> grid_match_;
};

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
