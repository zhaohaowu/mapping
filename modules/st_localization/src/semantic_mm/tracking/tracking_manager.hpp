/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Fang Yu <fangyu1@senseauto.com>
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
#include "semantic_mm/base/lane_line.hpp"
#include "semantic_mm/base/pole.hpp"
#include "semantic_mm/base/traffic_sign.hpp"
#include "semantic_mm/common/match_index.hpp"
#include "semantic_mm/common/math_tools.hpp"

namespace senseAD {
namespace localization {
namespace smm {

// modules
class FramePackage;
class MapManager;
class MatchingManager;

// element database
class ElementLaneLineDB;
class ElementTrafficSignDB;
class ElementPoleDB;

// class holds the tracking manager, execute temporal tracking and fusion task
// of perception data
class TrackingManager {
 public:
  DEFINE_SMART_PTR(TrackingManager)

  explicit TrackingManager(const LocalizationParam& param);
  ~TrackingManager() = default;

  // @brief: temporal tracking and fusion for frame package
  adLocStatus_t ProcessTracking(
      const std::shared_ptr<FramePackage>& frame_package,
      const std::shared_ptr<MapManager>& map_manager,
      const std::shared_ptr<MatchingManager>& matching_manager);

  // @brief: get fusion results
  const std::unordered_map<std::string, PerceptLaneLine::PtrUMap>&
  GetFusedLaneLines() const {
    return fused_lanelines_;
  }
  const std::unordered_map<std::string, PerceptTrafficSign::PtrUMap>&
  GetFusedSigns() const {
    return fused_signs_;
  }
  const std::unordered_map<std::string, PerceptPole::PtrUMap>& GetFusedPoles()
      const {
    return fused_poles_;
  }

  // @brief: set/get match indices between trackers and map
  void SetMatchIndex(const std::string& camera_name,
                     const MatchIndex::Ptr& match) {
    match_indices_[camera_name] = match;
  }
  void SetMatchIndex(
      const std::unordered_map<std::string, MatchIndex::Ptr>& match_indices) {
    match_indices_ = match_indices;
  }
  adLocStatus_t GetMatchIndex(const std::string& camera_name,
                              MatchIndex::Ptr* match) const {
    auto iter = match_indices_.find(camera_name);
    if (iter == match_indices_.end()) return LOC_OUT_OF_RANGE;
    *match = iter->second;
    return LOC_SUCCESS;
  }
  std::unordered_map<std::string, MatchIndex::Ptr> GetMatchIndex() const {
    return match_indices_;
  }

 private:
  //////////////////// Tracking for each semantic type //////////////////////
  adLocStatus_t TrackingLaneLine(
      const std::string& camera_name, const SE3d& frame_pose,
      const std::unordered_map<id_t, PerceptLaneLine>& percept_data,
      const LaneLine::PtrUMap& map_data,
      const std::vector<std::pair<id_t, id_t>>& match_indices,
      bool relocalization_mode);

  adLocStatus_t TrackingTrafficSign(
      const uint64_t timestamp, const std::string& camera_name,
      const SE3d& frame_pose,
      const std::unordered_map<id_t, PerceptTrafficSign>& percept_data,
      const TrafficSign::PtrUMap& map_data,
      const std::vector<std::pair<id_t, id_t>>& match_indices,
      bool relocalization_mode);

  adLocStatus_t TrackingPole(
      const uint64_t timestamp, const std::string& camera_name,
      const SE3d& frame_pose,
      const std::unordered_map<id_t, PerceptPole>& percept_data,
      const Pole::PtrUMap& map_data,
      const std::vector<std::pair<id_t, id_t>>& match_indices,
      bool relocalization_mode);

  // TODO(xx): support extended, tracking other semantic elements ...

 private:
  LocalizationParam param_;

  // laneline tracking related
  // databases
  std::unordered_map<std::string, std::shared_ptr<ElementLaneLineDB>>
      laneline_dbs_;
  std::unordered_map<std::string, std::shared_ptr<ElementTrafficSignDB>>
      sign_dbs_;
  std::unordered_map<std::string, std::shared_ptr<ElementPoleDB>> pole_dbs_;
  // fused elements
  std::unordered_map<std::string, PerceptLaneLine::PtrUMap> fused_lanelines_;
  std::unordered_map<std::string, PerceptTrafficSign::PtrUMap> fused_signs_;
  std::unordered_map<std::string, PerceptPole::PtrUMap> fused_poles_;

  // map matching indices for all semantics
  std::unordered_map<std::string, MatchIndex::Ptr> match_indices_;
};

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
