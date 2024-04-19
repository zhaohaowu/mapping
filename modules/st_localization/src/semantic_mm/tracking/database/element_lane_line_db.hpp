/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Fang Yu <fangyu1@senseauto.com>
 */

#pragma once

#include <list>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "localization/data_type/base.hpp"
#include "localization/data_type/percept_data.hpp"
#include "localization/data_type/road_structure.hpp"
#include "localization/data_type/semantic_type.hpp"
#include "localization/data_type/smart_ptr.hpp"
#include "semantic_mm/base/lane_line.hpp"
#include "semantic_mm/common/math_tools.hpp"
#include "semantic_mm/frame/frame.hpp"
#include "semantic_mm/frame/frame_package.hpp"
#include "semantic_mm/map/map_manager.hpp"
#include "semantic_mm/matching/matching_manager.hpp"
#include "semantic_mm/tracking/database/element_db_base.hpp"
#include "semantic_mm/tracking/tracker/lane_line_tracker.hpp"

namespace senseAD {
namespace localization {
namespace smm {

class ElementLaneLineDB : public ElementDBBase<PerceptLaneLine, LaneLine> {
 public:
  DEFINE_SMART_PTR(ElementLaneLineDB)

  ElementLaneLineDB() = default;
  virtual ~ElementLaneLineDB() = default;

  // @brief: reset input/ouput data and trackers
  void Reset();

  // @brief: main tracking and fusion process
  adLocStatus_t TrackingAndFusion(bool map_aided) final;

  // @brief: get live tracker number for debug
  size_t GetTrackerNumber() const { return ll_trackers_.size(); }

 private:
  // @brief: track laneline perception by map matching
  adLocStatus_t MapAidedTrackAndFusion(
      std::unordered_set<id_t>* new_tracker_ids);

  // @brief: track laneline perception by temporal matching
  adLocStatus_t TrackAndFusion(std::unordered_set<id_t>* new_tracker_ids);

  // @brief: fusion between newly created and old trackers
  adLocStatus_t BetweenTrackerFusion(
      const std::unordered_set<id_t>& new_tracker_ids);

  // @brief: rematch lost trackers to map
  adLocStatus_t RematchLostTrackerToMap();

  // @brief: get stable fused lanelines from trackers
  adLocStatus_t GetStableLaneLines(bool map_aided);

 private:
  // laneline trackers
  LaneLineTracker::PtrUMap ll_trackers_;

  SE3d last_frame_pose_;
};

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
