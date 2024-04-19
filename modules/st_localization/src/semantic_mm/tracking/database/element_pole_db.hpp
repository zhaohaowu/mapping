/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Fang Yu <fangyu1@senseauto.com>
 */

#pragma once

#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include "localization/data_type/base.hpp"
#include "localization/data_type/percept_data.hpp"
#include "localization/data_type/semantic_type.hpp"
#include "localization/data_type/smart_ptr.hpp"
#include "semantic_mm/base/pole.hpp"
#include "semantic_mm/common/camera_model.hpp"
#include "semantic_mm/common/math_tools.hpp"
#include "semantic_mm/tracking/database/element_db_base.hpp"
#include "semantic_mm/tracking/tracker/depth_tracker.hpp"

namespace senseAD {
namespace localization {
namespace smm {

class ElementPoleDB : public ElementDBBase<PerceptPole, Pole> {
 public:
  DEFINE_SMART_PTR(ElementPoleDB)

  ElementPoleDB() = default;
  ElementPoleDB(const CameraModel::Ptr& camera_model, const SE3d& T_veh_cam);
  ~ElementPoleDB() override {}

  // @brief: reset input/ouput data and trackers
  void Reset();

  // @brief: main tracking and fusion process
  adLocStatus_t TrackingAndFusion(bool map_aided) final;

  // @brief: get live tracker number for debug
  size_t GetTrackerNumber() const { return pole_trackers_.size(); }

 private:
  // @brief: get stable fused poles from trackers
  adLocStatus_t GetStablePoles();

  // @brief: update tracked data by new perception
  adLocStatus_t UpdateTrackedPerceptData(const id_t tracker_id,
                                         const PerceptPole& new_percept);

  // @brief: fusion between trackers with valid triangulated points
  adLocStatus_t BetweenTrackerFusion();

 private:
  CameraModel::Ptr camera_model_;
  SE3d T_veh_cam_;

  // pole trackers
  DepthTracker::PtrUMap pole_trackers_;

  // map from perception id to tracker id
  std::unordered_map<id_t, id_t> id_mapping_;

  // map from tracker id to perception id
  std::unordered_map<id_t, id_t> id_mapping_inverse_;

  // for object tracking, maintain latest perception results
  std::unordered_map<id_t, std::shared_ptr<PerceptPole>> tracked_pole_data_;
};

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
