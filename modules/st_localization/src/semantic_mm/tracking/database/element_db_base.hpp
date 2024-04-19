/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Fang Yu <fangyu1@senseauto.com>
 */

#pragma once

#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Sophus/se3.hpp>

#include "localization/data_type/base.hpp"
#include "localization/data_type/semantic_type.hpp"

namespace senseAD {
namespace localization {
namespace smm {

template <typename PerceptType, typename MapType>
class ElementDBBase {
 public:
  DEFINE_SMART_PTR(ElementDBBase)

  ElementDBBase() = default;
  virtual ~ElementDBBase() {}

  // @brief: set/get interface
  void SetTimestamp(uint64_t timestamp) { timestamp_ = timestamp; }

  void SetPoseState(const SE3d& pose) { frame_pose_ = pose; }

  void SetPerceptionData(const std::unordered_map<id_t, PerceptType>& data) {
    percept_data_ = data;  // TODO(xxx): save copy of perception data
  }

  void SetMapData(
      const std::unordered_map<id_t, std::shared_ptr<MapType>>& data) {
    map_data_ = data;
  }

  void SetMatchingIndecies(const std::vector<std::pair<id_t, id_t>>& data) {
    matching_indices_ = data;
  }

  const std::unordered_map<id_t, std::shared_ptr<PerceptType>>&
  GetFusedPerceptionData() {
    return fused_local_percept_data_;
  }

  // @brief: fusion core
  virtual adLocStatus_t TrackingAndFusion(bool map_aided) = 0;

 protected:
  uint64_t timestamp_{0};

  // frame pose and original perception data
  SE3d frame_pose_;
  std::unordered_map<id_t, PerceptType> percept_data_;

  // data for map-aided tracking, may not be set
  std::unordered_map<id_t, std::shared_ptr<MapType>> map_data_;
  std::vector<std::pair<id_t, id_t>> matching_indices_;

  // temporal-fused perception data in local query frame
  std::unordered_map<id_t, std::shared_ptr<PerceptType>>
      fused_local_percept_data_;
};

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
