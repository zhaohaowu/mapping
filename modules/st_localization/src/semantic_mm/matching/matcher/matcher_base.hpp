/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */

#pragma once

#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Sophus/se3.hpp>

#include "localization/data_type/base.hpp"
#include "localization/data_type/semantic_type.hpp"
#include "localization/data_type/smart_ptr.hpp"

namespace senseAD {
namespace localization {
namespace smm {

template <typename PerceptType, typename MapType>
class MatcherBase {
 public:
  DEFINE_SMART_PTR(MatcherBase)

  MatcherBase() = default;
  virtual ~MatcherBase() {}

  void SetPoseState(const SE3d& pose) { pose_state_ = pose; }

  void SetPerceptionData(const std::unordered_map<id_t, PerceptType>& data) {
    percept_data_ = data;
  }

  void SetMapData(
      const std::unordered_map<id_t, std::shared_ptr<MapType>>& data) {
    map_data_ = data;
  }

  // @brief: macthing core
  virtual adLocStatus_t Matching(bool relocalization_mode) = 0;

  std::vector<std::pair<id_t, id_t>> GetMatchingIndices() const {
    return matching_indices_;
  }

 protected:
  // perception frame pose in map coordinate
  SE3d pose_state_;

  // perception and map data
  std::unordered_map<id_t, PerceptType> percept_data_;
  std::unordered_map<id_t, std::shared_ptr<MapType>> map_data_;

  // matching results
  std::vector<std::pair<id_t, id_t>> matching_indices_;
};

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
