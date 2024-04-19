/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhu Kaiying <zhukaiying@sensetime.com>
 */

#include "semantic_mm/matching/matcher/matcher_traffic_sign.hpp"

#include <vector>

#include "localization/common/log.hpp"
#include "semantic_mm/common/match_tools.hpp"

namespace senseAD {
namespace localization {
namespace smm {

adLocStatus_t MatcherTrafficSign::Matching(bool relocalization_mode) {
  // preprocess
  if (PreProcess() != LOC_SUCCESS) return LOC_LOCALIZATION_ERROR;

  // matching strategy
  adLocStatus_t status = ProbabilityMatching();
  return status;
}

adLocStatus_t MatcherTrafficSign::PreProcess() {
  static std::pair<double, double> long_range{-20, 40};
  static std::pair<double, double> lateral_range{-20.0, 20.0};
  for (auto it = map_data_.begin(); it != map_data_.end(); ++it) {
    Point3D_t& pt = it->second->GetCenter();
    if (pt.x > long_range.first && pt.x < long_range.second &&
        pt.y > lateral_range.first && pt.y < lateral_range.second) {
      map_sign_points_[it->first] = pt;
      map_sign_point_covs_[it->first] =
          it->second->GetCenterCov().topLeftCorner(2, 2);
    }
  }
  for (auto it = percept_data_.begin(); it != percept_data_.end(); ++it) {
    Point3D_t& pt = it->second.processed_center;
    if (pt.x > long_range.first && pt.x < long_range.second &&
        pt.y > lateral_range.first && pt.y < lateral_range.second) {
      percept_sign_points_[it->first] = pt;
      percept_sign_point_covs_[it->first] =
          it->second.processed_center_cov.topLeftCorner(2, 2);
    }
  }
  return LOC_SUCCESS;
}

adLocStatus_t MatcherTrafficSign::ProbabilityMatching() {
  std::unordered_map<id_t, std::unordered_map<id_t, double>> pp_distances;
  if (CalMatchingManhattanDistance(
          percept_sign_points_, percept_sign_point_covs_, map_sign_points_,
          map_sign_point_covs_, &pp_distances) != LOC_SUCCESS) {
    LC_LERROR(MATCHING) << "calculate traffic sign mah distance failed.";
    return LOC_LOCALIZATION_ERROR;
  }
  std::vector<std::pair<id_t, id_t>> match_pairs;
  if (CalNearestMatchingPairs(pp_distances, 5.99, true, &match_pairs) !=
      LOC_SUCCESS) {
    LC_LERROR(MATCHING) << "calculate traffic sign matching pairs failed.";
    return LOC_LOCALIZATION_ERROR;
  }

  matching_indices_ = std::move(match_pairs);

  return LOC_SUCCESS;
}

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
