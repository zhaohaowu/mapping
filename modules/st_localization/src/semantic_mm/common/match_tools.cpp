/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhu Kaiying <zhukaiying@sensetime.com>
 */

#include "semantic_mm/common/match_tools.hpp"

#include <limits>

#include "localization/common/log.hpp"
#include "semantic_mm/common/math_tools.hpp"

namespace senseAD {
namespace localization {
namespace smm {

adLocStatus_t CalMatchingDistance(
    const std::unordered_map<id_t, Point3D_t>& percept_data,
    const std::unordered_map<id_t, Point3D_t>& map_data,
    std::unordered_map<id_t, std::unordered_map<id_t, double>>* distances) {
  if (!distances) {
    LC_LERROR(MATCHING) << "input nullptr.";
    return LOC_NULL_PTR;
  }

  for (auto percept_it = percept_data.begin(); percept_it != percept_data.end();
       ++percept_it) {
    const auto percept_id = percept_it->first;
    const auto& percept_pt = percept_it->second;

    std::unordered_map<id_t, double> to_map_distance;
    for (auto map_it = map_data.begin(); map_it != map_data.end(); ++map_it) {
      const auto map_id = map_it->first;
      const auto& map_pt = map_it->second;
      double distance = (percept_pt - map_pt).Norm2D();
      to_map_distance.insert({map_id, distance});
    }
    if (!to_map_distance.empty()) {
      distances->insert({percept_id, std::move(to_map_distance)});
    }
  }

  return LOC_SUCCESS;
}

adLocStatus_t CalMatchingDistance(
    const std::unordered_map<id_t, std::vector<Point3D_t>>& percept_sets,
    const std::unordered_map<id_t, std::vector<Point3D_t>>& map_sets,
    std::unordered_map<id_t, std::unordered_map<id_t, double>>* distances) {
  if (!distances) {
    LC_LERROR(MATCHING) << "input nullptr.";
    return LOC_NULL_PTR;
  }

  for (auto it = percept_sets.begin(); it != percept_sets.end(); ++it) {
    const auto& percept_id = it->first;
    const auto& percept_pts = it->second;
    if (percept_pts.empty()) continue;

    (*distances).insert({percept_id, std::unordered_map<id_t, double>()});
    for (auto it2 = map_sets.begin(); it2 != map_sets.end(); ++it2) {
      const auto& map_id = it2->first;
      const auto& map_pts = it2->second;

      // calculate distance of percept and map line
      int valid_pt_num = 0;
      double ave_distance =
          Line2LineDistance2D(percept_pts, map_pts, &valid_pt_num);

      // check whether has enough matching point or enough valid ratio
      double valid_ratio =
          static_cast<double>(valid_pt_num) / percept_pts.size();
      if (valid_pt_num < 5 || valid_ratio < 0.3) {
        ave_distance = std::numeric_limits<double>::max();
      }
      (*distances)[percept_id].insert({map_id, ave_distance});
    }
  }

  return LOC_SUCCESS;
}

adLocStatus_t CalMatchingManhattanDistance(
    const std::unordered_map<id_t, Point3D_t>& percept_data,
    const std::unordered_map<id_t, Eigen::Matrix2d>& percept_data_cov,
    const std::unordered_map<id_t, Point3D_t>& map_data,
    const std::unordered_map<id_t, Eigen::Matrix2d>& map_data_cov,
    std::unordered_map<id_t, std::unordered_map<id_t, double>>* distances) {
  if (!distances || percept_data.size() != percept_data_cov.size() ||
      map_data.size() != map_data_cov.size()) {
    LC_LERROR(MATCHING) << "invalid param input.";
    return LOC_NULL_PTR;
  }

  for (auto percept_it = percept_data.begin(); percept_it != percept_data.end();
       ++percept_it) {
    const auto percept_id = percept_it->first;
    const auto& percept_pt = percept_it->second;
    const auto percept_pt_cov_it = percept_data_cov.find(percept_id);
    if (percept_pt_cov_it == percept_data_cov.end()) continue;

    std::unordered_map<id_t, double> to_map_distance;
    for (auto map_it = map_data.begin(); map_it != map_data.end(); ++map_it) {
      const auto map_id = map_it->first;
      const auto& map_pt = map_it->second;
      const auto map_pt_cov_it = map_data_cov.find(map_id);
      if (map_pt_cov_it == map_data_cov.end()) continue;
      Eigen::Matrix2d match_cov =
          percept_pt_cov_it->second + map_pt_cov_it->second;
      double distance =
          Point2PointManhattanDist2D(percept_pt, map_pt, match_cov.inverse());
      to_map_distance.insert({map_id, distance});
    }
    if (!to_map_distance.empty()) {
      distances->insert({percept_id, std::move(to_map_distance)});
    }
  }

  return LOC_SUCCESS;
}

adLocStatus_t CalMatchingManhattanDistance(
    const std::unordered_map<id_t, std::vector<Point3D_t>>& percept_sets,
    const std::unordered_map<id_t, std::vector<Eigen::Matrix2d>>&
        percept_set_covs,
    const std::unordered_map<id_t, std::vector<Point3D_t>>& map_sets,
    std::unordered_map<id_t, std::unordered_map<id_t, double>>* distances) {
  if (!distances || percept_sets.size() != percept_set_covs.size()) {
    LC_LERROR(MATCHING) << "invalid param input.";
    return LOC_NULL_PTR;
  }

  for (auto it = percept_sets.begin(); it != percept_sets.end(); ++it) {
    const auto& percept_id = it->first;
    const auto& percept_pts = it->second;
    if (percept_pts.empty()) continue;
    if (!percept_set_covs.count(percept_id)) continue;
    const auto& percept_pt_cov = percept_set_covs.at(percept_id);

    (*distances).insert({percept_id, std::unordered_map<id_t, double>()});
    for (auto it2 = map_sets.begin(); it2 != map_sets.end(); ++it2) {
      const auto& map_id = it2->first;
      const auto& map_pts = it2->second;
      if (map_pts.empty()) continue;

      int valid_pt_num = 0;
      double ave_distance = Line2LineManhattanDistance2D(
          percept_pts, percept_pt_cov, map_pts, &valid_pt_num);

      // check whether has enough matching point or enough valid ratio
      double valid_ratio =
          static_cast<double>(valid_pt_num) / percept_pts.size();
      if (valid_pt_num < 5 || valid_ratio < 0.3) {
        ave_distance = std::numeric_limits<double>::max();
      }
      (*distances)[percept_id].insert({map_id, ave_distance});
    }
  }

  return LOC_SUCCESS;
}

adLocStatus_t CalNearestMatchingPairs(
    const std::unordered_map<id_t, std::unordered_map<id_t, double>>& distances,
    double distance_thre, bool unique_match,
    std::vector<std::pair<id_t, id_t>>* matching_pairs) {
  if (!matching_pairs) {
    LC_LERROR(MATCHING) << "input nullptr.";
    return LOC_NULL_PTR;
  }
  matching_pairs->clear();

  if (distances.empty()) return LOC_SUCCESS;

  std::vector<std::pair<id_t, id_t>> temp_matching_pairs;
  temp_matching_pairs.reserve(distances.size());
  std::unordered_map<id_t, std::pair<id_t, double>> reverse_distances;
  reverse_distances.reserve(distances.size());
  for (auto it = distances.begin(); it != distances.end(); ++it) {
    const id_t percept_id = it->first;

    // search the matching map id with the nearest distance
    id_t min_map_id = 0;
    double min_dist = std::numeric_limits<double>::max();
    double snd_min_dist = std::numeric_limits<double>::max();
    for (auto it2 = it->second.begin(); it2 != it->second.end(); ++it2) {
      if (it2->second < min_dist) {
        snd_min_dist = min_dist;
        min_dist = it2->second;
        min_map_id = it2->first;
      } else if (it2->second < snd_min_dist) {
        snd_min_dist = it2->second;
      }
    }
    // check whether close enough
    if (min_dist > distance_thre) continue;
    // check whether has unique closed, now has redundant map laneline
    // if (min_dist / snd_min_dist > 0.3) continue;
    temp_matching_pairs.emplace_back(percept_id, min_map_id);

    if (!reverse_distances.count(min_map_id) ||
        min_dist < reverse_distances[min_map_id].second) {
      reverse_distances[min_map_id] = std::make_pair(percept_id, min_dist);
    }
  }

  if (unique_match) {
    // reverse check, make sure injective matching(one-to-one)
    for (const auto& pair : reverse_distances) {
      id_t map_id = pair.first;
      id_t percept_id = pair.second.first;
      matching_pairs->emplace_back(percept_id, map_id);
      LC_LDEBUG(MATCHING) << "percept " << percept_id << " -> map " << map_id;
    }
  } else {
    *matching_pairs = std::move(temp_matching_pairs);
  }

  return LOC_SUCCESS;
}

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
