/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */

#include "semantic_mm/matching/matcher/matcher_lane_line.hpp"

#include <map>

#include "localization/common/log.hpp"
#include "semantic_mm/common/configure.hpp"
#include "semantic_mm/common/match_tools.hpp"
#include "semantic_mm/common/math_tools.hpp"

namespace senseAD {
namespace localization {
namespace smm {

adLocStatus_t MatcherLaneLine::Matching(bool relocalization_mode) {
  // preprocess
  if (PreProcess() != LOC_SUCCESS) return LOC_LOCALIZATION_ERROR;

  // matching strategy
  if (!relocalization_mode) {
    // tracking mode: probability matching for laneline and roadside
    if (ProbabilityMatching() != LOC_SUCCESS) return LOC_LOCALIZATION_ERROR;
  } else {
    // relocalization mode: matching roadside first, then laneline
    LC_LDEBUG(MATCHING) << "SMM relocalization matching ...";
    auto status = NearestMatchingForRoadSide();
    if (status == LOC_SUCCESS) {
      // inorder matching for laneline, roadside aided
      InOrderMatchingRoadSideAidedForLaneLine();
      LC_LDEBUG(MATCHING) << "relocalization success.";
    } else {
      LC_LERROR(MATCHING)
          << "matching for roadside failed, relocalization fail.";
      return LOC_LOCALIZATION_ERROR;
    }
  }

  return LOC_SUCCESS;
}

adLocStatus_t MatcherLaneLine::PreProcess() {
  // distinguish the laneline and roadside datas for map and perception
  for (auto it = map_data_.begin(); it != map_data_.end(); ++it) {
    LineType line_type = it->second->GetLineSegments().front()->GetLineType();
    if (IsLaneMarkingLine(line_type)) {
      map_laneline_points_.insert(std::make_pair(
          it->first, it->second->GetLineSegments().front()->GetPoints()));
    } else if (IsRoadSideLine(line_type)) {
      map_roadside_points_.insert(std::make_pair(
          it->first, it->second->GetLineSegments().front()->GetPoints()));
    }
  }
  for (auto it = percept_data_.begin(); it != percept_data_.end(); ++it) {
    if (it->second.line_type == LineType::LaneMarking) {
      percept_laneline_points_.insert(
          std::make_pair(it->first, it->second.processed_bv_points));
      percept_laneline_point_covs_.insert(
          std::make_pair(it->first, it->second.processed_bv_point_covs));
    } else if (it->second.line_type == LineType::Curb) {
      percept_roadside_points_.insert(
          std::make_pair(it->first, it->second.processed_bv_points));
      percept_roadside_point_covs_.insert(
          std::make_pair(it->first, it->second.processed_bv_point_covs));
    }
  }

  return LOC_SUCCESS;
}

adLocStatus_t MatcherLaneLine::ProbabilityMatching() {
  // calculate matching distance (percept line to map line)
  std::unordered_map<id_t, std::unordered_map<id_t, double>> ll_distances,
      rs_distances;

  if (CalMatchingManhattanDistance(
          percept_laneline_points_, percept_laneline_point_covs_,
          map_laneline_points_, &ll_distances) != LOC_SUCCESS) {
    LC_LERROR(MATCHING) << "calculate laneline mah distance failed.";
    return LOC_LOCALIZATION_ERROR;
  }
  if (CalMatchingManhattanDistance(
          percept_roadside_points_, percept_roadside_point_covs_,
          map_roadside_points_, &rs_distances) != LOC_SUCCESS) {
    LC_LERROR(MATCHING) << "calculate roadside mah distance failed.";
    return LOC_LOCALIZATION_ERROR;
  }

  // calculate nearest matching pairs, pair: <percept id, map id>
  std::vector<std::pair<id_t, id_t>> ll_pairs, rs_pairs;

  if (CalNearestMatchingPairs(ll_distances, 5.99, true, &ll_pairs) !=
      LOC_SUCCESS) {
    LC_LERROR(MATCHING) << "calculate laneline matching pairs failed.";
    return LOC_LOCALIZATION_ERROR;
  }
  if (CalNearestMatchingPairs(rs_distances, 5.99, true, &rs_pairs) !=
      LOC_SUCCESS) {
    LC_LERROR(MATCHING) << "calculate roadside matching pairs failed.";
    return LOC_LOCALIZATION_ERROR;
  }

  matching_indices_ = std::move(ll_pairs);
  matching_indices_.insert(matching_indices_.end(), rs_pairs.begin(),
                           rs_pairs.end());
  LC_LDEBUG(MATCHING) << "found laneline matching size: "
                      << matching_indices_.size();
  if (matching_indices_.empty()) return LOC_LOCALIZATION_ERROR;

  return LOC_SUCCESS;
}

adLocStatus_t MatcherLaneLine::NearestMatchingForRoadSide() {
  if (percept_roadside_points_.size() < 2 || map_roadside_points_.size() < 2) {
    LC_LERROR(MATCHING) << "percept or map has less roadside number.";
    return LOC_LOCALIZATION_ERROR;
  }

  // sort the lanelines for percept and map
  std::vector<id_t> percept_sorted_ids;
  SortLineByLateral(percept_roadside_points_, &percept_sorted_ids);
  std::vector<id_t> map_sorted_ids;
  SortLineByLateral(map_roadside_points_, &map_sorted_ids);

  // found the percep and map left and right bound roadside id
  id_t pleft_id = -1, pright_id = -1;
  for (const auto& id : percept_sorted_ids) {
    if (percept_roadside_points_[id].front().y < 0) pright_id = id;
    if (percept_roadside_points_[id].front().y > 0) {
      pleft_id = id;
      break;
    }
  }
  if (pleft_id == -1 || pright_id == -1) {
    LC_LERROR(MATCHING) << "percept roadside not found satisfied id";
    return LOC_LOCALIZATION_ERROR;
  }

  id_t mleft_id = -1, mright_id = -1;
  for (const auto& id : map_sorted_ids) {
    if (map_roadside_points_[id].front().y < 0) mright_id = id;
    if (map_roadside_points_[id].front().y > 0) {
      mleft_id = id;
      break;
    }
  }
  if (mleft_id == -1 || mright_id == -1) {
    LC_LERROR(MATCHING) << "map roadside not found satisfied id";
    return LOC_LOCALIZATION_ERROR;
  }

  // adaptive search distance for roadside matching
  int p_valid_num = 0;
  double p_distance =
      Line2LineDistance2D(percept_roadside_points_[pleft_id],
                          percept_roadside_points_[pright_id], &p_valid_num);
  int m_valid_num = 0;
  double m_distance =
      Line2LineDistance2D(map_roadside_points_[mleft_id],
                          map_roadside_points_[mright_id], &m_valid_num);
  if (p_valid_num == 0 || m_valid_num == 0) {
    LC_LERROR(MATCHING) << "not valid points to calculate roadside distance";
    return LOC_LOCALIZATION_ERROR;
  }

  if (std::fabs(p_distance - m_distance) > 1.0) {
    LC_LERROR(MATCHING) << "roadside distance not satisfy close enough.";
    return LOC_LOCALIZATION_ERROR;
  }

  double adaptive_search_dist = (p_distance + m_distance) / 6.0;
  LC_LDEBUG(MATCHING) << "SMM roadside search dist: " << adaptive_search_dist;

  // calculate roadside matching distance
  std::unordered_map<id_t, std::unordered_map<id_t, double>> rs_distances;
  if (CalMatchingDistance(percept_roadside_points_, map_roadside_points_,
                          &rs_distances) != LOC_SUCCESS) {
    LC_LERROR(MATCHING) << "calculate laneline distance failed.";
    return LOC_LOCALIZATION_ERROR;
  }

  // calculate nearest matching pairs
  std::vector<std::pair<id_t, id_t>> rs_pairs;
  if (CalNearestMatchingPairs(rs_distances, adaptive_search_dist, true,
                              &rs_pairs) != LOC_SUCCESS) {
    LC_LERROR(MATCHING) << "calculate laneline matching pairs failed.";
    return LOC_LOCALIZATION_ERROR;
  }

  LC_LDEBUG(MATCHING) << "found roadside matching size: " << rs_pairs.size();
  matching_indices_ = std::move(rs_pairs);
  if (matching_indices_.size() < 2) return LOC_LOCALIZATION_ERROR;

  return LOC_SUCCESS;
}

adLocStatus_t MatcherLaneLine::InOrderMatchingRoadSideAidedForLaneLine() {
  if (matching_indices_.empty()) return LOC_LOCALIZATION_ERROR;

  // sort the lanelines for percept and map
  std::vector<id_t> percept_sorted_ids;
  SortLineByLateral(percept_laneline_points_, &percept_sorted_ids);
  std::vector<id_t> map_sorted_ids;
  SortLineByLateral(map_laneline_points_, &map_sorted_ids);

  // in-order matching
  std::vector<std::pair<id_t, id_t>> matching_ll_pairs;
  matching_ll_pairs.reserve(percept_laneline_points_.size());
  for (size_t i = 0; i < matching_indices_.size(); ++i) {
    id_t percept_rs_id = matching_indices_[i].first;
    id_t map_rs_id = matching_indices_[i].second;

    if (!percept_data_.count(percept_rs_id) || !map_data_.count(map_rs_id)) {
      LC_LERROR(MTACHING) << "why not exist the roadside id." << percept_rs_id
                          << " " << map_rs_id;
      continue;
    }
    id_t percept_ll_id = percept_data_.at(percept_rs_id).linked_id;
    id_t map_ll_id = map_data_.at(map_rs_id)->GetLinkedId();

    // linked laneline directly matching
    if (percept_ll_id == -1 || map_ll_id == -1) {
      LC_LERROR(MTACHING) << "why not has the linked relation." << percept_ll_id
                          << " " << map_ll_id;
      continue;
    }

    // found the index in sorted contain for linked laneline id
    auto p_it = std::find(percept_sorted_ids.begin(), percept_sorted_ids.end(),
                          percept_ll_id);
    auto m_it =
        std::find(map_sorted_ids.begin(), map_sorted_ids.end(), map_ll_id);
    if (p_it == percept_sorted_ids.end() || m_it == map_sorted_ids.end()) {
      LC_LERROR(MTACHING) << "why not found the index.";
      continue;
    }
    int64_t pll_idx = p_it - percept_sorted_ids.begin();
    int64_t mll_idx = m_it - map_sorted_ids.begin();
    matching_ll_pairs.emplace_back(percept_ll_id, map_ll_id);

    // check the roadside position w.r.t vehicle
    bool is_left = map_roadside_points_.at(map_rs_id).front().y > 0;
    if (is_left) {
      // search from left to right
      for (int64_t pi = pll_idx - 1, mi = mll_idx - 1; pi >= 0 && mi >= 0;
           --pi, --mi) {
        double map_dist =
            Line2LineDistance2D(map_laneline_points_.at(map_sorted_ids[mi + 1]),
                                map_laneline_points_.at(map_sorted_ids[mi]));
        double percept_dist = Line2LineDistance2D(
            percept_laneline_points_.at(percept_sorted_ids[pi + 1]),
            percept_laneline_points_.at(percept_sorted_ids[pi]));
        // check line delta distance if close enough
        if (std::fabs(map_dist - percept_dist) > 1.0) break;
        // check if has exist
        std::pair<id_t, id_t> ll_pair(percept_sorted_ids[pi],
                                      map_sorted_ids[mi]);
        auto it = std::find(matching_ll_pairs.begin(), matching_ll_pairs.end(),
                            ll_pair);
        if (it != matching_ll_pairs.end()) break;
        // matched !
        matching_ll_pairs.emplace_back(ll_pair);
      }
    } else {
      // search from right to left
      for (int64_t pi = pll_idx + 1, mi = mll_idx + 1;
           pi < percept_sorted_ids.size() && mi < map_sorted_ids.size();
           ++pi, ++mi) {
        double map_dist =
            Line2LineDistance2D(map_laneline_points_.at(map_sorted_ids[mi - 1]),
                                map_laneline_points_.at(map_sorted_ids[mi]));
        double percept_dist = Line2LineDistance2D(
            percept_laneline_points_.at(percept_sorted_ids[pi - 1]),
            percept_laneline_points_.at(percept_sorted_ids[pi]));
        // check line delta distance if close enough
        if (std::fabs(map_dist - percept_dist) > 1.0) break;
        // check if has exist
        std::pair<id_t, id_t> ll_pair(percept_sorted_ids[pi],
                                      map_sorted_ids[mi]);
        auto it = std::find(matching_ll_pairs.begin(), matching_ll_pairs.end(),
                            ll_pair);
        if (it != matching_ll_pairs.end()) break;
        // matched !
        matching_ll_pairs.emplace_back(ll_pair);
      }
    }
  }

  LC_LDEBUG(MATCHING) << "found roadside aided laneline matching size: "
                      << matching_ll_pairs.size();
  if (matching_ll_pairs.empty()) return LOC_LOCALIZATION_ERROR;
  matching_indices_.insert(matching_indices_.end(), matching_ll_pairs.begin(),
                           matching_ll_pairs.end());

  return LOC_SUCCESS;
}

adLocStatus_t MatcherLaneLine::ConvertPerceptLineToWorld(
    std::unordered_map<id_t, std::vector<Point3D_t>>* converted_lines) {
  if (!converted_lines) {
    LC_LERROR(MATCHING) << "input nullptr.";
    return LOC_NULL_PTR;
  }

  for (auto it = percept_data_.begin(); it != percept_data_.end(); ++it) {
    id_t id = it->first;
    const auto& line = it->second;
    const auto& points = line.processed_bv_points;

    // transform to world coordinate
    auto trans_points = TransformPoints(points, pose_state_);
    converted_lines->insert(std::make_pair(id, std::move(trans_points)));
  }

  return LOC_SUCCESS;
}

adLocStatus_t MatcherLaneLine::ConvertMapLineToVehicle(
    std::unordered_map<id_t, std::vector<Point3D_t>>* converted_lines) {
  if (!converted_lines) {
    LC_LERROR(MATCHING) << "input nullptr.";
    return LOC_NULL_PTR;
  }

  SE3d Tvw = pose_state_.inverse();
  for (auto it = map_data_.begin(); it != map_data_.end(); ++it) {
    id_t id = it->first;
    const auto& line = it->second;
    const auto& points = line->GetLineSegments().front()->GetPoints();

    // transform to vehicle coordinate
    auto trans_points = TransformPoints(points, Tvw);
    converted_lines->insert(std::make_pair(id, std::move(trans_points)));
  }

  return LOC_SUCCESS;
}

adLocStatus_t MatcherLaneLine::CutLinesByDistance(
    const std::unordered_map<id_t, std::vector<Point3D_t>>& lines,
    std::pair<double, double> lateral_range,
    std::pair<double, double> longitudinal_range,
    std::unordered_map<id_t, std::vector<Point3D_t>>* new_lines) {
  if (!new_lines) {
    LC_LERROR(MATCHING) << "input nullptr.";
    return LOC_NULL_PTR;
  }

  for (const auto& id_line : lines) {
    id_t id = id_line.first;
    const auto& points = id_line.second;
    std::vector<Point3D_t> filtered_points;
    filtered_points.reserve(points.size());
    for (const auto& point : points) {
      if (point.y > lateral_range.first && point.y < lateral_range.second &&
          point.x > longitudinal_range.first &&
          point.x < longitudinal_range.second) {
        filtered_points.push_back(point);
      }
    }
    if (!filtered_points.empty()) {
      new_lines->insert(std::make_pair(id, std::move(filtered_points)));
    }
  }

  return LOC_SUCCESS;
}

adLocStatus_t MatcherLaneLine::SortLineByLateral(
    const std::unordered_map<id_t, std::vector<Point3D_t>>& lines,
    std::vector<id_t>* sorted_line_id) {
  if (!sorted_line_id) {
    LC_LERROR(MATCHING) << "input nullptr.";
    return LOC_NULL_PTR;
  }
  if (lines.empty()) {
    return LOC_INVALID;
  }

  std::map<double, id_t> sorted_id;
  for (auto it = lines.begin(); it != lines.end(); ++it) {
    const std::vector<Point3D_t>& points = it->second;
    if (points.empty()) {
      continue;
    }
    int count = 0;
    double lateral_sum = 0.0;
    for (const Point3D_t& point : points) {
      lateral_sum += point.y;
      ++count;
    }
    double lateral_mean = lateral_sum / count;
    sorted_id.insert(std::pair<double, id_t>(lateral_mean, it->first));
  }

  sorted_line_id->clear();
  for (const auto& lateral_and_id : sorted_id) {
    const id_t id = lateral_and_id.second;
    sorted_line_id->push_back(id);
  }
  return LOC_SUCCESS;
}

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
