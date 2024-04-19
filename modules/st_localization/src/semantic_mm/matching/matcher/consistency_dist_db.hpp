/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Fang Yu <fangyu1@senseauto.com>
 */

#pragma once

#include <unordered_map>
#include <utility>
#include <vector>

#include "localization/common/log.hpp"
#include "localization/data_type/base.hpp"
#include "localization/data_type/percept_data.hpp"
#include "localization/data_type/smart_ptr.hpp"
#include "semantic_mm/common/match_index.hpp"
#include "semantic_mm/common/math_tools.hpp"
#include "semantic_mm/common/voxel_index.hpp"

namespace senseAD {
namespace localization {
namespace smm {

struct ConsistencyDistDB {
  DEFINE_SMART_PTR(ConsistencyDistDB)

  void AddElement(id_t id, SemanticType type,
                  const std::vector<Point3D_t>& points) {
    if (element_points_.count(id)) {
      LC_LERROR(SMM) << "element " << id << "already exists!";
      return;
    }
    element_points_.insert(std::make_pair(id, std::make_pair(type, points)));
    return;
  }

  bool GetDistBetweenElements(id_t id1, id_t id2, double* dist) {
    if (!element_points_.count(id1) || !element_points_.count(id2))
      return false;
    const auto& element1 = element_points_.at(id1);
    const auto& element2 = element_points_.at(id2);
    SemanticType type1 = element1.first, type2 = element2.first;
    const auto& points1 = element1.second;
    const auto& points2 = element2.second;
    bool signed_dist =
        type1 == SemanticType::LaneLine || type2 == SemanticType::LaneLine;

    // 1. find evaluated distance btw id1 and id2
    MatchPair pair = std::make_pair(id1, id2);
    if (consistency_dists_.count(pair)) {
      *dist = consistency_dists_.at(pair);
      return std::fabs(*dist) < 1e9;
    }
    // 2. reverted id pair means negative of signed distance
    pair = std::make_pair(id2, id1);
    if (consistency_dists_.count(pair)) {
      *dist = consistency_dists_.at(pair);
      if (signed_dist) *dist *= -1;
      return std::fabs(*dist) < 1e9;
    }

    // 3. no distance found for (id1, id2) or (id2, id1), try to evaluate
    double distance = 0;
    int valid_num = 0;
    bool p2p_distance = type1 != SemanticType::LaneLine &&
                        type2 != SemanticType::LaneLine &&
                        points1.size() == 1 && points2.size() == 1;
    if (p2p_distance) {
      // unsigned point to point distance
      distance = (points1[0] - points2[0]).Norm2D();
      valid_num = 1;
    } else if (points1.size() <= points2.size()) {
      distance = Line2LineDistance2D(points1, points2, &valid_num, true);
    } else if (points1.size() > points2.size()) {
      distance = -1.0 * Line2LineDistance2D(points2, points1, &valid_num, true);
    }
    if (valid_num == 0) distance = 1e10;
    LC_LDEBUG(SMM) << id1 << "-" << id2 << " dist " << distance;
    consistency_dists_.insert({std::make_pair(id1, id2), distance});
    *dist = distance;
    return valid_num != 0;
  }

  bool GetDistBetweenElements(id_t id, id_t id_other,
                              ConsistencyDistDB::Ptr other_db,
                              const SE3d& T_this_other, double* dist) const {
    auto iter = element_points_.find(id);
    if (iter == element_points_.end()) return false;
    auto type = iter->second.first;
    const auto& points = iter->second.second;

    auto iter_other = other_db->element_points_.find(id_other);
    if (iter_other == other_db->element_points_.end()) return false;
    auto type_other = iter_other->second.first;
    const auto& points_other = iter_other->second.second;
    std::vector<Point3D_t> points_other_transed =
        TransformPoints(points_other, T_this_other);

    double distance = 0;
    int valid_num = 0;
    bool p2p_distance = type != SemanticType::LaneLine &&
                        type_other != SemanticType::LaneLine &&
                        points.size() == 1 && points_other.size() == 1;
    if (p2p_distance) {
      distance = (points[0] - points_other_transed[0]).Norm2D();
      valid_num = 1;
    } else {
      distance = Line2LineDistance2D(points_other_transed, points, &valid_num);
    }
    if (valid_num == 0) distance = 1e10;
    *dist = distance;
    return true;
  }

 private:
  // element points with id and semantic type
  std::unordered_map<id_t, std::pair<SemanticType, std::vector<Point3D_t>>>
      element_points_;
  // distance btw element pairs
  std::unordered_map<MatchPair, double, MatchPairHash> consistency_dists_;
};

struct PerceptPointsToMapLineDist {
  DEFINE_SMART_PTR(PerceptPointsToMapLineDist)
  double GetDistbtwTwoPerceptPoints(
      id_t percept_id1, const std::vector<Point2DWithCov>& percept_points1,
      id_t percept_id2, const std::vector<Point2DWithCov>& percept_points2) {
    MatchPair match_pair(percept_id1, percept_id2);
    if (percept_points_dists_.count(match_pair)) {
      return percept_points_dists_.at(match_pair);
    }

    MatchPair inverse_match_pair(percept_id2, percept_id1);
    if (percept_points_dists_.count(inverse_match_pair)) {
      return -percept_points_dists_.at(inverse_match_pair);
    }

    double dist =
        CalculateDistbtwTwoPerceptPoints(percept_points1, percept_points2);
    percept_points_dists_.insert({match_pair, dist});
    return dist;
  }

  double GetDistbtwTwoMapLines(id_t map_id1,
                               const Eigen::Vector4d& map_line_params1,
                               id_t map_id2,
                               const Eigen::Vector4d& map_line_params2) {
    MatchPair match_pair(map_id1, map_id2);
    if (map_line_dists_.count(match_pair)) {
      return map_line_dists_.at(match_pair);
    }

    MatchPair inverse_match_pair(map_id2, map_id1);
    if (map_line_dists_.count(inverse_match_pair)) {
      return -map_line_dists_.at(inverse_match_pair);
    }

    double dist =
        CalculateDistbtwTwoPolyFitLine(map_line_params1, map_line_params2);
    map_line_dists_.insert({match_pair, dist});
    return dist;
  }

  double GetDistbtwPerceptPointsMapLine(
      id_t percept_id, const std::vector<Point2DWithCov>& percept_points,
      id_t map_id, const Eigen::Vector4d& map_line_param, double heading,
      double search_y = 0) {
    MatchPair match_pair(percept_id, map_id);
    if (multi_heading_percept_points_to_map_line_dists_.count(heading) &&
        multi_heading_percept_points_to_map_line_dists_[heading].count(
            match_pair)) {
      return multi_heading_percept_points_to_map_line_dists_[heading]
                                                            [match_pair] +
             search_y;
    }

    Sophus::SE2<double> T_map_percept(heading, Eigen::Vector2d::Zero());
    double dist = CalculateDistbtwPerceptPointsMapLine(
        percept_points, map_line_param, T_map_percept);
    auto& percept_points_to_map_line_dists =
        multi_heading_percept_points_to_map_line_dists_[heading];
    percept_points_to_map_line_dists.insert({match_pair, dist});

    return dist + search_y;
  }

 private:
  std::unordered_map<MatchPair, double, MatchPairHash> percept_points_dists_;
  std::unordered_map<MatchPair, double, MatchPairHash> map_line_dists_;
  std::unordered_map<double,
                     std::unordered_map<MatchPair, double, MatchPairHash>>
      multi_heading_percept_points_to_map_line_dists_;
};

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
