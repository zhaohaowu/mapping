/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhu Kaiying <zhukaiying@sensetime.com>
 */

#pragma once

#include <unordered_map>
#include <utility>
#include <vector>

#include "localization/data_type/base.hpp"

namespace senseAD {
namespace localization {
namespace smm {

// @brief: calculate matching distance for single point
adLocStatus_t CalMatchingDistance(
    const std::unordered_map<id_t, Point3D_t>& percept_data,
    const std::unordered_map<id_t, Point3D_t>& map_data,
    std::unordered_map<id_t, std::unordered_map<id_t, double>>* distances);

// @brief: calculate matching distance for point set
adLocStatus_t CalMatchingDistance(
    const std::unordered_map<id_t, std::vector<Point3D_t>>& percept_sets,
    const std::unordered_map<id_t, std::vector<Point3D_t>>& map_sets,
    std::unordered_map<id_t, std::unordered_map<id_t, double>>* distances);

// @brief: calculate matching manhattan distance for single point
adLocStatus_t CalMatchingManhattanDistance(
    const std::unordered_map<id_t, Point3D_t>& percept_data,
    const std::unordered_map<id_t, Eigen::Matrix2d>& percept_data_cov,
    const std::unordered_map<id_t, Point3D_t>& map_data,
    const std::unordered_map<id_t, Eigen::Matrix2d>& map_data_cov,
    std::unordered_map<id_t, std::unordered_map<id_t, double>>* distances);

// @brief: calculate matching manhattan distance for point set
adLocStatus_t CalMatchingManhattanDistance(
    const std::unordered_map<id_t, std::vector<Point3D_t>>& percept_sets,
    const std::unordered_map<id_t, std::vector<Eigen::Matrix2d>>&
        percept_set_covs,
    const std::unordered_map<id_t, std::vector<Point3D_t>>& map_sets,
    std::unordered_map<id_t, std::unordered_map<id_t, double>>* distances);

// @brief: get nearest matching pairs
adLocStatus_t CalNearestMatchingPairs(
    const std::unordered_map<id_t, std::unordered_map<id_t, double>>& distances,
    double distance_thre, bool unique_match,
    std::vector<std::pair<id_t, id_t>>* matching_pairs);

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
