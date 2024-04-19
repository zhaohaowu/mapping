/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */

#pragma once

#include <unordered_map>
#include <utility>
#include <vector>

#include "localization/data_type/base.hpp"
#include "localization/data_type/percept_data.hpp"
#include "localization/data_type/semantic_type.hpp"
#include "semantic_mm/base/lane_line.hpp"
#include "semantic_mm/matching/matcher/matcher_base.hpp"

namespace senseAD {
namespace localization {
namespace smm {

class MatcherLaneLine : public MatcherBase<PerceptLaneLine, LaneLine> {
 public:
  DEFINE_SMART_PTR(MatcherLaneLine)

  MatcherLaneLine() = default;
  ~MatcherLaneLine() {}

  // @brief: macthing core
  adLocStatus_t Matching(bool relocalization_mode) override;

 private:
  // @brief: preprocess for perception and map data
  adLocStatus_t PreProcess();

  // @brief: probability matching for laneline and roadside
  adLocStatus_t ProbabilityMatching();

  // @brief: nearest matching for roadside
  adLocStatus_t NearestMatchingForRoadSide();

  // @brief: inorder matching for laneline, roadside aided
  adLocStatus_t InOrderMatchingRoadSideAidedForLaneLine();

 private:
  // @brief: convert perception laneline points to world coodinate
  adLocStatus_t ConvertPerceptLineToWorld(
      std::unordered_map<id_t, std::vector<Point3D_t>>* converted_lines);

  // @brief: convert map laneline points to vehicle coodinate
  adLocStatus_t ConvertMapLineToVehicle(
      std::unordered_map<id_t, std::vector<Point3D_t>>* converted_lines);

  // @brief: remove line poins out of lateral range and longitudinal range
  adLocStatus_t CutLinesByDistance(
      const std::unordered_map<id_t, std::vector<Point3D_t>>& lines,
      std::pair<double, double> lateral_range,
      std::pair<double, double> longitudinal_range,
      std::unordered_map<id_t, std::vector<Point3D_t>>* new_lines);

  // @brief: sort lane lines from right to left
  adLocStatus_t SortLineByLateral(
      const std::unordered_map<id_t, std::vector<Point3D_t>>& lines,
      std::vector<id_t>* sorted_line_id);

 private:
  // percept datas
  std::unordered_map<id_t, std::vector<Point3D_t>> percept_laneline_points_;
  std::unordered_map<id_t, std::vector<Eigen::Matrix2d>>
      percept_laneline_point_covs_;
  std::unordered_map<id_t, std::vector<Point3D_t>> percept_roadside_points_;
  std::unordered_map<id_t, std::vector<Eigen::Matrix2d>>
      percept_roadside_point_covs_;

  // map datas
  std::unordered_map<id_t, std::vector<Point3D_t>> map_laneline_points_;
  std::unordered_map<id_t, std::vector<Point3D_t>> map_roadside_points_;
};

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
