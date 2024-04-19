/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhu Kaiying <zhukaiying@sensetime.com>
 */

#pragma once

#include <unordered_map>
#include <utility>

#include "localization/data_type/base.hpp"
#include "localization/data_type/percept_data.hpp"
#include "semantic_mm/base/traffic_sign.hpp"
#include "semantic_mm/matching/matcher/matcher_base.hpp"

namespace senseAD {
namespace localization {
namespace smm {

class MatcherTrafficSign : public MatcherBase<PerceptTrafficSign, TrafficSign> {
 public:
  DEFINE_SMART_PTR(MatcherTrafficSign)

  MatcherTrafficSign() = default;
  ~MatcherTrafficSign() {}

  // @brief: macthing core
  adLocStatus_t Matching(bool relocalization_mode) override;

 private:
  // @brief: preprocess for perception and map data
  adLocStatus_t PreProcess();

  // @brief: probability matching
  adLocStatus_t ProbabilityMatching();

 private:
  // percept datas
  std::unordered_map<id_t, Point3D_t> percept_sign_points_;
  std::unordered_map<id_t, Eigen::Matrix2d> percept_sign_point_covs_;

  // map datas
  std::unordered_map<id_t, Point3D_t> map_sign_points_;
  std::unordered_map<id_t, Eigen::Matrix2d> map_sign_point_covs_;
};

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
