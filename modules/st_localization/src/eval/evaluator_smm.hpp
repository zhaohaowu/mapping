/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming<zhaoming@senseauto.com>
 */

#pragma once

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "eval/evaluator_base.hpp"

namespace senseAD {
namespace localization {

struct SMMEvalData {
  bool is_perception_valid = false;
  bool is_map_valid = false;
  bool is_smm_success = false;
  std::map<std::string, std::pair<int, int>>
      roadline_statistics;  //  {name : <laneline num, curb num>}
  double process_time_ms = 0.0;
  Eigen::Matrix4d input_pose;    // Tworld_vehicle
  Eigen::Matrix4d refined_pose;  // Tworld_vehicle
  Eigen::Matrix<double, 6, 6> pose_cov = Eigen::Matrix<double, 6, 6>::Zero();
};

class EvaluatorSMM : public EvaluatorBase<SMMEvalData> {
 public:
  EvaluatorSMM() = default;
  ~EvaluatorSMM() {}

  adLocStatus_t WriteResult(double timestamp, const SMMEvalData& data) override;

  // @brief: write file header for parsing
  adLocStatus_t WriteHeader(
      const std::vector<std::string>& enable_camera_names);

 private:
  std::vector<std::string> camera_names_;
};

}  // namespace localization
}  // namespace senseAD
