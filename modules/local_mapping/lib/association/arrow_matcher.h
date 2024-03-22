// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: LiGuo (liguo03@hozon.com)
// @file: lane_matcher.h
// @brief: matcher for lane

#pragma once

#include <iomanip>
#include <list>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

// // #include "Eigen/src/Core/Matrix.h"
#include "modules/local_mapping/lib/association/base_struct.h"
#include "modules/local_mapping/lib/tracker/arrow_tracker.h"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d.hpp"

namespace hozon {
namespace mp {
namespace lm {

class ArrowMatcher {
 public:
  bool Init(const MatcherInitOptions& options);

  bool Associate(const MatcherOptions& options,
                 const std::vector<ArrowPtr>& detect_measurements,
                 const std::vector<ArrowTrackerPtr>& arrows_trackers,
                 AssociationResult* association_result);

 private:
  void Clear();

 private:
  std::vector<MatchScoreTuple> match_score_list_;
  MatchScoreTuple match_score_tuple_;
  // LaneLineMatchParam laneline_match_param_;
  std::vector<bool> target_used_mask_;
  std::vector<bool> det_used_mask_;

  size_t detect_arrows_size_;
  size_t track_arrows_size_;
  float arrow_match_x_dis_thresh_ = 10.0;
  float arrow_match_y_dis_thresh_ = 2.0;
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
