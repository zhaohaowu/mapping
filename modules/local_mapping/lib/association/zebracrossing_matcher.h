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
#include "modules/local_mapping/lib/tracker/zebracrossing_tracker.h"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d.hpp"

namespace hozon {
namespace mp {
namespace lm {

class ZebraCrossingMatcher {
 public:
  bool Init(const MatcherInitOptions& options);

  bool Associate(
      const MatcherOptions& options,
      const std::vector<ZebraCrossingPtr>& detected_zebracrossings,
      const std::vector<ZebraCrossingTrackerPtr>& zebracrossings_trackers,
      AssociationResult* association_result);

  void Clear();

 private:
  std::vector<MatchScoreTuple> match_score_list_;
  MatchScoreTuple match_score_tuple_;
  // LaneMatchParam lane_match_param_;
  std::vector<bool> target_used_mask_;
  std::vector<bool> det_used_mask_;

  size_t detect_zebracrossing_size_;
  size_t track_zebracrossing_size_;
  float match_x_dis_thresh_ = 5.0;
  float match_y_dis_thresh_ = 5.0;
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
