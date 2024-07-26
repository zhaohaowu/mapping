// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: LiGuo (liguo03@hozon.com)
// @file: occedge_matcher.h
// @brief: matcher for lane

#pragma once

#include <string>
#include <vector>

// // #include "Eigen/src/Core/Matrix.h"
#include "modules/local_mapping/lib/association/base_struct.h"
#include "modules/local_mapping/lib/tracker/occedge_tracker.h"
#include "opencv2/core/core.hpp"

namespace hozon {
namespace mp {
namespace lm {

// detect_index_point, track_index_point, distance
class OccEdgeMatcher {
 public:
  bool Init(const MatcherInitOptions& options);

  bool Associate(const MatcherOptions& options,
                 const std::vector<OccEdgePtr>& detected_occedges,
                 const std::vector<OccEdgeTrackerPtr>& occedge_trackers,
                 AssociationResult* association_result);

  std::string Name() { return "OccEdgeMatcher"; }

 private:
  void SolveBipartiteGraphMatchWithGreedy(
      const std::vector<MatchScoreTuple>& match_score_list,
      AssociationResult* association_result);

  void Clear();

  void Association(const MatcherOptions& options,
                   const std::vector<OccEdgeTrackerPtr>& occedge_trackers,
                   const std::vector<OccEdgePtr>& detected_occedges,
                   AssociationResult* association_result);

  std::vector<MatchScoreTuple> match_score_list_;
  MatchScoreTuple match_score_tuple_;
  std::vector<bool> target_used_mask_;
  std::vector<bool> det_used_mask_;
  std::vector<double> track_lanes_y_err_;
  std::string debug_timestamp_;
  size_t det_lanes_size_;
  size_t track_lanes_size_;

  int point_match_num_thresh_ = 6;
  float match_score_thresh_ = 5.0;
  float point_match_dis_thresh_ = 1.0;
  float point_quantile_thresh_ = 0.5;
  float vehicle_y_error_ratio_ = 2.0;

  std::vector<float> count_ratio_vector_;
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
