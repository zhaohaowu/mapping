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
#include "modules/local_mapping/lib/tracker/laneline_tracker.h"
#include "modules/local_mapping/lib/tracker/roadedge_tracker.h"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d.hpp"

namespace hozon {
namespace mp {
namespace lm {

// detect_index_point, track_index_point, distance
class LaneLineMatcher {
 public:
  bool Init(const MatcherInitOptions& options);

  bool Associate(const MatcherOptions& options,
                 const std::vector<LaneLinePtr>& detect_measurements,
                 const std::vector<LaneTrackerPtr>& lane_trackers,
                 AssociationResult* association_result);

  std::string Name() { return "LaneLineMatcher"; }

  std::vector<MatchScoreTuple> GetMatchScoreList() {
    return out_match_score_list_;
  }

 private:
  void SolveBipartiteGraphMatchWithGreedy(
      const std::vector<LaneLinePtr>& detected_lanelines,
      const std::vector<MatchScoreTuple>& match_score_list,
      AssociationResult* association_result);

  void SetTrackKDTree(const std::vector<LaneTrackerPtr>& lane_trackers);

  void Clear();
  void AssociationKnn(const MatcherOptions& options,
                      const std::vector<LaneTrackerPtr>& lane_trackers,
                      const std::vector<LaneLinePtr>& detected_lanelines,
                      AssociationResult* association_result);

 private:
  std::vector<MatchScoreTuple> match_score_list_;
  std::vector<MatchScoreTuple> out_match_score_list_;
  MatchScoreTuple match_score_tuple_;
  // LaneLineMatchParam laneline_match_param_;
  std::vector<bool> target_used_mask_;
  std::vector<bool> det_used_mask_;
  std::vector<cv::flann::Index*> track_kdtrees_;
  std::vector<double> track_lanes_y_err_;
  std::string debug_timestamp_;
  size_t det_lanes_size_;
  size_t track_lanes_size_;

  int point_match_num_thresh_ = 6;
  float match_score_thresh_ = 5.0;
  float point_match_dis_thresh_ = 1.0;
  float point_quantile_thresh_ = 0.5;
  float vehicle_y_error_ratio_ = 2.0;
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
