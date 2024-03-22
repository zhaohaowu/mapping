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
#include "modules/local_mapping/lib/tracker/roadedge_tracker.h"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d.hpp"

namespace hozon {
namespace mp {
namespace lm {

// detect_index_point, track_index_point, distance
class RoadEdgeMatcher {
 public:
  bool Init(const MatcherInitOptions& options);

  bool Associate(const MatcherOptions& options,
                 const std::vector<RoadEdgePtr>& detected_roadedges,
                 const std::vector<RoadEdgeTrackerPtr>& roadedge_trackers,
                 AssociationResult* association_result);

  std::string Name() { return "RoadEdgeMatcher"; }

 private:
  void SolveBipartiteGraphMatchWithGreedy(
      const std::vector<RoadEdgePtr>& detected_roadedges,
      const std::vector<MatchScoreTuple>& match_score_list,
      AssociationResult* association_result);

  void SetTrackKDTree(const std::vector<RoadEdgeTrackerPtr>& roadedge_trackers);

  void Clear();

  void AssociationKnn(const MatcherOptions& options,
                      const std::vector<RoadEdgeTrackerPtr>& roadedge_trackers,
                      const std::vector<RoadEdgePtr>& detected_roadedges,
                      AssociationResult* association_result);

  std::vector<MatchScoreTuple> match_score_list_;
  MatchScoreTuple match_score_tuple_;
  std::vector<bool> target_used_mask_;
  std::vector<bool> det_used_mask_;
  std::vector<cv::flann::Index*> track_kdtrees_;
  std::vector<double> track_lanes_y_err_;
  std::vector<std::vector<double>> det_knn_thd_;
  std::string debug_timestamp_;
  int det_lanes_size_;
  int track_lanes_size_;

  int point_match_num_thresh_ = 6;
  float match_score_thresh_ = 5.0;
  float point_match_dis_thresh_ = 1.0;
  float point_quantile_thresh_ = 0.5;
  float vehicle_y_error_ratio_ = 2.0;
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
