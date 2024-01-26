// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: LiGuo (liguo03@hozon.com)
// @file: lane_matcher.h
// @brief: matcher for lane

#pragma once
#include <list>
#include <tuple>
#include <utility>
#include <vector>

#include "modules/laneline_postprocess/lib/laneline/proto/lane_postprocess.pb.h"
#include "modules/laneline_postprocess/lib/laneline/tracker/bev_lane_tracker/base/lane_target.h"
#include "modules/laneline_postprocess/lib/laneline/tracker/bev_lane_tracker/data_fusion/simple_lane_tracker.h"
#include "modules/laneline_postprocess/lib/laneline/utils/laneline_polynomial.h"
#include "perception-base/base/measurement/laneline_measurement.h"
#include "perception-base/base/scene/laneline.h"

namespace hozon {
namespace mp {
namespace environment {

using base::LaneLine;

typedef std::pair<size_t, std::vector<size_t>> LaneIndexPointsPair;
// track_index, detect_lane_index, point_indexs_in_lane, match_score
typedef std::tuple<size_t, size_t, std::vector<size_t>, float> MatchScoreTuple;

struct LaneAssociationResult {
  std::vector<MatchScoreTuple> assignments;
  std::vector<size_t> unassigned_tracks;
  std::vector<LaneIndexPointsPair> unsigned_objects;
};

struct LaneMatcherInitOptions {
  LaneMatchParam lane_match_param;
};

struct LaneMatcherOptions {
  double camera_timestamp;
};

class LaneMatcher {
 public:
  LaneMatcher();

  virtual ~LaneMatcher();

  bool Init(const LaneMatcherInitOptions& options);

  bool Associate(
      const LaneMatcherOptions& options,
      const std::vector<base::LaneLineMeasurementPtr>* detected_lanelines,
      const std::vector<SimpleLaneTrackerPtr>& lane_trackers,
      LaneAssociationResult* association_result);

 private:
  void SolveBipartiteGraphMatchWithGreedy(
      const std::vector<base::LaneLineMeasurementPtr>* detected_lanelines,
      const std::vector<MatchScoreTuple>& match_score_list, size_t targets_size,
      size_t objects_size, LaneAssociationResult* association_result);

 private:
  std::vector<size_t> int_vec_;
  LaneLinePolynomialPtr polynomial_ = nullptr;
  std::vector<MatchScoreTuple> match_score_list_;
  MatchScoreTuple match_score_tuple_;
  LaneMatchParam lane_match_param_;
  std::vector<bool> target_used_mask_;
  std::vector<bool> det_used_mask_;
};

}  // namespace environment
}  // namespace mp
}  // namespace hozon
