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

#include "modules/laneline_postprocess/lib/laneline/proto/lane_postprocess.pb.h"
#include "modules/laneline_postprocess/lib/laneline/tracker/bev_lane_tracker/base/lane_target.h"
#include "modules/laneline_postprocess/lib/laneline/tracker/bev_lane_tracker/data_fusion/simple_lane_tracker.h"
#include "modules/laneline_postprocess/lib/laneline/tracker/bev_lane_tracker/data_fusion/simple_roadedge_tracker.h"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d.hpp"
#include "perception-base/base/point/point.h"
#include "perception-base/base/scene/laneline.h"

namespace hozon {
namespace mp {
namespace environment {

namespace perception_base = hozon::perception::base;

// detect_index_point, track_index_point, distance
typedef std::tuple<size_t, size_t, double> PointPair;
typedef std::vector<PointPair> PointIndexPointsPair;
typedef std::shared_ptr<PointIndexPointsPair> PointIndexPointsPairPtr;

// track_index, detect_lane_index, points_pair, match_score
typedef std::tuple<size_t, size_t, PointIndexPointsPairPtr, float>
    PointMatchScoreTuple;

struct PointAssociationResult {
  std::vector<PointMatchScoreTuple> assignments;
  std::vector<size_t> unassigned_tracks;
  std::vector<size_t> unsigned_objects;
};

struct PointMatcherInitOptions {
  LaneMatchParam lane_match_param;
};

struct PointMatcherOptions {
  double timestamp = 0.0;
};

class PointMatcher {
 public:
  PointMatcher();

  virtual ~PointMatcher();

  bool Init(const PointMatcherInitOptions& options);

  bool Associate(const PointMatcherOptions& options,
                 const std::vector<perception_base::LaneLineMeasurementPtr>*
                     detected_lanelines,
                 const std::vector<SimpleLaneTrackerPtr>& lane_trackers,
                 PointAssociationResult* association_result);

  bool Associate(const PointMatcherOptions& options,
                 const std::vector<perception_base::RoadEdgeMeasurementPtr>*
                     detected_road_edges,
                 const std::vector<SimpleRoadEdgeTrackerPtr>& lane_trackers,
                 PointAssociationResult* association_result);

 private:
  void SolveBipartiteGraphMatchWithGreedy(
      const std::vector<perception_base::LaneLineMeasurementPtr>*
          detected_lanelines,
      const std::vector<PointMatchScoreTuple>& match_score_list,
      PointAssociationResult* association_result);

  void SolveBipartiteGraphMatchWithGreedy(
      const std::vector<perception_base::RoadEdgeMeasurementPtr>*
          detected_road_edges,
      const std::vector<PointMatchScoreTuple>& match_score_list,
      PointAssociationResult* association_result);

  void SetTrackKDTree(const std::vector<SimpleLaneTrackerPtr>& lane_trackers);
  void SetTrackKDTree(
      const std::vector<SimpleRoadEdgeTrackerPtr>& lane_trackers);

  void Clear();
  void AssociationKnn(
      const PointMatcherOptions& options,
      const std::vector<SimpleLaneTrackerPtr>& lane_trackers,
      const std::vector<perception_base::LaneLineMeasurementPtr>*
          detected_lanelines,
      PointAssociationResult* association_result);

  void AssociationKnn(
      const PointMatcherOptions& options,
      const std::vector<SimpleRoadEdgeTrackerPtr>& lane_trackers,
      const std::vector<perception_base::RoadEdgeMeasurementPtr>*
          detected_road_edges,
      PointAssociationResult* association_result);

  double NormPoint(perception_base::Point3DF point);
  void SetDetectionPointDist(
      const std::vector<perception_base::LaneLineMeasurementPtr>*
          detected_lanelines);

  void SetDetectionPointDist(
      const std::vector<perception_base::RoadEdgeMeasurementPtr>*
          detected_roadedges);

 private:
  std::vector<PointMatchScoreTuple> match_score_list_;
  PointMatchScoreTuple match_score_tuple_;
  LaneMatchParam lane_match_param_;
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

}  // namespace environment
}  // namespace mp
}  // namespace hozon
