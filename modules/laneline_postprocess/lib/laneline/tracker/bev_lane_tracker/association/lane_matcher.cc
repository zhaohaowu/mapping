// Copyright 2023 Hozon Inc. All Rights Reserved.
// @author: hozon
// @file: lane_matcher.cc
// @brief: matcher for lane

#include "modules/laneline_postprocess/lib/laneline/tracker/bev_lane_tracker/association/lane_matcher.h"

#include <algorithm>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

namespace hozon {
namespace mp {
namespace environment {

LaneMatcher::LaneMatcher() {}

LaneMatcher::~LaneMatcher() {}

bool LaneMatcher::Init(const LaneMatcherInitOptions& options) {
  lane_match_param_ = options.lane_match_param;

  int_vec_.reserve(300);
  match_score_list_.reserve(100);
  auto& match_pints_set = std::get<2>(match_score_tuple_);
  match_pints_set.reserve(300);

  polynomial_ = std::make_shared<LaneLinePolynomial>();
  polynomial_->params.reserve(10);

  target_used_mask_.reserve(50);
  det_used_mask_.reserve(50);
  return true;
}

bool LaneMatcher::Associate(
    const LaneMatcherOptions& options,
    const std::vector<base::LaneLineMeasurementPtr>* detected_lanelines,
    const std::vector<SimpleLaneTrackerPtr>& lane_trackers,
    LaneAssociationResult* association_result) {
  if (!association_result) {
    return false;
  }
  association_result->assignments.clear();
  association_result->unassigned_tracks.clear();
  association_result->unsigned_objects.clear();
  match_score_list_.clear();
  // count ratio vector
  float count_match_ratio = 1.0;
  float decay_ratio = 0.90;
  const int count_num = 100;
  std::vector<float> count_ratio_vector(count_num, 0.01);
  for (size_t i = 0; i < count_num; i++) {
    count_ratio_vector[i] = count_match_ratio;
    count_match_ratio *= decay_ratio;
  }

  // todo(liguo) enrich match method
  for (size_t i = 0; i < lane_trackers.size(); i++) {
    auto& laneline =
        lane_trackers[i]->GetConstLaneTarget()->GetConstTrackedLaneLine();
    polynomial_->params.clear();
    for (int i = 0; i < laneline->vehicle_curve.coeffs.size(); ++i) {
      polynomial_->params.push_back(laneline->vehicle_curve.coeffs[i]);
    }
    polynomial_->min = laneline->vehicle_curve.min;
    polynomial_->max = laneline->vehicle_curve.max;
    // bool is_curb_track = IsCurb(laneline->position);

    for (size_t j = 0; j < detected_lanelines->size(); j++) {
      // bool is_curb_detect = IsCurb(detected_lanelines->at(j)->position);
      // if (is_curb_track != is_curb_detect) {
      //   continue;
      // }
      float average_dis = 0.0f;
      int count = 0;

      auto& match_pints_set = std::get<2>(match_score_tuple_);
      match_pints_set.clear();
      float min_detect = 1000.f;
      float max_detect = -1000.f;

      int valid_point_num = 0;
      int lane_point_match_num = 0;
      // average dist ratio
      float total_count_dist_ratio = 0.0;
      for (size_t k = 0; k < detected_lanelines->at(j)->point_set.size(); k++) {
        valid_point_num++;
        float y_pos = detected_lanelines->at(j)->point_set[k].vehicle_point.y;
        float x_pos = detected_lanelines->at(j)->point_set[k].vehicle_point.x;

        min_detect = std::min(min_detect, x_pos);
        max_detect = std::max(max_detect, x_pos);

        bool is_in_line_range =
            (x_pos <= polynomial_->max) && (x_pos >= polynomial_->min);
        float eval_value = polynomial_->eval(x_pos);
        float dist = fabs(eval_value - y_pos);
        // for not in range measurement, use a litte bigger threshold for add
        // measurement point
        float count_dist_ratio = k < count_num ? count_ratio_vector[k] : 0.01;
        const float param_dist_threshold =
            is_in_line_range ? lane_match_param_.same_group_max_dist()
                             : lane_match_param_.match_dis_for_no_in_range();
        match_pints_set.push_back(k);
        if (dist < param_dist_threshold) {
          average_dis += dist * count_dist_ratio;
          total_count_dist_ratio += count_dist_ratio;
          count++;
          lane_point_match_num++;
        }
      }

      average_dis =
          (count == 0) ? FLT_MAX : average_dis / total_count_dist_ratio;

      if (count < lane_match_param_.min_match_group_point_num() ||
          average_dis > lane_match_param_.max_match_distance()) {
        continue;
      }
      float match_detect_length = max_detect - min_detect;
      float track_length = polynomial_->max - polynomial_->min;
      float detect_length = detected_lanelines->at(j)->vehicle_curve.max -
                            detected_lanelines->at(j)->vehicle_curve.min + 0.01;
      float min_length = std::min(track_length, detect_length) + 1.0;

      // float length_score = fabs(track_length - match_detect_length) *
      //                      lane_match_param_.length_score_weight();
      float length_score = (1 - match_detect_length / min_length) *
                           lane_match_param_.length_score_weight();
      float point_match_ratio =
          count * 1.0 / detected_lanelines->at(j)->point_set.size();
      float point_match_score = (1.0 - point_match_ratio) *
                                lane_match_param_.point_match_score_weight();
      float total_score = average_dis + length_score + point_match_score;
      std::get<0>(match_score_tuple_) = i;
      std::get<1>(match_score_tuple_) = j;
      std::get<3>(match_score_tuple_) = total_score;
      match_score_list_.push_back(match_score_tuple_);
    }
  }

  std::sort(match_score_list_.begin(), match_score_list_.end(),
            [](MatchScoreTuple a, MatchScoreTuple b) {
              return std::get<3>(a) < std::get<3>(b);
            });

  SolveBipartiteGraphMatchWithGreedy(
      detected_lanelines, match_score_list_, lane_trackers.size(),
      detected_lanelines->size(), association_result);

  return true;
}

void LaneMatcher::SolveBipartiteGraphMatchWithGreedy(
    const std::vector<base::LaneLineMeasurementPtr>* detected_lanelines,
    const std::vector<MatchScoreTuple>& match_score_list, size_t targets_size,
    size_t objects_size, LaneAssociationResult* association_result) {
  target_used_mask_.resize(targets_size);
  target_used_mask_.assign(target_used_mask_.size(), false);
  det_used_mask_.resize(objects_size);
  det_used_mask_.assign(det_used_mask_.size(), false);

  for (auto& score_tuple : match_score_list) {
    size_t target_index = std::get<0>(score_tuple);
    size_t object_index = std::get<1>(score_tuple);

    if (target_used_mask_.at(target_index) || det_used_mask_.at(object_index)) {
      continue;
    }
    association_result->assignments.push_back(score_tuple);
    det_used_mask_.at(object_index) = true;
    target_used_mask_.at(target_index) = true;
  }
  // assign unassigned_obj_inds
  for (size_t i = 0; i < objects_size; ++i) {
    if (!det_used_mask_.at(i)) {
      int_vec_.clear();
      for (size_t k = 0; k < detected_lanelines->at(i)->point_set.size(); ++k) {
        int_vec_.push_back(k);
      }
      if (int_vec_.size() < lane_match_param_.min_init_track_point_num()) {
        continue;
      }
      LaneIndexPointsPair lane_points_pair(i, int_vec_);
      association_result->unsigned_objects.push_back(lane_points_pair);
    }
  }

  // assign unassigned_track_inds
  for (size_t i = 0; i < targets_size; ++i) {
    if (!target_used_mask_.at(i)) {
      association_result->unassigned_tracks.push_back(i);
    }
  }

  HLOG_DEBUG << "Greedy AssociationResult: matched_pair_num:"
             << association_result->assignments.size()
             << ", unmatch tracker_num:"
             << association_result->unassigned_tracks.size()
             << ", unmatch detect_num:"
             << association_result->unsigned_objects.size();
}

}  // namespace environment
}  // namespace mp
}  // namespace hozon
