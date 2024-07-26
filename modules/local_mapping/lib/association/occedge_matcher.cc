// Copyright 2023 Hozon Inc. All Rights Reserved.
// @author: hozon
// @file: occedge_matcher.cc
// @brief: matcher for lane

#include "modules/local_mapping/lib/association/occedge_matcher.h"

#include <algorithm>
#include <vector>

#include "perception-base/base/utils/log.h"
#include "perception-lib/lib/config_manager/config_manager.h"

namespace hozon {
namespace mp {
namespace lm {

namespace perception_lib = hozon::perception::lib;
bool OccEdgeMatcher::Init(const MatcherInitOptions& options) {
  // lane_match_param_ = options.lane_match_param;
  target_used_mask_.reserve(50);
  det_used_mask_.reserve(50);
  // count_ratio_vector初始化
  float count_match_ratio = 1.0;
  float decay_ratio = 0.90;
  const int count_num = 100;
  count_ratio_vector_.clear();
  for (size_t i = 0; i < count_num; i++) {
    count_ratio_vector_.push_back(count_match_ratio);
    count_match_ratio *= decay_ratio;
  }
  auto* config_manager = perception_lib::ConfigManager::Instance();
  const perception_lib::ModelConfig* model_config = nullptr;
  if (!config_manager->GetModelConfig(Name(), &model_config)) {
    HLOG_ERROR << "Parse config failed! Name: " << Name();
    return false;
  }

  if (!model_config->get_value("point_match_num_thresh",
                               &point_match_num_thresh_)) {
    HLOG_ERROR << "Get point_match_num_thresh failed!";
    return false;
  }

  if (!model_config->get_value("match_score_thresh", &match_score_thresh_)) {
    HLOG_ERROR << "Get match_score_thresh failed!";
    return false;
  }

  if (!model_config->get_value("match_dis_thresh", &point_match_dis_thresh_)) {
    HLOG_ERROR << "Get match_dis_thresh failed!";
    return false;
  }

  if (!model_config->get_value("point_quantile_thresh",
                               &point_quantile_thresh_)) {
    HLOG_ERROR << "Get point_quantile_thresh failed!";
    return false;
  }

  if (!model_config->get_value("vehicle_y_error_ratio",
                               &vehicle_y_error_ratio_)) {
    HLOG_ERROR << "Get vehicle_y_error_ratio failed!";
    return false;
  }

  return true;
}

void OccEdgeMatcher::Clear() { match_score_list_.clear(); }

void OccEdgeMatcher::SolveBipartiteGraphMatchWithGreedy(
    const std::vector<MatchScoreTuple>& match_score_list,
    AssociationResult* association_result) {
  HLOG_DEBUG << "bipartite size: " << track_lanes_size_ << ", "
             << det_lanes_size_;
  target_used_mask_.resize(track_lanes_size_);
  target_used_mask_.assign(target_used_mask_.size(), false);
  det_used_mask_.resize(det_lanes_size_);
  det_used_mask_.assign(det_used_mask_.size(), false);

  for (auto& score_tuple : match_score_list) {
    size_t track_index = std::get<0>(score_tuple);
    size_t detect_index = std::get<1>(score_tuple);

    if (target_used_mask_.at(track_index) || det_used_mask_.at(detect_index)) {
      continue;
    }
    association_result->assignments.push_back(score_tuple);
    det_used_mask_.at(detect_index) = true;
    target_used_mask_.at(track_index) = true;
  }

  // assign unassigned_obj_inds
  for (size_t i = 0; i < det_lanes_size_; ++i) {
    if (!det_used_mask_.at(i)) {
      association_result->unsigned_objects.push_back(i);
    }
  }

  // assign unassigned_track_inds
  for (size_t i = 0; i < track_lanes_size_; ++i) {
    if (!target_used_mask_.at(i)) {
      association_result->unassigned_tracks.push_back(i);
    }
  }

  HLOG_DEBUG << "occedge laneline_match_debug timestamp: " << debug_timestamp_
             << ", Greedy AssociationResult: matched_pair_num:"
             << association_result->assignments.size()
             << ", unmatch tracker_num:"
             << association_result->unassigned_tracks.size()
             << ", unmatch detect_num:"
             << association_result->unsigned_objects.size();
}

void OccEdgeMatcher::Association(
    const MatcherOptions& options,
    const std::vector<OccEdgeTrackerPtr>& occedge_trackers,
    const std::vector<OccEdgePtr>& detected_occedges,
    AssociationResult* association_result) {
  std::stringstream time_ss;
  time_ss << std::fixed << std::setprecision(10) << options.timestamp;
  debug_timestamp_ = time_ss.str();
  det_lanes_size_ = detected_occedges.size();
  track_lanes_size_ = occedge_trackers.size();
  for (size_t i = 0; i < occedge_trackers.size(); ++i) {
    const LaneLineCurve& track_vehicle_curve = occedge_trackers[i]
                                                   ->GetConstTarget()
                                                   ->GetConstTrackedObject()
                                                   ->vehicle_curve;
    CurveFitter track_curve(track_vehicle_curve);
    for (size_t j = 0; j < detected_occedges.size(); ++j) {
      int count = 0;
      int dist_match_cnt = 0;
      float average_dis = 0.0;
      float total_count_dist_ratio = 0.0;
      float dist_match_sum = 0;
      float max_detect = -1000.0;
      float min_detect = 1000.0;
      const auto& detected_occedge = detected_occedges[j];
      const auto& det_point_set = detected_occedge->vehicle_points;
      // 遍历检测点
      for (size_t k = 0; k < det_point_set.size(); ++k) {
        const auto& det_point = det_point_set[k];
        bool is_in_line_range = (det_point.x() <= track_curve.x_max) &&
                                (det_point.x() >= track_curve.x_min);
        auto eval_value =
            static_cast<float>(track_curve.evalueValue(det_point.x()));
        float dist = std::abs(eval_value - static_cast<float>(det_point.y()));
        float count_dist_ratio =
            k < count_ratio_vector_.size() ? count_ratio_vector_[k] : 0.01F;
        float param_dist_threshold = is_in_line_range ? 2.0 : 3.0;
        if (dist < param_dist_threshold) {
          max_detect = std::max(max_detect, static_cast<float>(det_point.x()));
          min_detect = std::min(min_detect, static_cast<float>(det_point.x()));
          average_dis += dist * count_dist_ratio;
          total_count_dist_ratio += count_dist_ratio;
          count++;
        }
      }
      average_dis =
          (count == 0) ? FLT_MAX : average_dis / total_count_dist_ratio;
      HLOG_DEBUG << "occedge laneline_match_debug timestamp: "
                 << debug_timestamp_
                 << ", trackId: " << occedge_trackers[i]->GetConstTarget()->Id()
                 << ", detectId: " << detected_occedges[j]->id
                 << ", match_point_num: " << count
                 << ", average distance: " << average_dis;
      // bool threshold_flag = count < 3 || average_dis > 1.5;
      bool threshold_flag = count < 4;
      if (threshold_flag) {
        continue;
      }
      float track_length = track_curve.x_max - track_curve.x_min;
      float detect_length = detected_occedge->vehicle_curve.max -
                            detected_occedge->vehicle_curve.min;
      float min_length = std::max(std::min(track_length, detect_length), 1.0F);
      float match_length = max_detect - min_detect;
      // 配置参数化
      float length_score_weight = 0.5;
      float length_score = (match_length / min_length) * length_score_weight;
      float point_match_ratio = count * 1.0F / det_point_set.size();
      // 配置参数化
      float point_score_weight = 0.5;
      float point_match_score = point_match_ratio * point_score_weight;
      float total_score = length_score + point_match_score;
      // 0: track_index, 1: detect_index
      std::get<0>(match_score_tuple_) = i;
      std::get<1>(match_score_tuple_) = j;
      std::get<2>(match_score_tuple_) = total_score;
      match_score_list_.push_back(match_score_tuple_);
    }
  }
  // 按照匹配打分排序
  std::sort(match_score_list_.begin(), match_score_list_.end(),
            [](MatchScoreTuple a, MatchScoreTuple b) {
              return std::get<2>(a) > std::get<2>(b);
            });
  // 计算二分匹配结果
  SolveBipartiteGraphMatchWithGreedy(match_score_list_, association_result);
}

bool OccEdgeMatcher::Associate(
    const MatcherOptions& options,
    const std::vector<OccEdgePtr>& detected_occedges,
    const std::vector<OccEdgeTrackerPtr>& occedge_trackers,
    AssociationResult* association_result) {
  Association(options, occedge_trackers, detected_occedges, association_result);
  Clear();
  return true;
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
