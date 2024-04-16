// Copyright 2023 Hozon Inc. All Rights Reserved.
// @author: hozon
// @file: lane_matcher.cc
// @brief: matcher for lane

#include "modules/local_mapping/lib/association/roadedge_matcher.h"

#include <algorithm>
#include <vector>

#include "perception-base/base/utils/log.h"
#include "perception-lib/lib/config_manager/config_manager.h"
#include "utils/lane_utils.h"

namespace hozon {
namespace mp {
namespace lm {

namespace perception_lib = hozon::perception::lib;
bool RoadEdgeMatcher::Init(const MatcherInitOptions& options) {
  // lane_match_param_ = options.lane_match_param;
  target_used_mask_.reserve(50);
  det_used_mask_.reserve(50);
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

void RoadEdgeMatcher::Clear() {
  match_score_list_.clear();
  for (size_t i = 0; i < track_kdtrees_.size(); ++i) {
    if (track_kdtrees_[i] != nullptr) {
      delete track_kdtrees_[i];
      track_kdtrees_[i] = nullptr;
    }
  }
}

void RoadEdgeMatcher::SetTrackKDTree(
    const std::vector<RoadEdgeTrackerPtr>& roadedge_trackers) {
  track_lanes_size_ = roadedge_trackers.size();
  track_kdtrees_.resize(track_lanes_size_);
  for (size_t i = 0; i < track_lanes_size_; ++i) {
    const auto& lane_line =
        roadedge_trackers[i]->GetConstTarget()->GetConstTrackedObject();
    if (lane_line->vehicle_points.empty()) {
      track_kdtrees_[i] = nullptr;
      continue;
    }
    std::vector<cv::Point2f> cv_points;
    for (size_t j = 0; j < lane_line->vehicle_points.size(); ++j) {
      const auto& point = lane_line->vehicle_points[j];
      if (std::isnan(point.x()) || std::isnan(point.y()) ||
          std::isnan(point.z())) {
        break;
      }
      cv_points.emplace_back(point.x(), point.y());
    }
    if (cv_points.size() < 5) {
      track_kdtrees_[i] = nullptr;
      continue;
    }
    cv::flann::KDTreeIndexParams index_params(1);
    auto* kdtree =
        new cv::flann::Index(cv::Mat(cv_points).reshape(1), index_params);
    track_kdtrees_[i] = kdtree;
  }
}

void RoadEdgeMatcher::SolveBipartiteGraphMatchWithGreedy(
    const std::vector<RoadEdgePtr>& detected_roadedges,
    const std::vector<MatchScoreTuple>& match_score_list,
    AssociationResult* association_result) {
  det_lanes_size_ = detected_roadedges.size();
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

  HLOG_DEBUG << "point_match_debug timestamp: " << debug_timestamp_
             << ", Greedy AssociationResult: matched_pair_num:"
             << association_result->assignments.size()
             << ", unmatch tracker_num:"
             << association_result->unassigned_tracks.size()
             << ", unmatch detect_num:"
             << association_result->unsigned_objects.size();
}

void RoadEdgeMatcher::AssociationKnn(
    const MatcherOptions& options,
    const std::vector<RoadEdgeTrackerPtr>& roadedge_trackers,
    const std::vector<RoadEdgePtr>& detected_roadedges,
    AssociationResult* association_result) {
  std::stringstream time_ss;
  time_ss << std::fixed << std::setprecision(10) << options.timestamp;
  debug_timestamp_ = time_ss.str();
  for (int i = 0; i < detected_roadedges.size(); ++i) {
    const auto& det_point_set = detected_roadedges[i]->vehicle_points;
    for (int j = 0; j < track_kdtrees_.size(); ++j) {
      if (track_kdtrees_[j] == nullptr) {
        continue;
      }
      const auto& track_point_set = roadedge_trackers[j]
                                        ->GetConstTarget()
                                        ->GetConstTrackedObject()
                                        ->vehicle_points;

      int dist_match_cnt = 0;
      double dist_match_sum = 0;
      // 加入近处匹配判断
      int near_dist_match_cnt = 0;
      double near_dist_match_sum = 0;
      // 遍历检测点
      float overlay_min =
          std::max(det_point_set.front().x(), track_point_set.front().x());
      float overlay_max =
          std::min(det_point_set.back().x(), track_point_set.back().x());
      float near_max = overlay_min + 0.25F * (overlay_max - overlay_min);
      // overlay_min 到 near_max总共的点数(2作为初值兜底)
      int near_count_point = 2;

      for (size_t k = 0; k < det_point_set.size(); ++k) {
        // 找最近的一个点
        const int dim = 2;
        std::vector<int> nearest_index(dim);
        std::vector<float> nearest_dist(dim);
        auto find_pt_x = static_cast<float>(det_point_set[k].x());
        auto find_pt_y = static_cast<float>(det_point_set[k].y());
        std::vector<float> query_point =
            std::vector<float>({find_pt_x, find_pt_y});
        if (std::isnan(find_pt_x) || std::isnan(find_pt_y)) {
          continue;
        }
        if (find_pt_x > overlay_min && find_pt_x < near_max) {
          near_count_point++;
        }
        // 用检测和跟踪公共部分的点来计算距离
        if ((find_pt_x < overlay_min) || (find_pt_x > overlay_max)) {
          continue;
        }

        track_kdtrees_[j]->knnSearch(query_point, nearest_index, nearest_dist,
                                     dim, cv::flann::SearchParams(-1));
        // 根据阈值筛选点
        float y_dist = GetDistPointLane(det_point_set[k],
                                        track_point_set[nearest_index[0]],
                                        track_point_set[nearest_index[1]]);
        if (y_dist < point_match_dis_thresh_) {
          dist_match_sum += y_dist;
          dist_match_cnt++;
          // 近处匹配统计
          if (find_pt_x > overlay_min && find_pt_x < near_max) {
            near_dist_match_sum += y_dist;
            near_dist_match_cnt++;
          }
        }
      }
      near_dist_match_cnt = near_dist_match_cnt > 0 ? near_dist_match_cnt : 1;
      dist_match_cnt = dist_match_cnt > 0 ? dist_match_cnt : 1;
      double score = dist_match_sum / dist_match_cnt;
      double near_score = near_dist_match_sum / near_dist_match_cnt;

      HLOG_DEBUG << "roadedge point_match_debug timestamp: " << debug_timestamp_
                 << ", track index: " << j << ", trackId: "
                 << roadedge_trackers[j]->GetConstTarget()->Id()
                 << ", detect index: " << i
                 << ", detectId: " << detected_roadedges[i]->id
                 << ", match_point_num: " << dist_match_cnt
                 << ", dist_match_sum: " << dist_match_sum
                 << ", average distance: " << score
                 << ", near_match_point_num: " << near_dist_match_cnt
                 << ", near_dist_match_sum: " << near_dist_match_sum
                 << ", near_average distance: " << near_score
                 << ", overlay_min: " << overlay_min
                 << ", overlay_max: " << overlay_max;
      // 设置近处匹配阈值
      int near_match_cnt_threshold =
          std::min(static_cast<int>(near_count_point * 0.5),
                   3);  // 存在近处不足3个点的情况
      bool near_threshold_flag =
          near_dist_match_cnt < near_match_cnt_threshold || near_score > 0.6;
      bool threshold_flag = dist_match_cnt < 6 || score > 1.0;

      if (near_threshold_flag || threshold_flag) {
        continue;
      }
      float total_score = 5.0 / (near_score + score);

      // 0: track_index, 1: detect_index
      std::get<0>(match_score_tuple_) = j;
      std::get<1>(match_score_tuple_) = i;
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
  SolveBipartiteGraphMatchWithGreedy(detected_roadedges, match_score_list_,
                                     association_result);
}

bool RoadEdgeMatcher::Associate(
    const MatcherOptions& options,
    const std::vector<RoadEdgePtr>& detected_roadedges,
    const std::vector<RoadEdgeTrackerPtr>& roadedge_trackers,
    AssociationResult* association_result) {
  SetTrackKDTree(roadedge_trackers);
  AssociationKnn(options, roadedge_trackers, detected_roadedges,
                 association_result);
  Clear();
  return true;
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
