// Copyright 2023 Hozon Inc. All Rights Reserved.
// @author: hozon
// @file: lane_matcher.cc
// @brief: matcher for lane

#include "modules/local_mapping/lib/association/laneline_matcher.h"

#include <algorithm>
#include <vector>

#include "modules/local_mapping/lib/datalogger/pose_manager.h"
#include "perception-base/base/utils/log.h"
#include "perception-lib/lib/config_manager/config_manager.h"
#include "utils/lane_utils.h"

namespace hozon {
namespace mp {
namespace lm {

namespace perception_lib = hozon::perception::lib;

bool LaneLineMatcher::Init(const MatcherInitOptions& options) {
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

void LaneLineMatcher::Clear() {
  for (size_t i = 0; i < track_kdtrees_.size(); ++i) {
    if (track_kdtrees_[i] != nullptr) {
      delete track_kdtrees_[i];
      track_kdtrees_[i] = nullptr;
    }
  }
}

void LaneLineMatcher::SetTrackKDTree(
    const std::vector<LaneTrackerPtr>& lane_trackers) {
  track_lanes_size_ = lane_trackers.size();
  track_kdtrees_.resize(track_lanes_size_);
  track_lanes_y_err_.resize(track_lanes_size_);
  for (size_t i = 0; i < track_lanes_size_; ++i) {
    const auto& lane_line =
        lane_trackers[i]->GetConstTarget()->GetConstTrackedObject();
    if (lane_line->vehicle_points.empty()) {
      track_kdtrees_[i] = nullptr;
      continue;
    }
    std::vector<cv::Point2f> cv_points;
    double last_y = 0.0;
    double std_y = 0.0;
    size_t pt_num = lane_line->vehicle_points.size();
    for (size_t j = 0; j < pt_num; ++j) {
      const auto& v_point = lane_line->vehicle_points[j];
      cv_points.emplace_back(v_point.x(), v_point.y());
      if (j > 0) {
        std_y += std::abs(lane_line->vehicle_points[j].y() - last_y);
      }
      last_y = lane_line->vehicle_points[j].y();
    }

    std_y /= (pt_num - 1 + 0.001);
    track_lanes_y_err_[i] = std::min(std_y * vehicle_y_error_ratio_ + 1.0, 3.0);
    cv::flann::KDTreeIndexParams index_params(1);
    auto* kdtree =
        new cv::flann::Index(cv::Mat(cv_points).reshape(1), index_params);
    track_kdtrees_[i] = kdtree;
  }

  // 根据车辆打方向盘幅度（角速度大小）调整阈值
  PoseManager* local_data = PoseManager::Instance();
  auto& dr_data_buffer = local_data->GetDrBuffer();
  if (dr_data_buffer.buffer_size() > 0) {
    float angular_velocity = dr_data_buffer.back()->angular_velocity[2];
    point_quantile_thresh_ = 0.5 + std::min(std::abs(angular_velocity), 0.2f);
  }
}

void LaneLineMatcher::SolveBipartiteGraphMatchWithGreedy(
    const std::vector<LaneLinePtr>& detected_lanelines,
    const std::vector<MatchScoreTuple>& match_score_list,
    AssociationResult* association_result) {
  det_lanes_size_ = detected_lanelines.size();
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

void LaneLineMatcher::AssociationKnn(
    const MatcherOptions& options,
    const std::vector<LaneTrackerPtr>& lane_trackers,
    const std::vector<LaneLinePtr>& detected_lanelines,
    AssociationResult* association_result) {
  match_score_list_.clear();
  out_match_score_list_.clear();
  std::stringstream time_ss;
  time_ss << std::fixed << std::setprecision(10) << options.timestamp;
  debug_timestamp_ = time_ss.str();

  for (size_t i = 0; i < detected_lanelines.size(); ++i) {
    const auto& det_point_set = detected_lanelines.at(i)->vehicle_points;
    for (int j = 0; j < track_kdtrees_.size(); ++j) {
      if (track_kdtrees_[j] == nullptr) {
        continue;
      }
      const auto& track_point_set = lane_trackers[j]
                                        ->GetConstTarget()
                                        ->GetConstTrackedObject()
                                        ->vehicle_points;
      int near_dist_match_cnt = 0;     // 考虑近处的匹配结果
      int far_dist_match_cnt = 0;      // 考虑整个的匹配结果
      double near_dist_match_sum = 0;  // 考虑近处的匹配距离误差
      double far_dist_match_sum = 0;   // 考虑整个的匹配距离误差
      std::vector<double> dist_list;
      dist_list.clear();
      // 遍历检测点
      float overlay_min =
          std::max(det_point_set.front().x(), track_point_set.front().x());
      float overlay_max =
          std::min(det_point_set.back().x(), track_point_set.back().x());
      bool close_near_match = false;
      // 相交重叠很少或者不重叠可能是遮挡引起的，用整条线点做匹配
      if (std::abs(overlay_min - overlay_max) < 2.0) {
        overlay_min = static_cast<float>(det_point_set.front().x());
        overlay_max = static_cast<float>(det_point_set.back().x());
        point_match_dis_thresh_ = 0.4;
        close_near_match = true;
      }
      int near_count_point =
          2;  // overlay_min 到 near_max总共的点数(2作为初值兜底)
      float near_max = overlay_min + 0.25F * (overlay_max - overlay_min);

      for (size_t k = 0; k < det_point_set.size(); ++k) {
        // 找最近的一个点
        const int dim = 2;
        std::vector<int> nearest_index(dim);
        std::vector<float> nearest_dist(dim);
        auto find_pt_x = static_cast<float>(det_point_set[k].x());
        auto find_pt_y = static_cast<float>(det_point_set[k].y());
        std::vector<float> query_point =
            std::vector<float>({find_pt_x, find_pt_y});

        if (find_pt_x > overlay_min && find_pt_x < near_max) {
          near_count_point++;
        }
        // 完全没有重合点continue掉
        if (find_pt_x < overlay_min || find_pt_x > overlay_max) {
          continue;
        }
        // 用检测和跟踪公共部分的点来计算距离
        track_kdtrees_[j]->knnSearch(query_point, nearest_index, nearest_dist,
                                     dim, cv::flann::SearchParams(-1));
        // 根据阈值筛选点
        float y_dist = GetDistPointLane(det_point_set[k],
                                        track_point_set[nearest_index[0]],
                                        track_point_set[nearest_index[1]]);
        if (y_dist < point_match_dis_thresh_) {
          if (find_pt_x > overlay_min && find_pt_x < near_max) {
            near_dist_match_sum += y_dist;
            near_dist_match_cnt++;
          } else {
            far_dist_match_sum += y_dist;
            far_dist_match_cnt++;
            dist_list.push_back(y_dist);
          }
        }
      }
      HLOG_DEBUG << "laneline point_match_debug timestamp: " << debug_timestamp_
                 << ", track index: " << j
                 << ", trackId: " << lane_trackers[j]->GetConstTarget()->Id()
                 << ", detect index: " << i
                 << ", detectId: " << detected_lanelines[i]->id
                 << ", near_match_point_num: " << near_dist_match_cnt
                 << ", near_dist_match_sum: " << near_dist_match_sum
                 << ", near_average distance: "
                 << near_dist_match_sum / near_dist_match_cnt
                 << ", far_match_point_num: " << far_dist_match_cnt
                 << ", far_dist_match_sum: " << far_dist_match_sum
                 << ", far_average distance: "
                 << far_dist_match_sum / far_dist_match_cnt
                 << ", y err: " << track_lanes_y_err_[j]
                 << ", close_near_match: " << close_near_match
                 << ", overlay_min: " << overlay_min
                 << ", near_max: " << near_max;
      near_dist_match_cnt = near_dist_match_cnt > 0 ? near_dist_match_cnt : 1;
      far_dist_match_cnt = far_dist_match_cnt > 0 ? far_dist_match_cnt : 1;
      double near_score = near_dist_match_sum / near_dist_match_cnt;
      double far_score = far_dist_match_sum / far_dist_match_cnt;
      int near_match_cnt_threshold =
          std::min(static_cast<int>(near_count_point * 0.5),
                   3);  // 存在近处不足3个点的情况
      bool near_threshold_flag =
          near_dist_match_cnt < near_match_cnt_threshold || near_score > 0.6;
      bool threshold_flag = far_dist_match_cnt < 6 || far_score > 1.0;
      if (close_near_match) {
        near_threshold_flag = false;  // 调头时近处匹配阈值不生效
        near_score = 0.0;
      }
      if (near_threshold_flag || threshold_flag) {
        continue;
      }
      // 匹配分用于二分匹配
      float total_score = 5.0f / (near_score + far_score);
      // 匹配分考虑匹配点数
      total_score =
          total_score * std::log2f(static_cast<float>(far_dist_match_cnt));
      if (dist_list.size() > 0) {
        std::sort(dist_list.begin(), dist_list.end());
        int key_idx = static_cast<int>(dist_list.size() * 0.70);
        HLOG_DEBUG << "point_match_debug 50 percent distance:"
                   << dist_list[key_idx];
        if (dist_list[key_idx] >
            point_quantile_thresh_ * track_lanes_y_err_[j]) {
          continue;
        }
      }

      // 0: track_index, 1: detect_index
      std::get<0>(match_score_tuple_) = j;
      std::get<1>(match_score_tuple_) = i;
      std::get<2>(match_score_tuple_) = total_score;

      MatchScoreTuple score_tuple;
      std::get<0>(score_tuple) = lane_trackers[j]->GetConstTarget()->Id();
      std::get<1>(score_tuple) = detected_lanelines[i]->id;
      std::get<2>(score_tuple) = total_score;

      match_score_list_.push_back(match_score_tuple_);
      out_match_score_list_.emplace_back(score_tuple);
    }
  }
  // 按照匹配打分排序
  std::sort(match_score_list_.begin(), match_score_list_.end(),
            [](MatchScoreTuple a, MatchScoreTuple b) {
              return std::get<2>(a) > std::get<2>(b);
            });
  // 计算二分匹配结果
  SolveBipartiteGraphMatchWithGreedy(detected_lanelines, match_score_list_,
                                     association_result);
}

bool LaneLineMatcher::Associate(
    const MatcherOptions& options,
    const std::vector<LaneLinePtr>& detect_measurements,
    const std::vector<LaneTrackerPtr>& lane_trackers,
    AssociationResult* association_result) {
  SetTrackKDTree(lane_trackers);
  AssociationKnn(options, lane_trackers, detect_measurements,
                 association_result);
  Clear();
  return true;
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
