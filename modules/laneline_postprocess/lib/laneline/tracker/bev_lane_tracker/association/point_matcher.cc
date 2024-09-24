// Copyright 2023 Hozon Inc. All Rights Reserved.
// @author: hozon
// @file: lane_matcher.cc
// @brief: matcher for lane

#include "modules/laneline_postprocess/lib/laneline/tracker/bev_lane_tracker/association/point_matcher.h"

#include <algorithm>
#include <vector>

namespace hozon {
namespace mp {
namespace environment {

PointMatcher::PointMatcher() {}

PointMatcher::~PointMatcher() {}

bool PointMatcher::Init(const PointMatcherInitOptions& options) {
  lane_match_param_ = options.lane_match_param;
  target_used_mask_.reserve(50);
  det_used_mask_.reserve(50);
  return true;
}

void PointMatcher::Clear() {
  det_knn_thd_.clear();
  match_score_list_.clear();
  for (size_t i = 0; i < track_kdtrees_.size(); ++i) {
    if (track_kdtrees_[i] != nullptr) {
      delete track_kdtrees_[i];
      track_kdtrees_[i] = nullptr;
    }
  }
}

double PointMatcher::NormPoint(perception_base::Point3DF point) {
  double dist = point.x * point.x + point.y * point.y + point.z * point.z;
  return std::sqrt(dist);
}

void PointMatcher::SetDetectionPointDist(
    const std::vector<perception_base::LaneLineMeasurementPtr>*
        detected_lanelines) {
  det_lanes_size_ = detected_lanelines->size();
  for (int i = 0; i < det_lanes_size_; ++i) {
    const auto& lane = detected_lanelines->at(i);
    int point_size = lane->point_set.size();
    std::vector<double> vec_dist;
    vec_dist.resize(point_size);
    for (int j = 0; j < point_size; ++j) {
      double pt_range = NormPoint(lane->point_set[j].local_point);
      // todo 改成配置化参数
      // double yaw_std = lane_match_param_.yaw_std();
      double trans_std = 0.04;
      double xyz_std = 0.4;
      double yaw_std = 0.035;
      double upper_thd_R = 2 * pt_range * sin(yaw_std / 180 * M_PI / 2);
      // std::cout << "upper_thd_R==" << upper_thd_R << std::endl;
      double upper_thd_t = trans_std * 2;
      double upper_thd_xyz = xyz_std * std::pow(2, 2);
      double upper_thd = upper_thd_R + upper_thd_t + upper_thd_xyz;

      // std::cout << "upper_thd==" << upper_thd << std::endl;
      upper_thd = std::max(upper_thd, 1.0);
      vec_dist[j] = upper_thd;
    }
    det_knn_thd_.emplace_back(vec_dist);
  }
}

void PointMatcher::SetDetectionPointDist(
    const std::vector<perception_base::RoadEdgeMeasurementPtr>*
        detected_lanelines) {
  det_lanes_size_ = detected_lanelines->size();
  for (int i = 0; i < det_lanes_size_; ++i) {
    const auto& lane = detected_lanelines->at(i);
    int point_size = lane->point_set.size();
    std::vector<double> vec_dist;
    vec_dist.resize(point_size);
    for (int j = 0; j < point_size; ++j) {
      double pt_range = NormPoint(lane->point_set[j].local_point);
      // todo 改成配置化参数
      // double yaw_std = lane_match_param_.yaw_std();
      double trans_std = 0.04;
      double xyz_std = 0.4;
      double yaw_std = 0.035;
      double upper_thd_R = 2 * pt_range * sin(yaw_std / 180 * M_PI / 2);
      // std::cout << "upper_thd_R==" << upper_thd_R << std::endl;
      double upper_thd_t = trans_std * 2;
      double upper_thd_xyz = xyz_std * std::pow(2, 2);
      double upper_thd = upper_thd_R + upper_thd_t + upper_thd_xyz;

      // std::cout << "upper_thd==" << upper_thd << std::endl;
      upper_thd = std::max(upper_thd, 1.0);
      vec_dist[j] = upper_thd;
    }
    det_knn_thd_.emplace_back(vec_dist);
  }
}

void PointMatcher::SetTrackKDTree(
    const std::vector<SimpleLaneTrackerPtr>& lane_trackers) {
  track_lanes_size_ = lane_trackers.size();
  track_kdtrees_.resize(track_lanes_size_);
  track_lanes_y_err_.resize(track_lanes_size_);
  for (size_t i = 0; i < track_lanes_size_; ++i) {
    const auto& lane_line =
        lane_trackers[i]->GetConstLaneTarget()->GetConstTrackedLaneLine();
    if (lane_line->point_set.empty()) {
      track_kdtrees_[i] = nullptr;
      continue;
    }
    std::vector<cv::Point2f> cv_points;
    double last_y = 0.0, std_y = 0.0;
    int pt_num = lane_line->point_set.size();
    for (size_t j = 0; j < pt_num; ++j) {
      const auto& point = lane_line->point_set[j].local_point;
      if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
        break;
      }
      if (std::isinf(point.x) || std::isinf(point.y) || std::isinf(point.z)) {
        break;
      }
      cv_points.emplace_back(point.x, point.y);
      if (j > 0) {
        std_y += std::abs(lane_line->point_set[j].vehicle_point.y - last_y);
      }
      last_y = lane_line->point_set[j].vehicle_point.y;
    }
    if (cv_points.size() < 5) {
      track_kdtrees_[i] = nullptr;
      continue;
    }
    std_y /= pt_num - 1;
    track_lanes_y_err_[i] = std::min(std_y * vehicle_y_error_ratio_ + 1.0, 3.0);
    cv::flann::KDTreeIndexParams index_params(1);
    cv::flann::Index* kdtree =
        new cv::flann::Index(cv::Mat(cv_points).reshape(1), index_params);
    track_kdtrees_[i] = kdtree;
  }
  // 根据车辆打方向盘幅度（角速度大小）调整阈值
  environment::InputDataSingleton* local_data_ =
      environment::InputDataSingleton::Instance();
  auto& dr_data_buffer = local_data_->dr_data_buffer_;
  if (dr_data_buffer.buffer_size() > 0) {
    float angular_velocity = dr_data_buffer.back()->angular_velocity[2];
    point_quantile_thresh_ += std::min(std::abs(angular_velocity), 0.2f);
  }
}

void PointMatcher::SetTrackKDTree(
    const std::vector<SimpleRoadEdgeTrackerPtr>& lane_trackers) {
  track_lanes_size_ = lane_trackers.size();
  track_kdtrees_.resize(track_lanes_size_);
  for (size_t i = 0; i < track_lanes_size_; ++i) {
    const auto& lane_line =
        lane_trackers[i]->GetConstLaneTarget()->GetConstTrackedLaneLine();
    if (lane_line->point_set.empty()) {
      track_kdtrees_[i] = nullptr;
      continue;
    }
    std::vector<cv::Point2f> cv_points;
    for (size_t j = 0; j < lane_line->point_set.size(); ++j) {
      const auto& point = lane_line->point_set[j].local_point;
      if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
        break;
      }
      if (std::isinf(point.x) || std::isinf(point.y) || std::isinf(point.z)) {
        break;
      }
      cv_points.emplace_back(point.x, point.y);
    }
    if (cv_points.size() < 5) {
      track_kdtrees_[i] = nullptr;
      continue;
    }
    cv::flann::KDTreeIndexParams index_params(1);
    cv::flann::Index* kdtree =
        new cv::flann::Index(cv::Mat(cv_points).reshape(1), index_params);
    track_kdtrees_[i] = kdtree;
  }
}

void PointMatcher::SolveBipartiteGraphMatchWithGreedy(
    const std::vector<perception_base::LaneLineMeasurementPtr>*
        detected_lanelines,
    const std::vector<PointMatchScoreTuple>& match_score_list,
    PointAssociationResult* association_result) {
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

void PointMatcher::SolveBipartiteGraphMatchWithGreedy(
    const std::vector<perception_base::RoadEdgeMeasurementPtr>*
        detected_roadedges,
    const std::vector<PointMatchScoreTuple>& match_score_list,
    PointAssociationResult* association_result) {
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

void PointMatcher::AssociationKnn(
    const PointMatcherOptions& options,
    const std::vector<SimpleLaneTrackerPtr>& lane_trackers,
    const std::vector<perception_base::LaneLineMeasurementPtr>*
        detected_lanelines,
    PointAssociationResult* association_result) {
  std::stringstream time_ss;
  time_ss << std::fixed << std::setprecision(10) << options.timestamp;
  debug_timestamp_ = time_ss.str();
  for (int i = 0; i < detected_lanelines->size(); ++i) {
    const auto& det_point_set = detected_lanelines->at(i)->point_set;
    for (int j = 0; j < track_kdtrees_.size(); ++j) {
      if (track_kdtrees_[j] == nullptr) {
        continue;
      }
      lane_trackers[j]->GetLaneTarget()->SetLastestTrackedTimestamp(
          options.timestamp);
      const auto& track_point_set = lane_trackers[j]
                                        ->GetConstLaneTarget()
                                        ->GetConstTrackedLaneLine()
                                        ->point_set;
      int near_dist_match_cnt = 0;     // 考虑近处的匹配结果
      int far_dist_match_cnt = 0;      // 考虑整个的匹配结果
      double near_dist_match_sum = 0;  // 考虑近处的匹配距离误差
      double far_dist_match_sum = 0;   // 考虑整个的匹配距离误差
      std::vector<double> dist_list;
      dist_list.clear();
      PointIndexPointsPairPtr point_dist_ptr =
          std::make_shared<PointIndexPointsPair>();
      point_dist_ptr->reserve(det_point_set.size());
      // 遍历检测点

      float overlay_min = std::max(det_point_set.front().vehicle_point.x,
                                   track_point_set.front().vehicle_point.x);
      float overlay_max = std::min(det_point_set.back().vehicle_point.x,
                                   track_point_set.back().vehicle_point.x);
      bool close_near_match = false;
      // 相交重叠很少或者不重叠可能是遮挡引起的，用整条线点做匹配
      if (std::abs(overlay_min - overlay_max) < 2.0) {
        overlay_min = static_cast<float>(det_point_set.front().vehicle_point.x);
        overlay_max = static_cast<float>(det_point_set.back().vehicle_point.x);
        point_match_dis_thresh_ = 0.4;
        close_near_match = true;
      }
      int near_count_point =
          2;  // overlay_min 到 near_max总共的点数(2作为初值兜底)
      float near_max = overlay_min + 0.25F * (overlay_max - overlay_min);
      for (size_t k = 0; k < det_point_set.size(); ++k) {
        // 找最近的一个点
        const int dim = 1;
        std::vector<int> nearest_index(dim);
        std::vector<float> nearest_dist(dim);
        std::vector<float> query_point = std::vector<float>(
            {det_point_set[k].local_point.x, det_point_set[k].local_point.y});
        if (std::isnan(query_point[0]) || std::isnan(query_point[1])) {
          continue;
        }
        if (std::isinf(query_point[0]) || std::isinf(query_point[1])) {
          continue;
        }
        if ((det_point_set[k].vehicle_point.x > overlay_min) &&
            (det_point_set[k].vehicle_point.x < near_max)) {
          near_count_point++;
        }

        // 用检测和跟踪公共部分的点来计算距离
        if ((det_point_set[k].vehicle_point.x < overlay_min) ||
            (det_point_set[k].vehicle_point.x > overlay_max))
          continue;

        track_kdtrees_[j]->knnSearch(query_point, nearest_index, nearest_dist,
                                     dim, cv::flann::SearchParams(-1));
        // 根据阈值筛选点
        float y_dist =
            std::abs(track_point_set[nearest_index[0]].vehicle_point.y -
                     det_point_set[k].vehicle_point.y);

        if (y_dist < point_match_dis_thresh_) {
          if (det_point_set[k].vehicle_point.x > overlay_min &&
              det_point_set[k].vehicle_point.x < near_max) {
            near_dist_match_sum += y_dist;
            near_dist_match_cnt++;
            PointPair pt =
                std::make_tuple(k, nearest_index[0], nearest_dist[0]);
            point_dist_ptr->emplace_back(pt);
          } else {
            far_dist_match_sum += y_dist;
            far_dist_match_cnt++;
            dist_list.push_back(y_dist);
            PointPair pt =
                std::make_tuple(k, nearest_index[0], nearest_dist[0]);
            point_dist_ptr->emplace_back(pt);
          }
        }
      }
      near_dist_match_cnt = near_dist_match_cnt > 0 ? near_dist_match_cnt : 1;
      far_dist_match_cnt = far_dist_match_cnt > 0 ? far_dist_match_cnt : 1;
      double near_score = near_dist_match_sum / near_dist_match_cnt;
      double far_score = far_dist_match_sum / far_dist_match_cnt;

      HLOG_DEBUG << "laneline point_match_debug timestamp: " << debug_timestamp_
                 << ", track index: " << j << ", trackId: "
                 << lane_trackers[j]->GetConstLaneTarget()->Id()
                 << ", detect index: " << i
                 << ", detectId: " << detected_lanelines->at(i)->id
                 << ", near_match_point_num: " << near_dist_match_cnt
                 << ", near_dist_match_sum: " << near_dist_match_sum
                 << ", near_average distance: " << near_score
                 << ", far_match_point_num: " << far_dist_match_cnt
                 << ", far_dist_match_sum: " << far_dist_match_sum
                 << ", far_average distance: " << far_score
                 << ", y err: " << track_lanes_y_err_[j]
                 << ", close_near_match: " << close_near_match
                 << ", overlay_min: " << overlay_min
                 << ", near_max: " << near_max;

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
      std::get<2>(match_score_tuple_) = point_dist_ptr;
      std::get<3>(match_score_tuple_) = total_score;

      match_score_list_.push_back(match_score_tuple_);
    }
  }
  // 按照匹配打分排序
  std::sort(match_score_list_.begin(), match_score_list_.end(),
            [](PointMatchScoreTuple a, PointMatchScoreTuple b) {
              return std::get<3>(a) > std::get<3>(b);
            });
  // 计算二分匹配结果
  SolveBipartiteGraphMatchWithGreedy(detected_lanelines, match_score_list_,
                                     association_result);
}

void PointMatcher::AssociationKnn(
    const PointMatcherOptions& options,
    const std::vector<SimpleRoadEdgeTrackerPtr>& lane_trackers,
    const std::vector<perception_base::RoadEdgeMeasurementPtr>*
        detected_lanelines,
    PointAssociationResult* association_result) {
  std::stringstream time_ss;
  time_ss << std::fixed << std::setprecision(10) << options.timestamp;
  debug_timestamp_ = time_ss.str();
  for (int i = 0; i < detected_lanelines->size(); ++i) {
    const auto& det_point_set = detected_lanelines->at(i)->point_set;
    for (int j = 0; j < track_kdtrees_.size(); ++j) {
      if (track_kdtrees_[j] == nullptr) {
        continue;
      }
      const auto& track_point_set = lane_trackers[j]
                                        ->GetConstLaneTarget()
                                        ->GetConstTrackedLaneLine()
                                        ->point_set;

      int near_dist_match_cnt = 0;     // 考虑近处的匹配结果
      int far_dist_match_cnt = 0;      // 考虑整个的匹配结果
      double near_dist_match_sum = 0;  // 考虑近处的匹配距离误差
      double far_dist_match_sum = 0;   // 考虑整个的匹配距离误差
      PointIndexPointsPairPtr point_dist_ptr =
          std::make_shared<PointIndexPointsPair>();
      point_dist_ptr->reserve(det_point_set.size());
      // 遍历检测点

      float overlay_min = std::max(det_point_set.front().vehicle_point.x,
                                   track_point_set.front().vehicle_point.x);
      float overlay_max = std::min(det_point_set.back().vehicle_point.x,
                                   track_point_set.back().vehicle_point.x);
      bool close_near_match = false;
      // 相交重叠很少或者不重叠可能是遮挡引起的，用整条线点做匹配
      if (std::abs(overlay_min - overlay_max) < 2.0) {
        overlay_min = static_cast<float>(det_point_set.front().vehicle_point.x);
        overlay_max = static_cast<float>(det_point_set.back().vehicle_point.x);
        point_match_dis_thresh_ = 0.4;
        close_near_match = true;
      }
      int near_count_point =
          2;  // overlay_min 到 near_max总共的点数(2作为初值兜底)
      float near_max = overlay_min + 0.25F * (overlay_max - overlay_min);
      for (size_t k = 0; k < det_point_set.size(); ++k) {
        // 找最近的一个点
        const int dim = 1;
        std::vector<int> nearest_index(dim);
        std::vector<float> nearest_dist(dim);
        std::vector<float> query_point = std::vector<float>(
            {det_point_set[k].local_point.x, det_point_set[k].local_point.y});
        if (std::isnan(query_point[0]) || std::isnan(query_point[1])) {
          continue;
        }
        if (std::isinf(query_point[0]) || std::isinf(query_point[1])) {
          continue;
        }
        if ((det_point_set[k].vehicle_point.x > overlay_min) &&
            (det_point_set[k].vehicle_point.x < near_max)) {
          near_count_point++;
        }

        // 用检测和跟踪公共部分的点来计算距离
        if ((det_point_set[k].vehicle_point.x < overlay_min) ||
            (det_point_set[k].vehicle_point.x > overlay_max))
          continue;

        track_kdtrees_[j]->knnSearch(query_point, nearest_index, nearest_dist,
                                     dim, cv::flann::SearchParams(-1));
        // 根据阈值筛选点
        float y_dist =
            std::abs(track_point_set[nearest_index[0]].vehicle_point.y -
                     det_point_set[k].vehicle_point.y);

        if (y_dist < point_match_dis_thresh_) {
          if (det_point_set[k].vehicle_point.x > overlay_min &&
              det_point_set[k].vehicle_point.x < near_max) {
            near_dist_match_sum += y_dist;
            near_dist_match_cnt++;
            PointPair pt =
                std::make_tuple(k, nearest_index[0], nearest_dist[0]);
            point_dist_ptr->emplace_back(pt);
          } else {
            far_dist_match_sum += y_dist;
            far_dist_match_cnt++;
            PointPair pt =
                std::make_tuple(k, nearest_index[0], nearest_dist[0]);
            point_dist_ptr->emplace_back(pt);
          }
        }
      }
      near_dist_match_cnt = near_dist_match_cnt > 0 ? near_dist_match_cnt : 1;
      far_dist_match_cnt = far_dist_match_cnt > 0 ? far_dist_match_cnt : 1;
      double near_score = near_dist_match_sum / near_dist_match_cnt;
      double far_score = far_dist_match_sum / far_dist_match_cnt;

      HLOG_DEBUG << "roadedge point_match_debug timestamp: " << debug_timestamp_
                 << ", track index: " << j << ", trackId: "
                 << lane_trackers[j]->GetConstLaneTarget()->Id()
                 << ", detect index: " << i
                 << ", detectId: " << detected_lanelines->at(i)->id
                 << ", near_match_point_num: " << near_dist_match_cnt
                 << ", near_dist_match_sum: " << near_dist_match_sum
                 << ", near_average distance: " << near_score
                 << ", far_match_point_num: " << far_dist_match_cnt
                 << ", far_dist_match_sum: " << far_dist_match_sum
                 << ", far_average distance: " << far_score
                 << ", close_near_match: " << close_near_match
                 << ", overlay_min: " << overlay_min
                 << ", near_max: " << near_max;

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

      // 0: track_index, 1: detect_index
      std::get<0>(match_score_tuple_) = j;
      std::get<1>(match_score_tuple_) = i;
      std::get<2>(match_score_tuple_) = point_dist_ptr;
      std::get<3>(match_score_tuple_) = total_score;

      match_score_list_.push_back(match_score_tuple_);
    }
  }
  // 按照匹配打分排序
  std::sort(match_score_list_.begin(), match_score_list_.end(),
            [](PointMatchScoreTuple a, PointMatchScoreTuple b) {
              return std::get<3>(a) > std::get<3>(b);
            });
  // 计算二分匹配结果
  SolveBipartiteGraphMatchWithGreedy(detected_lanelines, match_score_list_,
                                     association_result);
}

bool PointMatcher::Associate(
    const PointMatcherOptions& options,
    const std::vector<perception_base::LaneLineMeasurementPtr>*
        detect_measurements,
    const std::vector<SimpleLaneTrackerPtr>& lane_trackers,
    PointAssociationResult* association_result) {
  SetTrackKDTree(lane_trackers);
  SetDetectionPointDist(detect_measurements);
  AssociationKnn(options, lane_trackers, detect_measurements,
                 association_result);
  Clear();
  return true;
}

bool PointMatcher::Associate(
    const PointMatcherOptions& options,
    const std::vector<perception_base::RoadEdgeMeasurementPtr>*
        detect_measurements,
    const std::vector<SimpleRoadEdgeTrackerPtr>& lane_trackers,
    PointAssociationResult* association_result) {
  SetTrackKDTree(lane_trackers);
  SetDetectionPointDist(detect_measurements);
  AssociationKnn(options, lane_trackers, detect_measurements,
                 association_result);
  Clear();
  return true;
}

}  // namespace environment
}  // namespace mp
}  // namespace hozon
