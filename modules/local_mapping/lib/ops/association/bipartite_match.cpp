/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: chenlongxi
 *Date: 2023-10-09
 *****************************************************************************/
#include "modules/local_mapping/lib/ops/association/bipartite_match.h"

#include <algorithm>
#include <vector>

namespace hozon {
namespace mp {
namespace lm {

std::unordered_map<int, int> BipartiteLaneAssoc::Process(
    const std::vector<LanePointsPtr>& lanes_det,
    const std::vector<LocalMapLane>& lanes_lm, const Vec3d& pose_ab) {
  Clear();
  SetDetection(lanes_det, pose_ab);
  Association(lanes_lm);
  return map_det_lm_;
}

void BipartiteLaneAssoc::SetDetection(
    const std::vector<LanePointsPtr>& lanes_det, const Vec3d& pose_ab) {
  num_det_ = lanes_det.size();
  for (size_t i = 0; i < lanes_det.size(); ++i) {
    if (lanes_det.size() == 0) continue;
    std::vector<double> distance_thd = GetDistThd(*lanes_det[i]);
    std::vector<Eigen::Vector3d> trans_xyz =
        TranformPoints(pose_ab, *lanes_det[i]);
    det_xyzs_.push_back(trans_xyz);
    vehicle_det_xyzs_.push_back(*lanes_det[i]);
    det_knn_thd_.push_back(distance_thd);
  }
}

std::vector<Eigen::Vector3d> BipartiteLaneAssoc::TranformPoints(
    const Vec3d& pose_ab, const std::vector<Eigen::Vector3d>& points) {
  Mat3d trans_mat = CommonUtil::Se2Vector2Matrix(pose_ab);

  std::vector<Eigen::Vector3d> points_w;
  points_w.reserve(points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    Vec3d point(points[i].x(), points[i].y(), 1);
    Vec3d point_w = trans_mat * point;
    points_w.emplace_back(Eigen::Vector3d(point_w.x(), point_w.y(), 0));
  }

  return points_w;
}

std::vector<double> BipartiteLaneAssoc::GetDistThd(
    const std::vector<Eigen::Vector3d>& points) {
  std::vector<double> dist_thds(points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    double pt_range = points[i].norm();
    double yaw_thd = options_.yaw_std;
    double upper_thd_R = 2 * pt_range * sin(yaw_thd / 180 * M_PI / 2);
    double upper_thd_t = options_.trans_std * 2;
    double upper_thd_xyz = options_.xyz_std * std::pow(2, options_.dim);
    double upper_thd = upper_thd_R + upper_thd_t + upper_thd_xyz;
    upper_thd = std::max(upper_thd, 1.0);
    dist_thds[i] = upper_thd;
  }

  return dist_thds;
}

bool BipartiteLaneAssoc::Association(
    const std::vector<LocalMapLane>& lanes_lm) {
  if (lanes_lm.empty()) {
    return false;
  }
  num_lm_ = lanes_lm.size();
  // 计算匹配分
  for (size_t i = 0; i < num_lm_; ++i) {
    const LocalMapLane& map_lane = lanes_lm[i];
    for (size_t j = 0; j < num_det_; ++j) {
      const auto& lane_points = vehicle_det_xyzs_[j];
      float count = 0, valid_count = 0, size_point = lane_points.size();
      MatchScoreTuple match_score_tuple;
      for (size_t k = 0; k < size_point;) {
        double x_pos = lane_points[k].x(), y_pos = lane_points[k].y();
        double eval_value = 0.0;
        if (map_lane.eval_vehicle(x_pos, &eval_value)) {
          count++;
          if (fabs(y_pos - eval_value) < options_.params.same_group_max_dist) {
            valid_count++;
          }
        }
        k += 4;
      }
      if (count < options_.params.min_overlap_point_num ||
          valid_count < options_.params.min_match_point_num) {
        continue;
      }
      float match_score = count / (size_point + 1) * 0.1 +
                          valid_count / (count + 1) * 0.6 +
                          valid_count / (size_point + 1) * 0.3;
      if (match_score < options_.params.match_score_threshold) {
        continue;
      }
      std::get<0>(match_score_tuple) = i;
      std::get<1>(match_score_tuple) = j;
      std::get<3>(match_score_tuple) = match_score;
      match_score_list_.emplace_back(match_score_tuple);
    }
  }
  // 按照匹配打分排序
  std::sort(match_score_list_.begin(), match_score_list_.end(),
            [](MatchScoreTuple a, MatchScoreTuple b) {
              return std::get<3>(a) > std::get<3>(b);
            });
  // 计算二分匹配结果
  SolveBipartiteGraphMatchWithGreedy(match_score_list_, num_lm_, num_det_);
  return true;
}

void BipartiteLaneAssoc::SolveBipartiteGraphMatchWithGreedy(
    const std::vector<MatchScoreTuple>& match_score_list, size_t targets_size,
    size_t objects_size) {
  HLOG_ERROR << "bipartite size: " << targets_size << ", " << objects_size;
  target_used_mask_.resize(targets_size);
  target_used_mask_.assign(target_used_mask_.size(), false);
  det_used_mask_.resize(objects_size);
  det_used_mask_.assign(det_used_mask_.size(), false);

  LaneAssociationResult association_result;

  for (auto& score_tuple : match_score_list) {
    size_t lm_index = std::get<0>(score_tuple);
    size_t det_index = std::get<1>(score_tuple);

    if (target_used_mask_.at(lm_index) || det_used_mask_.at(det_index)) {
      continue;
    }
    association_result.assignments.push_back(score_tuple);
    det_used_mask_.at(det_index) = true;
    target_used_mask_.at(lm_index) = true;
    map_det_lm_[det_index] = lm_index;
  }

    // assign unassigned_obj_inds
  for (size_t i = 0; i < objects_size; ++i) {
    if (!det_used_mask_.at(i)) {
      association_result.unsigned_objects.push_back(i);
    }
  }

  // assign unassigned_track_inds
  for (size_t i = 0; i < targets_size; ++i) {
    if (!target_used_mask_.at(i)) {
      association_result.unassigned_tracks.push_back(i);
    }
  }

  HLOG_ERROR << "Greedy AssociationResult: matched_pair_num:"
             << association_result.assignments.size()
             << ", unmatch tracker_num:"
             << association_result.unassigned_tracks.size()
             << ", unmatch detect_num:"
             << association_result.unsigned_objects.size();
}

void BipartiteLaneAssoc::Clear() {
  lm_xyzs_.clear();
  det_xyzs_.clear();
  vehicle_det_xyzs_.clear();
  det_knn_thd_.clear();
  map_det_lm_.clear();
  match_score_list_.clear();
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
