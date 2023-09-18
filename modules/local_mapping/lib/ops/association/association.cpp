/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: likuan
 *Date: 2023-09-06
 *****************************************************************************/
#include "modules/local_mapping/lib/ops/association/association.h"

#include <algorithm>
#include <vector>

namespace hozon {
namespace mp {
namespace lm {

std::unordered_map<int, int> LaneAssoc::Process(
    const std::vector<LanePointsPtr>& lanes_det,
    const std::vector<LocalMapLane>& lanes_lm, const Vec3d& pose_ab) {
  Clear();
  SetDetection(lanes_det, pose_ab);
  SetLandmark(lanes_lm);
  AssociationKnn();
  return map_det_lm_;
}

void LaneAssoc::SetDetection(const std::vector<LanePointsPtr>& lanes_det,
                             const Vec3d& pose_ab) {
  num_det_ = lanes_det.size();
  for (size_t i = 0; i < lanes_det.size(); ++i) {
    if (lanes_det.size() == 0) continue;
    std::vector<double> distance_thd = GetDistThd(*lanes_det[i]);
    std::vector<Eigen::Vector3d> trans_xyz =
        TranformPoints(pose_ab, *lanes_det[i]);
    det_xyzs_.push_back(trans_xyz);
    det_knn_thd_.push_back(distance_thd);
  }
}

void LaneAssoc::SetLandmark(const std::vector<LocalMapLane>& lanes_lm) {
  num_lm_ = lanes_lm.size();
  for (size_t i = 0; i < lanes_lm.size(); ++i) {
    if (lanes_lm[i].points_.size() == 0) continue;
    lm_xyzs_.push_back(lanes_lm[i].points_);
    if (lanes_lm[i].kdtree_ == nullptr) {
      std::vector<cv::Point2f> cv_points;
      for (size_t j = 0; j < lanes_lm[i].points_.size(); ++j) {
        cv_points.push_back(
            cv::Point2f(static_cast<float>(lanes_lm[i].points_[j].x()),
                        static_cast<float>(lanes_lm[i].points_[j].y())));
      }
      cv::flann::KDTreeIndexParams index_params(1);
      cv::flann::Index* kdtree =
          new cv::flann::Index(cv::Mat(cv_points).reshape(1), index_params);
      lm_kdtrees_.push_back(kdtree);
    }
  }
}

void LaneAssoc::AssociationKnn() {
  if (num_det_ == 0 || num_lm_ == 0) return;

  Matxd assoc_scores = Matxd::Zero(num_lm_, num_det_);
  Matxd assoc_dist = Matxd::Zero(num_lm_, num_det_);
  Matxd assoc_dist_thd = Matxd::Zero(num_lm_, num_det_);

  for (int i = 0; i < num_det_; ++i) {
    std::vector<Eigen::Vector3d> det_xyz = det_xyzs_[i];
    std::vector<double> distance_thd = det_knn_thd_[i];
    for (int j = 0; j < num_lm_; ++j) {
      int dist_match_cnt = 0;
      double dist_match_sum = 0;
      double dist_sum = 0;
      for (size_t k = 0; k < det_xyzs_[i].size(); ++k) {
        const int dim = 1;
        std::vector<int> nearest_index(dim);
        std::vector<float> nearest_dist(dim);

        std::vector<float> query_points =
            std::vector<float>{static_cast<float>(det_xyzs_[i][k].x()),
                               static_cast<float>(det_xyzs_[i][k].y())};
        lm_kdtrees_[j]->knnSearch(query_points, nearest_index, nearest_dist,
                                  dim, cv::flann::SearchParams(-1));

        if (std::sqrt(nearest_dist[0]) < det_knn_thd_[i][k]) {
          dist_match_sum += std::sqrt(nearest_dist[0]);
          dist_match_cnt++;
        }
        dist_sum += det_knn_thd_[i][k];
      }
      if (dist_match_cnt == 0) continue;
      double score = dist_match_sum / dist_match_cnt *
                     std::sqrt(det_xyzs_[i].size() / dist_match_cnt);
      double ideal_dist = dist_sum / det_xyzs_[i].size() *
                          std::sqrt(1 / options_.min_match_ratio);
      assoc_dist(j, i) = score;
      assoc_dist_thd(j, i) = ideal_dist;
      assoc_scores(j, i) = score < ideal_dist ? 1 / score : 0;
    }
  }

  Affinity2Assoc(assoc_scores);

  return;
}

void LaneAssoc::Affinity2Assoc(const Matxd& affinity) {
  map_det_lm_.clear();
  for (int i = 0; i < affinity.cols(); ++i) {
    Eigen::VectorXd::Index max_col;
    affinity.col(i).maxCoeff(&max_col);
    int lane_id = static_cast<int>(max_col);
    if (affinity(lane_id, i) > 0) map_det_lm_[i] = lane_id;
  }
}

void LaneAssoc::Association() {
  //   if (num_det_ == 0 || num_lm_ == 0) return;

  //   Matxd assoc_scores = Matxd::Zero(num_lm_, num_det_);
  //   Matxd assoc_dist = Matxd::Zero(num_lm_, num_det_);
  //   Matxd assoc_dist_thd = Matxd::Zero(num_lm_, num_det_);

  //   for (int i = 0; i < num_det_; ++i) {
  //     Matxd distance_thd = det_knn_thd_[i];
  //     std::vector<double> dist_candidates;
  //     for (int j = 0; j < num_lm_; ++j) {
  //       if (lm_categories_[j] != det_categories_[i]) continue;

  //       int dist_match_cnt = 0;
  //       double dist_match_sum = 0;
  //       double dist_sum = 0;
  //       for (int k = 0; k < det_xyzs_[i].rows(); ++k) {
  //         const int dim = 1;
  //         std::vector<int> nearest_index(dim);
  //         std::vector<float> nearest_dist(dim);

  //         std::vector<float> query_points =
  //             std::vector<float>{det_xyzs_[i][k].x, det_xyzs_[i][k].y};
  //         lm_kdtrees_[j]->knnSearch(query_points, nearest_index,
  //         nearest_dist,
  //                                   dim, cv::flann::SearchParams(-1));
  //         if (std::sqrt(nearest_dist[0]) < distance_thd(k, 0)) {
  //           dist_match_sum += std::sqrt(nearest_dist[0]);
  //           dist_match_cnt++;
  //         }
  //         dist_sum += distance_thd(k, 0);
  //       }
  //       if (dist_match_cnt == 0) continue;

  //       double score = dist_match_sum / dist_match_cnt *
  //                      std::sqrt(det_xyzs_[i].size() / dist_match_cnt);
  //       double ideal_dist = dist_sum / det_xyzs_[i].size() *
  //                           std::sqrt(1 / options_.min_match_ratio);
  //       assoc_dist(j, i) = score;
  //       assoc_dist_thd(j, i) = ideal_dist;
  //       assoc_scores(j, i) = score < ideal_dist ? 1 / score : 0;
  //     }
  //   }

  //   if (options_.use_consistency && num_lm > 2 && num_det > 2) {
  //   }
}

std::vector<double> LaneAssoc::GetDistThd(
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

std::vector<Eigen::Vector3d> LaneAssoc::TranformPoints(
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

// Matxd LaneAssoc::ConstructConsistency(const Matxd& affinity) {
// Matxd A_consistency = Matxd::Zero(num_lm_, num_det_);
// std::vector<std::pair<int, int>> A;
// for (int i = 0; i < num_lm_; ++i) {
//   for (int j = 0; j < num_det_; ++j) {
//     if (affinity(i, j) > 0) A.push_back({i, j});
//   }
// }

// int w = 1;
// for (const auto& aij : A) {
//   int i = aij.first;
//   int j = aij.second;
//   for (const auto& bkl : A) {
//     int k = bkl.first;
//     int l = bkl.second;
//     if (i == k || j == l) {
//       A_consistency(i, j) += w;
//     } else {
//     }
//   }
// }
// }

bool LaneAssoc::LeftOrRight(const Matxd& lane_a, const Matxd& lane_b) {
  return true;
}

void LaneAssoc::Clear() {
  lm_xyzs_.clear();
  det_xyzs_.clear();
  det_knn_thd_.clear();

  for (size_t i = 0; i < lm_kdtrees_.size(); ++i) {
    delete lm_kdtrees_[i];
  }
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
