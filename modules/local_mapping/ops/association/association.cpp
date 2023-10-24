/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: likuan
 *Date: 2023-09-06
 *****************************************************************************/
#include "modules/local_mapping/ops/association/association.h"

namespace hozon {
namespace mp {
namespace lm {

std::unordered_map<int, int> LaneAssoc::Process(
    const std::vector<LaneLine>& lanes_det,
    const std::vector<LaneLine>& lanes_lm) {
  Clear();
  SetDetection(lanes_det);
  SetLandmark(lanes_lm);
  AssociationKnn();
  return map_det_lm_;
}

void LaneAssoc::SetDetection(const std::vector<LaneLine>& lanes_det) {
  num_det_ = static_cast<int>(lanes_det.size());
  for (size_t i = 0; i < lanes_det.size(); ++i) {
    if (lanes_det.empty()) {
      continue;
    }
    std::vector<double> distance_thd = GetDistThd(lanes_det[i].points_);
    det_xyzs_.push_back(lanes_det[i].points_);
    det_knn_thd_.push_back(distance_thd);
  }
}

void LaneAssoc::SetLandmark(const std::vector<LaneLine>& lanes_lm) {
  for (const auto& lane_line : lanes_lm) {
    lm_xyzs_.push_back(lane_line.points_);
    if (lane_line.kdtree_ == nullptr) {
      std::vector<cv::Point2f> cv_points;
      for (const auto& point : lane_line.points_) {
        if (point.x() < -0.5) {
          continue;
        }
        cv::Point2f point_tmp = {static_cast<float>(point.x()),
                                 static_cast<float>(point.y())};
        cv_points.emplace_back(point_tmp);
      }
      if (cv_points.empty()) {
        lm_kdtrees_.emplace_back(nullptr);
        continue;
      }
      cv::flann::KDTreeIndexParams index_params(1);
      std::shared_ptr<cv::flann::Index> kdtree =
          std::make_shared<cv::flann::Index>(cv::Mat(cv_points).reshape(1),
                                             index_params);
      lm_kdtrees_.push_back(kdtree);
    }
  }
  num_lm_ = static_cast<int>(lm_kdtrees_.size());
}

void LaneAssoc::AssociationKnn() {
  if (num_det_ == 0 || num_lm_ == 0) {
    return;
  }

  Matxd assoc_scores = Matxd::Zero(num_lm_, num_det_);
  Matxd assoc_dist = Matxd::Zero(num_lm_, num_det_);
  Matxd assoc_dist_thd = Matxd::Zero(num_lm_, num_det_);
  for (int i = 0; i < num_det_; ++i) {
    // HLOG_ERROR << "det id: " << i;
    std::vector<Eigen::Vector3d> det_xyz = det_xyzs_[i];
    std::vector<double> distance_thd = det_knn_thd_[i];
    for (int j = 0; j < num_lm_; ++j) {
      // HLOG_ERROR << "lm id: " << j;
      if (lm_kdtrees_[j] == nullptr) {
        HLOG_ERROR << "invalid lm kd tree";
        continue;
      }
      int dist_match_cnt = 0;
      double dist_match_sum = 0;
      double dist_sum = 0;
      for (size_t k = 0; k < det_xyzs_[i].size(); ++k) {
        if (det_xyzs_[i][k].x() > 50) {
          continue;
        }
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
      // HLOG_ERROR << "dist_match_cnt " << dist_match_cnt;
      if (dist_match_cnt == 0) {
        continue;
      }
      double score = dist_match_sum / dist_match_cnt *
                     std::sqrt(det_xyzs_[i].size() / dist_match_cnt);
      double ideal_dist = dist_sum / static_cast<double>(det_xyzs_[i].size()) *
                          std::sqrt(1 / options_.min_match_ratio);
      assoc_dist(j, i) = score;
      assoc_dist_thd(j, i) = ideal_dist;
      assoc_scores(j, i) = score < ideal_dist ? 1 / score : 0;
    }
  }
  // std::cout << assoc_scores << std::endl;
  Affinity2Assoc(assoc_scores);
}

void LaneAssoc::Affinity2Assoc(const Matxd& affinity) {
  map_det_lm_.clear();
  for (int det_id = 0; det_id < affinity.cols(); ++det_id) {
    Eigen::VectorXd::Index max_col = 0;
    affinity.col(det_id).maxCoeff(&max_col);
    int lm_id = static_cast<int>(max_col);
    if (affinity(lm_id, det_id) > 0) {
      map_det_lm_[det_id] = lm_id;
    }
  }

  std::vector<std::vector<int>> map_lm_det(affinity.rows());
  for (const auto& iter : map_det_lm_) {
    map_lm_det[iter.second].push_back(iter.first);
  }

  for (int i = 0; i < static_cast<int>(map_lm_det.size()); ++i) {
    if (map_lm_det[i].size() > 1) {
      int max_id = map_lm_det[i][0];
      double max_val = affinity(i, map_lm_det[i][0]);
      for (int j = 1; j < static_cast<int>(map_lm_det[i].size()); ++j) {
        if (affinity(i, map_lm_det[i][j]) > max_val) {
          map_det_lm_.erase(max_id);
          delete_det_lines_.insert(max_id);
          max_id = map_lm_det[i][j];
          max_val = affinity(i, map_lm_det[i][j]);
        } else {
          map_det_lm_.erase(map_lm_det[i][j]);
          delete_det_lines_.insert(map_lm_det[i][j]);
        }
      }
    }
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
    const std::vector<Eigen::Vector3d>& points) const {
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

void LaneAssoc::Clear() {
  lm_xyzs_.clear();
  det_xyzs_.clear();
  det_knn_thd_.clear();
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
