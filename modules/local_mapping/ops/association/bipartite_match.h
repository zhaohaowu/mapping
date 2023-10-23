/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: chenlongxi
 *Date: 2023-10-09
 *****************************************************************************/
#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "modules/local_mapping/types/common.h"
#include "modules/local_mapping/types/types.h"
#include "modules/local_mapping/utils/common.h"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d.hpp"

namespace hozon {
namespace mp {
namespace lm {

// track_index, detect_lane_index, point_indexs_in_lane, match_score
typedef std::tuple<size_t, size_t, std::vector<size_t>, float> MatchScoreTuple;

struct BipartiteAssocParams {
  int min_overlap_point_num;
  int min_match_point_num;
  float same_group_max_dist;
  float match_score_threshold;
};

struct LaneAssociationResult {
  std::vector<MatchScoreTuple> assignments;
  std::vector<size_t> unassigned_tracks;
  std::vector<size_t> unsigned_objects;
};

struct BipartiteLaneAssocOptions {
  double yaw_std = 0.035;
  double trans_std = 0.04;
  double xyz_std = 0.4;
  int dim = 2;
  double min_match_ratio = 0.5;
  BipartiteAssocParams params;

  explicit BipartiteLaneAssocOptions(BipartiteAssocParams params)
      : params(params) {}
  BipartiteLaneAssocOptions(const BipartiteLaneAssocOptions& options)
      : yaw_std(options.yaw_std),
        trans_std(options.trans_std),
        xyz_std(options.xyz_std),
        dim(options.dim),
        min_match_ratio(options.min_match_ratio),
        params(options.params) {}
};

class BipartiteLaneAssoc {
 public:
  explicit BipartiteLaneAssoc(const BipartiteLaneAssocOptions& options)
      : options_(options) {}
  ~BipartiteLaneAssoc() {}

 private:
  BipartiteLaneAssocOptions options_;

  int num_lm_ = 0;
  std::vector<std::vector<Eigen::Vector3d>> lm_xyzs_;

  int num_det_ = 0;
  std::vector<std::vector<Eigen::Vector3d>> det_xyzs_;
  std::vector<std::vector<Eigen::Vector3d>> vehicle_det_xyzs_;
  std::vector<std::vector<double>> det_knn_thd_;

  std::vector<MatchScoreTuple> match_score_list_;

  std::vector<bool> target_used_mask_;
  std::vector<bool> det_used_mask_;

  void SolveBipartiteGraphMatchWithGreedy(
      const std::vector<MatchScoreTuple>& match_score_list, size_t targets_size,
      size_t objects_size);

  void SetDetection(const std::vector<LanePointsPtr>& lanes_det,
                    const Vec3d& pose_ab);
  bool Association(const std::vector<std::vector<Eigen::Vector3d>>& lanes_det,
                   std::vector<LocalMapLane>* lanes_lm);

  std::vector<double> GetDistThd(const std::vector<Eigen::Vector3d>& points);
  std::vector<Eigen::Vector3d> TranformPoints(
      const Vec3d& pose_ab, const std::vector<Eigen::Vector3d>& points);

 public:
  std::unordered_map<int, int> map_det_lm_;

  std::unordered_map<int, int> Process(
      const std::vector<std::vector<Eigen::Vector3d>>& lanes_det,
      std::vector<LocalMapLane>* lanes_lm);

  void Clear();
};

typedef std::shared_ptr<BipartiteLaneAssoc> BipartiteLaneAssocPtr;
typedef std::shared_ptr<const BipartiteLaneAssoc> BipartiteLaneAssocConstPtr;

}  // namespace lm
}  // namespace mp
}  // namespace hozon
