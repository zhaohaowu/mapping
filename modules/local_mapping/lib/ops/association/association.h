/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: likuan
 *Date: 2023-09-06
 *****************************************************************************/
#pragma once
#include <Eigen/Core>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "modules/local_mapping/lib/types/common.h"
#include "modules/local_mapping/lib/types/types.h"
#include "modules/local_mapping/lib/utils/common.h"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d.hpp"
#include "modules/util/include/util/temp_log.h"

namespace hozon {
namespace mp {
namespace lm {

struct LaneAssocOptions {
  double yaw_std = 0.035;
  double trans_std = 0.04;
  double xyz_std = 0.4;
  int dim = 2;
  double min_match_ratio = 0.5;
  bool use_consistency = true;

  LaneAssocOptions() {}
  LaneAssocOptions(const LaneAssocOptions& options)
      : yaw_std(options.yaw_std),
        trans_std(options.trans_std),
        xyz_std(options.xyz_std),
        dim(options.dim),
        min_match_ratio(options.min_match_ratio),
        use_consistency(options.use_consistency) {}
};

class LaneAssoc {
 public:
  explicit LaneAssoc(const LaneAssocOptions& options) : options_(options) {}
  ~LaneAssoc() {}

 private:
  LaneAssocOptions options_;

  int num_lm_ = 0;
  std::vector<std::vector<Eigen::Vector3d>> lm_xyzs_;
  std::vector<cv::flann::Index*> lm_kdtrees_;

  int num_det_ = 0;
  std::vector<std::vector<Eigen::Vector3d>> det_xyzs_;
  std::vector<std::vector<double>> det_knn_thd_;

 public:
  std::unordered_map<int, int> map_det_lm_;
  std::unordered_set<int> delete_det_lines_;

  std::unordered_map<int, int> Process(
      const std::vector<LanePointsPtr>& lanes_det,
      const std::vector<LocalMapLane>& lanes_lm, const Vec3d& pose_ab);

  void SetDetection(const std::vector<LanePointsPtr>& lanes_det,
                    const Vec3d& pose_ab);
  void SetLandmark(const std::vector<LocalMapLane>& lanes_lm);
  void Association();

  void AssociationKnn();
  void Affinity2Assoc(const Matxd& affinity);
  std::vector<double> GetDistThd(const std::vector<Eigen::Vector3d>& points);
  std::vector<Eigen::Vector3d> TranformPoints(
      const Vec3d& pose_ab, const std::vector<Eigen::Vector3d>& points);
  Matxd ConstructConsistency(const Matxd& affinity);
  bool LeftOrRight(const Matxd& lane_a, const Matxd& lane_b);
  void Clear();
};

typedef std::shared_ptr<LaneAssoc> LaneAssocPtr;
typedef std::shared_ptr<const LaneAssoc> LaneAssocConstPtr;

}  // namespace lm
}  // namespace mp
}  // namespace hozon
