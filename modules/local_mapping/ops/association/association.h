/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: likuan
 *Date: 2023-09-06
 *****************************************************************************/
#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "modules/local_mapping/types/types.h"
#include "modules/local_mapping/utils/common.h"
#include "modules/util/include/util/mapping_log.h"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d.hpp"
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
};

class LaneAssoc {
 public:
  explicit LaneAssoc(const LaneAssocOptions& options) : options_(options) {}
  std::unordered_map<int, int> Process(const std::vector<LaneLine>& lanes_det,
                                       const std::vector<LaneLine>& lanes_lm);
  void SetDetection(const std::vector<LaneLine>& lanes_det);
  void SetLandmark(const std::vector<LaneLine>& lanes_lm);
  void Association();
  void AssociationKnn();
  void Affinity2Assoc(const Matxd& affinity);
  std::vector<double> GetDistThd(
      const std::vector<Eigen::Vector3d>& points) const;
  Matxd ConstructConsistency(const Matxd& affinity);
  void Clear();
  bool NeedDelete(const int& i) {
    return delete_det_lines_.find(i) != delete_det_lines_.end();
  }

 private:
  LaneAssocOptions options_;
  int num_lm_ = 0;
  std::vector<std::vector<Eigen::Vector3d>> lm_xyzs_;
  std::vector<std::shared_ptr<cv::flann::Index>> lm_kdtrees_;
  int num_det_ = 0;
  std::vector<std::vector<Eigen::Vector3d>> det_xyzs_;
  std::vector<std::vector<double>> det_knn_thd_;
  std::unordered_map<int, int> map_det_lm_;
  std::unordered_set<int> delete_det_lines_;
};

using LaneAssocPtr = std::shared_ptr<LaneAssoc>;

}  // namespace lm
}  // namespace mp
}  // namespace hozon
