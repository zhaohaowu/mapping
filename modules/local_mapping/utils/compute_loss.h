/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: shenliangchen
 *Date: 2023-09-25
 *****************************************************************************/
#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <vector>

#include "modules/local_mapping/types/common.h"
#include "modules/local_mapping/utils/common.h"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d.hpp"

namespace hozon {
namespace mp {
namespace lm {
class Loss {
 public:
  float Process(const std::vector<Eigen::Vector3d>& hq_pts,
                const std::vector<LocalMapLane>& lanes,
                float dist_threshold = 1.0);

 private:
  float MaxDistance(cv::flann::Index* kdtree, float x, float y);
};
}  // namespace lm
}  // namespace mp
}  // namespace hozon
