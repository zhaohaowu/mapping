/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： frechet_distance.h
 *   author     ： Nihongjie
 *   date       ： 2023.12
 ******************************************************************************/
#include <math.h>

#include <iostream>
#include <vector>

#include "algorithm"
#include "modules/location/pose_estimation/lib/pose_estimate/bw_label.h"
#include "modules/location/pose_estimation/lib/pose_estimate/pose_estimate_base.h"

namespace hozon {
namespace mp {
namespace loc {
class FrechetDistance3D {
 public:
  FrechetDistance3D();
  ~FrechetDistance3D();
  double frechetDistance(
      const hozon::mp::loc::LaneLinePerceptionPtr& fil_line,
      const std::vector<hozon::mp::loc::V3>& P2,
      std::vector<hozon::mp::loc::PointMatchPair>* const p_q);

 private:
  void compareMatrix(const std::vector<std::vector<double>>& origin_matrix,
                     std::vector<std::vector<int>>* const result_matrix,
                     const double& range, const int& mode);
  double getRatio(double result, double f_max, double f_min);

 private:
  double normcdf(double x);
};

}  // namespace loc
}  // namespace mp
}  // namespace hozon
