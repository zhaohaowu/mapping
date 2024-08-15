/******************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       : calc_util.hpp
 *   author     : chenlongxi
 *   date       : 2023.10
 ******************************************/

#pragma once

#include <algorithm>
#include <numeric>
#include <vector>

#include "modules/map_fusion_02/common/calc_util.h"

namespace hozon {
namespace mp {
namespace mf {
namespace math {

template <typename T1, typename T2>
float evaluateHeadingDiff(const T1& x, const std::vector<T2>& params) {
  CHECK_EQ(params.size(), 4);
  float sum = 0.0;
  float val = 1.0;
  // 二阶导
  if (params.size() == 4) {
    sum = 6.0 * params[3] * x + 2 * params[2];
  }
  return sum;
}

template <typename PointType>
void ComputerLineDis(const std::vector<PointType>& line_pts,
                     const std::vector<PointType>& right_line_pts,
                     std::vector<double>* line_dis) {
  // 计算线与线之间距离
  if (line_pts.empty() || right_line_pts.empty()) {
    return;
  }
  std::vector<double> val_dis;
  for (const auto& P : line_pts) {
    for (size_t i = 1; i < right_line_pts.size(); i++) {
      const auto& A = right_line_pts[i - 1];
      const auto& B = right_line_pts[i];
      const auto& AB = B - A;
      const auto& AP = P - A;
      double ABLengthSquared = AB.squaredNorm();
      if (std::fabs(ABLengthSquared) < 1e-6) {
        continue;
      }
      double t = AB.dot(AP) / ABLengthSquared;
      if (t < 0 || t > 1) {
        continue;
      }
      t = std::max(0.0, std::min(t, 1.0));
      auto C = A + t * AB;  // 点到线段的最近点
      val_dis.emplace_back((P - C).norm());
      break;
    }
  }
  double sum = std::accumulate(val_dis.begin(), val_dis.end(), 0.0);
  double mean = sum / static_cast<double>(val_dis.size());
  *line_dis = val_dis;
}

}  // namespace math

}  // namespace mf
}  // namespace mp
}  // namespace hozon
