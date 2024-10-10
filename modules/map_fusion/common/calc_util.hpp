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

#include "modules/map_fusion/common/calc_util.h"

namespace hozon {
namespace mp {
namespace mf {
namespace math {

template <typename T, typename M>
T CubicResolve(const std::vector<T>& coefs, M t) {
  int order = coefs.size();
  switch (order) {
    case 4: {
      return ((coefs[3] * t + coefs[2]) * t + coefs[1]) * t + coefs[0];
    }
    case 3: {
      return (coefs[2] * t + coefs[1]) * t + coefs[0];
    }
    case 2: {
      return coefs[1] * t + coefs[0];
    }
    case 1: {
      return coefs[0];
    }
    default:
      return 0.0;
  }
}

template <typename T>
bool InRange(T x, T min_x, T max_x) {
  return (x >= min_x && x <= max_x);
}

template <typename T>
bool OutRange(T x, T min_x, T max_x) {
  return (x < min_x || x > max_x);
}

template <typename T>
bool InCubicRange(T x, const LineCubic& cubic) {
  return x >= cubic.start && x <= cubic.end;
}

// 采样三次多项式曲线
template <typename T>
void SamplingCubic(const LineCubic& cubic, float step,
                   std::vector<Eigen::Matrix<T, 3, 1>>* pts) {
  if (pts == nullptr) return;

  pts->clear();
  float x = cubic.start;
  while (x < cubic.end) {
    float y = CubicResolve(cubic.coefs, x);
    Eigen::Matrix<T, 3, 1> pt(x, y, 0);
    pts->emplace_back(pt);

    x += step;
  }
  x = cubic.end;
  float end_y = CubicResolve(cubic.coefs, x);
  pts->emplace_back(x, end_y, 0);
}

template <typename pointType>
double GetLength(const std::vector<pointType>& point_sets) {
  double total_length = 0;
  // 遍历point_sets中的点
  for (size_t i = 1; i < point_sets.size(); ++i) {
    // 计算车体系两点之间的欧式距离
    double dx = point_sets[i].x() - point_sets[i - 1].x();
    double dy = point_sets[i].y() - point_sets[i - 1].y();
    double length = std::sqrt(dx * dx + dy * dy);

    // 累加得到整个车道线的长度
    total_length += length;
  }
  return total_length;
}

template <typename pointType>
float GetOverLayRatioBetweenTwoLane(const std::vector<pointType>& curve1,
                                    const std::vector<pointType>& curve2) {
  auto length1 = GetLength(curve1);
  auto length2 = GetLength(curve2);

  auto overlay_start = std::max(curve1.front().x(), curve2.front().x());
  auto overlay_end = std::min(curve1.back().x(), curve2.back().x());
  auto overlay_length =
      overlay_end - overlay_start > 0 ? overlay_end - overlay_start : 0;

  return overlay_length / std::min(length1, length2);
}

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
bool ComputerLineDis(const std::vector<PointType>& line_pts,
                     const std::vector<PointType>& right_line_pts,
                     double* avg_width, int pts_interval) {
  return ComputerLineDis(line_pts, right_line_pts, nullptr, avg_width,
                         pts_interval);
}

template <typename PointType>
bool ComputerLineDis(const std::vector<PointType>& line_pts,
                     const std::vector<PointType>& right_line_pts,
                     std::vector<double>* line_dis, double* avg_width,
                     int pts_interval) {
  // 计算线与线之间距离
  if (line_pts.empty() || right_line_pts.empty()) {
    return false;
  }
  int count_num = 0;
  bool flag = false;
  double original_val = *avg_width;
  *avg_width = 0.0;
  for (int index = 0; index < static_cast<int>(line_pts.size());
       index += pts_interval) {
    const auto& P = line_pts[index];
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
      auto dis = (P - C).norm();
      *avg_width += dis;
      if (line_dis != nullptr) {
        line_dis->emplace_back(dis);
      }
      count_num++;
      break;
    }
  }
  if (count_num > 0) {
    flag = true;
    *avg_width /= count_num;
  } else {
    flag = false;
    *avg_width = original_val;
  }
  return flag;
}

}  // namespace math

}  // namespace mf
}  // namespace mp
}  // namespace hozon
