/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： frechet_distance.cc
 *   author     ： Nihongjie
 *   date       ： 2023.12
 ******************************************************************************/
#include "modules/location/pose_estimation/lib/pose_estimate/frechet_distance.h"

#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <unordered_set>

namespace hozon {
namespace mp {
namespace loc {
FrechetDistance3D::FrechetDistance3D() {}

FrechetDistance3D::~FrechetDistance3D() {}

double FrechetDistance3D::frechetDistance(
    const hozon::mp::loc::LaneLinePerceptionPtr& fil_line,
    const std::vector<hozon::mp::loc::V3>& P2,
    std::vector<hozon::mp::loc::PointMatchPair>* const p_q, const SE3& T_V_W) {
  if (std::min(fil_line->points().size(), P2.size()) <= 0) {
    return -1.0;
  }
  HLOG_DEBUG << "fil_line->points().size = " << fil_line->points().size()
            << " , map_points.size = " << P2.size();
  std::vector<hozon::mp::loc::V3> p0, p1;
  double result = -1, ratio;
  // calculate frechet distance matrix
  std::vector<std::vector<double>> frechet_dis_matrix;
  double f_max = std::numeric_limits<double>::min();
  double f_min = std::numeric_limits<double>::max();
  double distance = 0;
  std::vector<double> frechet_dis_row;
  std::unordered_multiset<int> filter_ids;

  for (const auto& p : fil_line->points()) {  // 感知点
    double p_q_distance = std::numeric_limits<double>::max();
    hozon::mp::loc::V3 tmp_q;
    auto line_id = 0;
    for (const auto& q : P2) {  // 地图点
      if (filter_ids.find(line_id) != filter_ids.end()) {
        continue;
      }
      distance = (p - T_V_W * q).norm();
      if (distance < f_min) {
        f_min = distance;
      }
      if (distance > f_max) {
        f_max = distance;
      }
      if (p_q_distance > distance) {
        tmp_q = q;
        p_q_distance = distance;
      }
      HLOG_DEBUG << "percep_point: " << p.x() << ", " << p.y() << ", " << p.z()
                << " | map_point[" << line_id << "]:" << q.x() << ", " << q.y()
                << ", " << q.z() << " | cal_distance: " << distance
                << " | std::min(1.5, distance): " << std::min(1.5, distance);
      filter_ids.emplace(line_id);
      distance = std::min(1.5, distance);
      frechet_dis_row.emplace_back(distance);
      ++line_id;
    }
    frechet_dis_matrix.emplace_back(frechet_dis_row);
    frechet_dis_row.clear();
    if (p_q_distance ==
        std::numeric_limits<double>::max()) {  // 该感知点未找到最近的地图点
      continue;
    }
    auto weight = 1;
    if (fabs(fil_line->lane_position_type()) == 2) {
      weight *= 0.75;
    }
    hozon::mp::loc::PercepLineType cur_type =
        static_cast<hozon::mp::loc::PercepLineType>(
            fil_line->lane_position_type());
    p_q->emplace_back(p, tmp_q, cur_type,
                      weight);  // 找到最匹配的点 加入match_pairs中
  }
    HLOG_DEBUG << "f_min = " << f_min << ", f_max = " << f_max;

  // compute frechet distance
  double res = (f_max - f_min) / 10;

  // get region number of beginning and end points
  std::vector<std::vector<int>> frechet_compare_matrix;
  for (double range = f_min; range < f_max; range += res) {
    int mode = 0;  // bwlabel = 0, astar = 1;
    HLOG_DEBUG << "frechet_dis_matrix row size = " << frechet_dis_matrix.size()
              << " , col size = " << frechet_dis_matrix[0].size();
    compareMatrix(frechet_dis_matrix, &frechet_compare_matrix, range, mode);
    if (frechet_compare_matrix.empty()) {
      continue;
    }
    if (frechet_compare_matrix[0].empty()) {
      continue;
    }
    HLOG_DEBUG << "frechet_compare_matrix row size = "
              << frechet_compare_matrix.size()
              << " , col size = " << frechet_compare_matrix[0].size();
    hozon::mp::loc::BwLbel bwlabel_process;
    std::vector<std::vector<int>> frechet_group_mat =
        bwlabel_process.bwlabel(frechet_compare_matrix);
    if (frechet_group_mat.empty()) {
      continue;
    }
    if (frechet_group_mat.front().empty()) {
      continue;
    }
    if (frechet_group_mat[0][0] != 0 &&
        frechet_group_mat[0][0] == (frechet_group_mat.back()).back()) {
      result = range;
      break;
    }
    frechet_compare_matrix.clear();
  }
  if (result == -1) {
    result = f_max;
  }
  ratio = getRatio(result, f_max, f_min);
  if (ratio < 0.8) {
    p_q->clear();
  }
  HLOG_DEBUG << "ratio = " << ratio;
  return ratio;
}

void FrechetDistance3D::compareMatrix(
    const std::vector<std::vector<double>>& origin_matrix,
    std::vector<std::vector<int>>* const result_matrix, const double& range,
    const int& mode) {
  std::vector<int> frechet_compare_row;
  for (size_t index_row = 0; index_row < origin_matrix.size(); ++index_row) {
    for (size_t index_column = 0;
         index_column < origin_matrix[index_row].size(); ++index_column) {
      if (origin_matrix[index_row][index_column] <= range) {
        if (mode == 0) {
          frechet_compare_row.emplace_back(1);
        } else if (mode == 1) {
          frechet_compare_row.emplace_back(0);
        }
      } else {
        if (mode == 0) {
          frechet_compare_row.emplace_back(0);
        } else if (mode == 1) {
          frechet_compare_row.emplace_back(1);
        }
      }
    }
    result_matrix->emplace_back(frechet_compare_row);
    frechet_compare_row.clear();
  }
}

double FrechetDistance3D::getRatio(double result, double f_max, double f_min) {
  if (f_max > 1) {
    f_max = 1000;
  }
  double probablity = (1 - normcdf((result - f_min) / (f_max / 3))) * 2;
  return probablity;
}

double FrechetDistance3D::normcdf(double x) {
  // constants
  double a1 = 0.254829592;
  double a2 = -0.284496736;
  double a3 = 1.421413741;
  double a4 = -1.453152027;
  double a5 = 1.061405429;
  double p = 0.3275911;

  // Save the sign of x
  int sign = 1;
  if (x < 0) {
    sign = -1;
  }
  x = fabs(x) / sqrt(2.0);

  // A&S formula 7.1.26
  double t = 1.0 / (1.0 + p * x);
  double y =
      1.0 - (((((a5 * t + a4) * t) + a3) * t + a2) * t + a1) * t * exp(-x * x);

  return 0.5 * (1.0 + sign * y);
}
}  // namespace loc
}  // namespace mp
}  // namespace hozon
