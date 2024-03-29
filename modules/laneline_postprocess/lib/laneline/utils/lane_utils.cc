// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: Hozon Inc.
// @file: lane_utils.cc
// @brief: lane utils

// #include "modules/laneline_postprocess/lib/laneline/utils/lane_utils.h"

// #include <3rd_party/math-operation/core/i_basic.h>
// #include <3rd_party/math-operation/core/i_blas.h>

// #include "camera/common/common_flags.h"
// #include "camera/common/common_utils.h"
// #include "camera/common/laneline_quality_evaluator.h"
// #include "camera/lib/lane/common/common_functions.h"
#include "modules/laneline_postprocess/lib/laneline/utils/lane_utils.h"
#include <float.h>

#include <algorithm>
#include <numeric>
#include <utility>
// #include
// "modules/laneline_postprocess/lib/laneline/tracker/bev_lane_tracker/data_fusion/laneline_quality_filter.h"
#include "perception-base/base/utils/log.h"
// #include "perception-common/common/traits/comparison_traits.h"

namespace hozon {
namespace mp {
namespace environment {

// using common::Equal;

bool IsForkOrConverge(const LaneLineSceneType& scene_type) {
  return scene_type == LaneLineSceneType::FORK ||
         scene_type == LaneLineSceneType::CONVERGE;
}

bool IsForkOrConvergePair(const LaneLineSceneType& scene_type1,
                          const LaneLineSceneType& scene_type2) {
  return IsForkPair(scene_type1, scene_type2) ||
         IsConvergePair(scene_type1, scene_type2);
}

bool IsForkPair(const LaneLineSceneType& scene_type1,
                const LaneLineSceneType& scene_type2) {
  return (scene_type1 == LaneLineSceneType::FORK ||
          scene_type2 == LaneLineSceneType::FORK);
}

bool IsConvergePair(const LaneLineSceneType& scene_type1,
                    const LaneLineSceneType& scene_type2) {
  return (scene_type1 == LaneLineSceneType::CONVERGE ||
          scene_type2 == LaneLineSceneType::CONVERGE);
}

void GetRefValueForLineCurve(const LaneLineCurve& curve, float* d,
                             float ref_min, float ref_length, int sample_num) {
  if (d == nullptr) {
    return;
  }
  float ref_max_lane = ref_min + ref_length;
  float ref_min_lane = ref_min;
  if (curve.min > ref_min_lane || curve.max < ref_max_lane) {
    ref_min_lane = curve.min;
    ref_max_lane = std::min(curve.min + ref_length, curve.max);
  }
  float sample_interval = (ref_max_lane - ref_min_lane) / sample_num;
  *d = 0.0f;
  for (float y = ref_min_lane; y < ref_max_lane; y += sample_interval) {
    for (int i = 0; i < curve.coeffs.size(); ++i) {
      *d += curve.coeffs[i] * powf(y, i);
    }
  }
  *d = *d / sample_num;
  return;
}

void GetRefValueForLinePointSet(const std::vector<LaneLinePoint>& point_set,
                                float* d) {
  if (d == nullptr) {
    return;
  }
  *d = 0.0f;
  // 拿前20%的点计算横向位置
  int key_point_nums = point_set.size() / 5;
  for (int i = 0; i < key_point_nums; ++i) {
    if (point_set[i].vehicle_point.x < 0.0) {
      continue;
    }
    *d += point_set[i].vehicle_point.y;
  }

  *d = *d / key_point_nums;
  return;
}

void SetLanePosition(
    const float& ref_min, const float& ref_length, const int& sample_num,
    const std::vector<LaneLinePtr>& lane_lines,
    const std::vector<bool>& far_lanes_flag,
    std::unordered_map<int, std::tuple<float, float>>* lane_d_map,
    bool use_far) {
  std::vector<std::pair<int, float>> left_lane_index;
  std::vector<std::pair<int, float>> right_lane_index;
  int size = lane_lines.size();

  for (int i = 0; i < size; ++i) {
    auto& curve = lane_lines[i]->vehicle_curve;
    float d = 0.f;
    GetRefValueForLineCurve(curve, &d, ref_min, ref_length, sample_num);
    auto iter = lane_d_map->find(lane_lines[i]->id);
    if (iter != lane_d_map->end()) {
      auto& d_data = iter->second;
      auto& last_d = std::get<0>(d_data);
      auto& d_error = std::get<1>(d_data);
      d_error = std::abs(d - last_d);
      last_d = d;
    } else {
      lane_d_map->emplace(
          std::make_pair(lane_lines[i]->id, std::make_tuple(d, 0.0)));
    }
    // 近处和远处车道线赋值逻辑
    if (use_far && !far_lanes_flag[i]) {
      lane_lines[i]->position = LaneLinePosition::OTHER;
      continue;
    } else if (!use_far && far_lanes_flag[i]) {
      lane_lines[i]->position = LaneLinePosition::OTHER;
      continue;
    } else {
    }

    if (d > 0) {
      left_lane_index.push_back(std::pair<int, float>(i, d));
    } else {
      right_lane_index.push_back(std::pair<int, float>(i, d));
    }
  }
  // left_lane: sort by decrease
  std::sort(left_lane_index.begin(), left_lane_index.end(),
            [](std::pair<int, float>& a, std::pair<int, float>& b) {
              return a.second < b.second;
            });
  // right_lane: sort by increase
  std::sort(right_lane_index.begin(), right_lane_index.end(),
            [](std::pair<int, float>& a, std::pair<int, float>& b) {
              return a.second > b.second;
            });

  // set left lane pos_type
  for (int i = 0; i < left_lane_index.size(); ++i) {
    int lane_index = left_lane_index[i].first;
    int temp = 0 - i - 1;
    if (kIndex2LanePosMap.count(temp) > 0) {
      lane_lines[lane_index]->position = kIndex2LanePosMap.at(temp);
    } else {
      lane_lines[lane_index]->position = LaneLinePosition::FOURTH_LEFT;
    }
  }

  // set right lane pos_type
  for (int i = 0; i < right_lane_index.size(); ++i) {
    int lane_index = right_lane_index[i].first;
    int temp = i + 1;
    if (kIndex2LanePosMap.count(temp) > 0) {
      lane_lines[lane_index]->position = kIndex2LanePosMap.at(temp);
    } else {
      lane_lines[lane_index]->position = LaneLinePosition::FOURTH_RIGHT;
    }
  }
}

void SetLanePosition(const float& ref_min, const float& ref_length,
                     const int& sample_num,
                     const std::vector<LaneLinePtr>& lane_lines) {
  std::vector<std::pair<int, float>> left_lane_index;
  std::vector<std::pair<int, float>> right_lane_index;
  int size = lane_lines.size();
  for (int i = 0; i < size; ++i) {
    float d = 0.f;
    auto& curve = lane_lines[i]->vehicle_curve;
    GetRefValueForLineCurve(curve, &d, ref_min, ref_length, sample_num);

    if (d > 0) {
      left_lane_index.push_back(std::pair<int, float>(i, d));
    } else {
      right_lane_index.push_back(std::pair<int, float>(i, d));
    }
  }
  // left_lane: sort by decrease
  std::sort(left_lane_index.begin(), left_lane_index.end(),
            [](std::pair<int, float>& a, std::pair<int, float>& b) {
              return a.second < b.second;
            });
  // right_lane: sort by increase
  std::sort(right_lane_index.begin(), right_lane_index.end(),
            [](std::pair<int, float>& a, std::pair<int, float>& b) {
              return a.second > b.second;
            });

  // set left lane pos_type
  for (int i = 0; i < left_lane_index.size(); ++i) {
    int lane_index = left_lane_index[i].first;
    int temp = 0 - i - 1;
    if (kIndex2LanePosMap.count(temp) > 0) {
      lane_lines[lane_index]->position = kIndex2LanePosMap.at(temp);
    } else {
      lane_lines[lane_index]->position = LaneLinePosition::FOURTH_LEFT;
    }
  }

  // set right lane pos_type
  for (int i = 0; i < right_lane_index.size(); ++i) {
    int lane_index = right_lane_index[i].first;
    int temp = i + 1;
    if (kIndex2LanePosMap.count(temp) > 0) {
      lane_lines[lane_index]->position = kIndex2LanePosMap.at(temp);
    } else {
      lane_lines[lane_index]->position = LaneLinePosition::FOURTH_RIGHT;
    }
  }
}

// void GetCurvartureRadius(const std::vector<float> &coeffs, float input,
//                          float *result) {
//   // r = (1+(f'x^2))^1.5 / |f''x|
//   // k = 1 /r
//   if (coeffs.size() < 3 || Equal(coeffs[2], static_cast<float>(0.0))) {
//     *result = FLT_MAX;
//     return;
//   }
//   // first order differential
//   float diff_1 = 0.0f;
//   float value = 1.0f;
//   for (int i = 1; i < coeffs.size(); ++i) {
//     diff_1 += i * coeffs[i] * value;
//     value *= input;
//   }
//   // second order differential
//   float diff_2 = 0.0f;
//   value = 1.0f;
//   for (int i = 2; i < coeffs.size(); ++i) {
//     diff_2 += i * (i - 1) * coeffs[i] * value;
//     value *= input;
//   }

//   *result = sqrt(pow(1 + pow(diff_1, 2), 3)) / (fabs(diff_2) + 1e-9);
// }

void GetTrangentialAngle(const std::vector<float>& coeffs, float input,
                         float* result) {
  // first order differential
  float diff_1 = 0.0f;
  float value = 1.0f;
  for (int i = 1; i < coeffs.size(); ++i) {
    diff_1 += i * coeffs[i] * value;
    value *= input;
  }
  *result = atan(diff_1) * 180 / M_PI;
}

bool TransformLaneLineCurveInNovatelPolyfit(
    const LaneLineCurve& curve, const Eigen::Affine3d& transform_matrix,
    LaneLineCurve* transform_line_curve) {
  if (!transform_line_curve) {
    return false;
  }

  // HLOG_DEBUG << "transform curve:" << transform_matrix.matrix();
  if (transform_matrix.matrix() == Eigen::Affine3d::Identity().matrix()) {
    *transform_line_curve = curve;
    return true;
  }

  if ((curve.max - curve.min < 1e-9) || (curve.coeffs.size() != 4)) {
    return false;
  }

  float interval = 4;
  int points_num = (curve.max - curve.min) / interval;
  if (points_num < 4) {
    points_num = 4;
    interval = (curve.max - curve.min) / points_num;
  }

  std::vector<Eigen::Matrix<float, 2, 1>> camera_pos_vec;
  // HLOG_DEBUG << "src curve.coeffs: " << curve.coeffs[0] << " ,"
  //           << curve.coeffs[1] << " ," << curve.coeffs[2] << " ,"
  //           << curve.coeffs[3] << " max: " << curve.max;
  for (size_t i = 0; i <= points_num; ++i) {
    float point = curve.min + i * interval;
    float curve_p = 0.f;

    LaneLinePolynomialEval(curve.coeffs, point, &curve_p);

    Eigen::Vector3d point_old(point, curve_p, 0);
    Eigen::Vector3d point_new = transform_matrix * point_old;

    Eigen::Matrix<float, 2, 1> camera_pos;
    camera_pos << point_new.x(), point_new.y();
    camera_pos_vec.push_back(camera_pos);

    // if (i == 0) {
    //   transform_line_curve->min = point_new.x();
    // } else if (i == points_num) {
    //   transform_line_curve->max = point_new.x();
    // }
    if (i == 0) {
      transform_line_curve->min = point_new.x();
    }
  }
  // transform_line_curve->min = curve.min;
  transform_line_curve->max = curve.max;
  Eigen::Matrix<float, max_poly_order + 1, 1> camera_coeff;
  bool is_x_axis = true;
  PolyFit(camera_pos_vec, max_poly_order, &camera_coeff, is_x_axis);
  // printf("拟合方程 1 为:y = %lf + %lfx + %lfx^2 + %lfx^3 \n",
  //        camera_coeff(0, 0), camera_coeff(1, 0), camera_coeff(2, 0),
  //        camera_coeff(3, 0));
  // HLOG_DEBUG << "polyfit coeff: " << camera_coeff(0, 0) << " ,"
  //           << camera_coeff(1, 0) << " ," << camera_coeff(2, 0) << " ,"
  //           << camera_coeff(3, 0) << " ,max:" << transform_line_curve->max;
  transform_line_curve->coeffs.clear();
  transform_line_curve->coeffs.emplace_back(camera_coeff(0, 0));
  transform_line_curve->coeffs.emplace_back(camera_coeff(1, 0));
  transform_line_curve->coeffs.emplace_back(camera_coeff(2, 0));
  transform_line_curve->coeffs.emplace_back(camera_coeff(3, 0));
  return true;
}

bool TransformLaneLineCurveInNovatel(const LaneLineCurve& curve,
                                     const Eigen::Affine3d& transform_matrix,
                                     LaneLineCurve* transform_line_curve) {
  if (!transform_line_curve) {
    return false;
  }

  // HLOG_DEBUG << "transform curve:" <<  transform_matrix.matrix();
  if (transform_matrix.matrix() == Eigen::Affine3d::Identity().matrix()) {
    *transform_line_curve = curve;
    return true;
  }
  if ((curve.max - curve.min < 1e-9) || (curve.coeffs.size() != 4)) {
    HLOG_DEBUG << "Invalid curve:" << curve.max << " ," << curve.min << ","
               << curve.coeffs.size();
    return false;
  }

  float matrix[16];
  float vector[4];
  float interval = (curve.max - curve.min) / 3;

  for (size_t i = 0; i < 4; ++i) {
    float point = curve.min + i * interval;
    float curve_p = 0.f;

    LaneLinePolynomialEval(curve.coeffs, point, &curve_p);

    // Eigen::Vector3d point_old(curve_p, point, 0);
    Eigen::Vector3d point_old(point, curve_p, 0);
    Eigen::Vector3d point_new = transform_matrix * point_old;
    HLOG_DEBUG << "point x:" << point << ","
               << "curve y:" << curve_p << ","
               << "x new:" << point_new.x() << ", y new:" << point_new.y();
    float x = point_new.x();
    float x2 = x * x;
    float x3 = x2 * x;

    matrix[4 * i] = x3;
    matrix[4 * i + 1] = x2;
    matrix[4 * i + 2] = x;
    matrix[4 * i + 3] = 1;
    vector[i] = point_new.y();

    if (i == 0) {
      transform_line_curve->min = x;
    } else if (i == 3) {
      transform_line_curve->max = x;
    }
  }

  Eigen::Matrix4f matrix_solve;
  matrix_solve << matrix[0], matrix[1], matrix[2], matrix[3], matrix[4],
      matrix[5], matrix[6], matrix[7], matrix[8], matrix[9], matrix[10],
      matrix[11], matrix[12], matrix[13], matrix[14], matrix[15];
  Eigen::Vector4f vector_solve;
  vector_solve << vector[0], vector[1], vector[2], vector[3];

  Eigen::Vector4f parameter = matrix_solve.inverse() * vector_solve;
  transform_line_curve->coeffs.clear();
  transform_line_curve->coeffs.emplace_back(parameter(3));
  transform_line_curve->coeffs.emplace_back(parameter(2));
  transform_line_curve->coeffs.emplace_back(parameter(1));
  transform_line_curve->coeffs.emplace_back(parameter(0));
  return true;
}

bool GetSamplePoint(const LaneLineCurve& curve1, const LaneLineCurve& curve2,
                    std::vector<float>* sample_values, float sample_interval,
                    float care_start, float care_end, bool hard_interval) {
  if (sample_values == nullptr) {
    return false;
  }
  sample_values->clear();
  float start = std::max(care_start, std::max(curve1.min, curve2.min));
  float end = std::min(care_end, std::min(curve1.max, curve2.max));
  // when hard mode, cal specific range no matter origin lane range
  if (hard_interval) {
    start = care_start;
    end = care_end;
  }

  if (end - start < sample_interval) {
    return false;
  }

  float result1 = 0.0f;
  float result2 = 0.0f;
  float temp_value = 0.0f;
  for (float y = start; y < end; y += sample_interval) {
    LaneLinePolynomialEval(curve1.coeffs, y, &result1);
    LaneLinePolynomialEval(curve2.coeffs, y, &result2);
    temp_value = fabs(result1 - result2);
    sample_values->push_back(temp_value);
  }
  // HLOG_DEBUG << "start:" << start << ", end:" << end << ",sample_interval:"
  //   << sample_interval << ", hard_interval:" << hard_interval
  // << ", sample_size:" << sample_values->size();
  return true;
}

bool CalLaneLineDistance(const LaneLineCurve& curve1,
                         const LaneLineCurve& curve2, float* distance,
                         float* var, float sample_interval, float care_start,
                         float care_end, bool hard_interval) {
  if (distance == nullptr || var == nullptr) {
    return false;
  }
  *distance = 0.0f;
  *var = 0.0f;

  std::vector<float> discrete_distances;
  bool status =
      GetSamplePoint(curve1, curve2, &discrete_distances, sample_interval,
                     care_start, care_end, hard_interval);
  if (!status) {
    return false;
  }

  float sum = std::accumulate(std::begin(discrete_distances),
                              std::end(discrete_distances), 0.0f);
  *distance = sum / discrete_distances.size();

  // compute val
  for (int i = 0; i < discrete_distances.size(); ++i) {
    *var += (discrete_distances[i] - *distance) *
            (discrete_distances[i] - *distance);
  }
  *var /= discrete_distances.size();
  // HLOG_DEBUG << ",distance:" << *distance << ",var:" << *var;
  return true;
}

bool CalLaneLineDistance(const LaneLineCurve& curve1,
                         const LaneLineCurve& curve2, float* distance,
                         float sample_interval, float care_start,
                         float care_end, bool hard_interval) {
  if (distance == nullptr) {
    return false;
  }
  *distance = 0.0f;

  std::vector<float> discrete_distances;
  bool status =
      GetSamplePoint(curve1, curve2, &discrete_distances, sample_interval,
                     care_start, care_end, hard_interval);
  if (!status) {
    return false;
  }

  float sum = std::accumulate(std::begin(discrete_distances),
                              std::end(discrete_distances), 0.0f);
  *distance = sum / discrete_distances.size();

  return true;
}

bool GetMiddleLineCurve(const LaneLineCurve& curve1,
                        const LaneLineCurve& curve2,
                        LaneLineCurve* middle_curve) {
  if (middle_curve == nullptr) {
    return false;
  }
  middle_curve->coeffs.clear();
  middle_curve->min = std::max(curve1.min, curve2.min);
  middle_curve->max = std::min(curve1.max, curve2.max);

  if (middle_curve->max < middle_curve->min ||
      curve1.coeffs.size() != curve2.coeffs.size()) {
    return false;
  }

  int dim = std::min(curve1.coeffs.size(), curve2.coeffs.size());
  for (size_t i = 0; i < dim; ++i) {
    middle_curve->coeffs.push_back(0.5 * (curve1.coeffs[i] + curve2.coeffs[i]));
  }

  return true;
}

float GetLength(const LaneLineCurve& curve) { return curve.max - curve.min; }

float GetLength(const std::vector<LaneLinePoint>& point_sets) {
  // auto start_x = point_sets.begin()->vehicle_point.x;
  // auto end_x = point_sets.end()->vehicle_point.x;
  // auto length = end_x - start_x;
  // return length;
  float total_length = 0;
  // 遍历point_sets中的点
  for (size_t i = 1; i < point_sets.size(); ++i) {
    // 计算车体系两点之间的欧式距离
    double dx =
        point_sets[i].vehicle_point.x - point_sets[i - 1].vehicle_point.x;
    double dy =
        point_sets[i].vehicle_point.y - point_sets[i - 1].vehicle_point.y;
    double length = std::sqrt(dx * dx + dy * dy);

    // 累加得到整个车道线的长度
    total_length += length;
  }
  return total_length;
}

float GetDistPointLane(const Point3DF& point_a, const Point3DF& point_b,
                       const Point3DF& point_c) {
  Eigen::Vector2f A(point_a.x, point_a.y), B(point_b.x, point_b.y),
      C(point_c.x, point_c.y);
  // 以B为起点计算向量BA 在向量BC上的投影
  Eigen::Vector2f BC = C - B;
  Eigen::Vector2f BA = A - B;
  float dist_proj = BA.dot(BC) / BC.norm();
  // 计算点到直线的距离
  double point_dist = sqrt(pow(BA.norm(), 2) - pow(dist_proj, 2));

  return point_dist;
}

float GetDistBetweenTwoLane(const std::vector<LaneLinePoint>& point_set1,
                            const std::vector<LaneLinePoint>& point_set2) {
  // 前提两条线的车辆系下的点已经从近到远排好序
  float curve_length1 =
      point_set1.back().vehicle_point.x - point_set1.front().vehicle_point.x;
  float curve_length2 =
      point_set2.back().vehicle_point.x - point_set2.front().vehicle_point.x;
  auto lane_short = point_set1, lane_long = point_set2;
  if (curve_length1 > curve_length2) {
    lane_short = point_set2;
    lane_long = point_set1;
  }
  double dist_sum = 0.0;
  Point3DF A, B, C;
  for (int i = 0, j = 0; i < lane_short.size() && j < lane_long.size(); ++i) {
    A = lane_short[i].vehicle_point;
    if (A.x <= lane_long[0].vehicle_point.x) {
      B = lane_long[0].vehicle_point;
      C = lane_long[1].vehicle_point;
    } else if (A.x >= lane_long.back().vehicle_point.x) {
      B = lane_long[lane_long.size() - 2].vehicle_point;
      C = lane_long[lane_long.size() - 1].vehicle_point;
    } else if (A.x < lane_long[j].vehicle_point.x) {
      B = lane_long[j - 1].vehicle_point;
      C = lane_long[j].vehicle_point;
    } else {
      ++j;
      --i;
      continue;
    }
    double dist = GetDistPointLane(A, B, C);
    dist_sum += dist;
  }
  return dist_sum / (lane_short.size());
}

float GetLengthRatioBetweenTwoLane(const LaneLineMeasurementConstPtr& curve1,
                                   const LaneLineMeasurementConstPtr& curve2) {
  auto length1 = GetLength(curve1->point_set);
  auto length2 = GetLength(curve2->point_set);

  if (length1 > length2) {
    return length2 / (length1 + 0.001);
  }

  return length1 / (length2 + 0.001);
}

float GetOverLayRatioBetweenTwoLane(const LaneLineMeasurementConstPtr& curve1,
                                    const LaneLineMeasurementConstPtr& curve2) {
  auto length1 = GetLength(curve1->point_set);
  auto length2 = GetLength(curve2->point_set);

  auto overlay_start = std::max(curve1->point_set.front().vehicle_point.x,
                                curve2->point_set.front().vehicle_point.x);
  auto overlay_end = std::min(curve1->point_set.back().vehicle_point.x,
                              curve2->point_set.back().vehicle_point.x);
  auto overlay_length =
      overlay_end - overlay_start > 0 ? overlay_end - overlay_start : 0;

  return overlay_length / std::min(length1, length2);
}

}  // namespace environment
}  // namespace mp
}  // namespace hozon
