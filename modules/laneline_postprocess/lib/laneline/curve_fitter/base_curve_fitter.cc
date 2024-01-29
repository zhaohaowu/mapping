// Copyright 2020 Hozon Inc. All Rights Reserved.
// File Name: base_curve_fitter.cc
// Author: YangYuhao (yangyuhao@hozon.com)
// Descriptions: base curve function

#include "modules/laneline_postprocess/lib/laneline/curve_fitter/base_curve_fitter.h"

namespace hozon {
namespace mp {
namespace environment {

void BaseCurveFitter::DoNormalization(
    std::vector<Eigen::Matrix<float, 2, 1>>* pos_vec) {
  int count = static_cast<int>(pos_vec->size());
  for (int i = 0; i < count; i++) {
    point_statistic_.x_sum += (*pos_vec)[i](0);
    point_statistic_.y_sum += (*pos_vec)[i](1);
  }
  point_statistic_.x_mean = static_cast<float>(point_statistic_.x_sum / count);
  point_statistic_.y_mean = static_cast<float>(point_statistic_.y_sum / count);
  // Calculate stdev
  double x_sum_square = 0.0;
  double y_sum_square = 0.0;
  for (const auto& pos : (*pos_vec)) {
    x_sum_square += pow(pos(0) - point_statistic_.x_mean, 2);
    y_sum_square += pow(pos(1) - point_statistic_.y_mean, 2);
  }
  point_statistic_.x_stdev = static_cast<float>(sqrt(x_sum_square / count));
  point_statistic_.y_stdev = static_cast<float>(sqrt(y_sum_square / count));

  // Normalize x to (x - x_mean) / 2*stdev
  float scale_factor_x = 1.f / (2.f * (point_statistic_.x_stdev));
  float scale_factor_y = 1.f / (2.f * (point_statistic_.y_stdev));
  for (auto& pos : (*pos_vec)) {
    pos(0) = (pos(0) - point_statistic_.x_mean) * scale_factor_x;
    pos(1) = (pos(1) - point_statistic_.y_mean) * scale_factor_y;
  }
}

bool BaseCurveFitter::CurveFitting(const std::vector<Point2DF>& point_set,
                                   LaneLinePolynomialPtr polynomial,
                                   const LaneLineCurve& target_curve) {
  return false;
}

bool BaseCurveFitter::CurveFitting(const std::vector<Point3DF>& point_set,
                                   LaneLinePolynomialPtr polynomial,
                                   const LaneLineCurve& target_curve) {
  return false;
}

void BaseCurveFitter::DoDenormalization(
    LaneLinePolynomialPtr polynomial_candidate) {
  auto& pos_coeff = polynomial_candidate->params;
  int count = static_cast<int>(pos_coeff.size());
  // max_order_ = 3, downward compatibility
  std::vector<float> inverse_coeff(max_order_ + 1);
  for (int i = 0; i < count; i++) {
    inverse_coeff[max_order_ - i] = pos_coeff[i];
  }
  if (count >= 1) {
    pos_coeff[0] =
        point_statistic_.y_stdev *
            (2 * point_statistic_.x_stdev * inverse_coeff[1] *
                 pow(point_statistic_.x_mean, 2) -
             inverse_coeff[0] * pow(point_statistic_.x_mean, 3) -
             4 * inverse_coeff[2] * pow(point_statistic_.x_stdev, 2) *
                 point_statistic_.x_mean) /
            (4 * pow(point_statistic_.x_stdev, 3)) +
        inverse_coeff[3] * point_statistic_.y_stdev * 2 +
        point_statistic_.y_mean;
  }
  if (count >= 2) {
    pos_coeff[1] = point_statistic_.y_stdev *
                   (3 * inverse_coeff[0] * pow(point_statistic_.x_mean, 2) -
                    4 * point_statistic_.x_stdev * inverse_coeff[1] *
                        point_statistic_.x_mean +
                    4 * inverse_coeff[2] * pow(point_statistic_.x_stdev, 2)) /
                   (4 * pow(point_statistic_.x_stdev, 3));
  }
  if (count >= 3) {
    pos_coeff[2] = point_statistic_.y_stdev *
                   (2 * point_statistic_.x_stdev * inverse_coeff[1] -
                    3 * inverse_coeff[0] * point_statistic_.x_mean) /
                   (4 * pow(point_statistic_.x_stdev, 3));
  }
  if (count >= 4) {
    pos_coeff[3] = point_statistic_.y_stdev * inverse_coeff[0] /
                   (4 * pow(point_statistic_.x_stdev, 3));
  }
  polynomial_candidate->min =
      polynomial_candidate->min * 2 * point_statistic_.x_stdev +
      point_statistic_.x_mean;
  polynomial_candidate->max =
      polynomial_candidate->max * 2 * point_statistic_.x_stdev +
      point_statistic_.x_mean;
}

}  // namespace environment
}  // namespace mp
}  // namespace hozon
