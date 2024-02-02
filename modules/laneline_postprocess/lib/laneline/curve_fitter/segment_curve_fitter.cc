// Copyright 2021 Hozon Inc. All Rights Reserved.
// @author: Yang Yuhao (yangyuhao@hozon.com)
// @file: segment.cc
// @brief: segment curve fitter function

#include "modules/laneline_postprocess/lib/laneline/curve_fitter/segment_curve_fitter.h"

#include <algorithm>
#include <memory>
#include <random>

#include "Eigen/Core"
#include "Eigen/LU"
#include "Eigen/QR"

namespace hozon {
namespace mp {
namespace environment {

bool SegmentCurveFitter::Init(const BaseCurveFitterInitOptions& options) {
  // options init
  avg_dist_thresh_ = options.avg_dist_thresh;
  max_dist_thresh_ = options.max_dist_thresh;
  ransac_select_ratio_ = options.ransac_select_ratio;
  ransac_max_select_pt_num_ = options.ransac_max_select_pt_num;
  ransac_epoch_num_ = options.ransac_epoch_num;
  do_normalization_ = options.do_normalization;

  // fix params init
  curve_dist_param.fix_param_xmin = options.fix_param_xmin;
  curve_dist_param.linear_select_ratio = options.linear_select_ratio;
  curve_dist_param.linear_xmin = options.linear_xmin;
  curve_dist_param.linear_xmax = options.linear_xmax;
  curve_dist_param.linear_len = options.linear_len;
  curve_dist_param.quadratic_len = options.quadratic_len;

  // reserve vector memory
  point_set_2d_.reserve(max_point_num_);
  pos_vec_.reserve(max_point_num_);
  pos_vec_random_index_.reserve(max_point_num_);
  bin_offset_vec_.reserve(bin_num_);
  linear_pos_vec_index_.reserve(max_point_num_);
  // reserve Eigen matrix memory
  A_.resize(max_point_num_, max_order_ + 1);
  y_.resize(max_point_num_);
  result_.resize(max_order_ + 1);
  return true;
}

bool SegmentCurveFitter::CurveFitting(
    const std::vector<perception_base::Point2DF>& point_set,
    LaneLinePolynomialPtr polynomial) {
  order_ = polynomial->order;
  HLOG_DEBUG << "polynomial->order:" << polynomial->order;
  if (order_ > max_order_) {
    HLOG_ERROR << "order > max_order.";
    return false;
  }
  int count = static_cast<int>(point_set.size());
  if (point_set.empty() || count < order_ + 1) {
    HLOG_ERROR << "point_set num < order + 1.";
    return false;
  }
  pos_vec_.clear();
  pos_vec_.resize(count);
  point_statistic_.Reset();
  linear_pos_vec_index_.clear();
  Eigen::Matrix<float, 2, 1> pos;
  // 纵向距离
  point_statistic_.x_min = point_set[0].x;
  point_statistic_.x_max = point_set[count - 1].x;
  point_statistic_.x_len = point_statistic_.x_max - point_statistic_.x_min;

  for (int i = 0; i < count; i++) {
    pos << point_set[i].x, point_set[i].y;
    pos_vec_[i] = pos;
    if (pos_vec_[i][0] > curve_dist_param.linear_xmin &&
        pos_vec_[i][0] <= curve_dist_param.linear_xmax) {
      linear_pos_vec_index_.emplace_back(i);
    }
  }
  if (do_normalization_) {
    DoNormalization(&pos_vec_);
    HLOG_DEBUG << "CurveFitting Norm Params:"
               << "x_mean:" << point_statistic_.x_mean
               << ",x_stdev:" << point_statistic_.x_stdev
               << ",y_mean:" << point_statistic_.y_mean
               << ",y_stdev:" << point_statistic_.y_stdev;
  }
  Candidate king_candidate;
  if (point_statistic_.x_min < curve_dist_param.fix_param_xmin) {
    if (!FixParamCurveFitting(&king_candidate)) {
      HLOG_ERROR << "FixParamCurveFitting return false.";
      return false;
    }
  } else {
    if (!MultiOrderCurveFitting(&king_candidate, false)) {
      HLOG_ERROR << "MultiOrderCurveFitting return false.";
      return false;
    }
  }

  // check the validity of laneline
  int inliers_count =
      static_cast<int>(king_candidate.inliers_pos_vec_indices.size());
  if (inliers_count == 0) {
    HLOG_ERROR << "Can't PolyCurve from points:" << count;
    return false;
  }
  float avg_dist = king_candidate.sum_dist / inliers_count;
  if (avg_dist > avg_dist_thresh_ - epsilon_) {
    HLOG_ERROR << "avg_dist larger than avg_dist_thresh_:" << avg_dist;
    return false;
  }
  // find xmin and xmax
  for (const auto& inliers_pos_vec_index :
       king_candidate.inliers_pos_vec_indices) {
    king_candidate.polynomial_candidate->min =
        std::min(pos_vec_[inliers_pos_vec_index](0),
                 king_candidate.polynomial_candidate->min);
    king_candidate.polynomial_candidate->max =
        std::max(pos_vec_[inliers_pos_vec_index](0),
                 king_candidate.polynomial_candidate->max);
  }
  // expand polynomial_candidate->params
  for (int i = 0; i < order_ - king_candidate.polynomial_candidate->order;
       ++i) {
    king_candidate.polynomial_candidate->params.emplace_back(0.f);
  }
  king_candidate.polynomial_candidate->order = order_;

  // Inverse transform coeff
  if (do_normalization_) {
    DoDenormalization(king_candidate.polynomial_candidate);
  }
  polynomial->min = king_candidate.polynomial_candidate->min;
  polynomial->max = king_candidate.polynomial_candidate->max;
  polynomial->params = king_candidate.polynomial_candidate->params;
  polynomial->inliers_indices = king_candidate.inliers_pos_vec_indices;

  return true;
}

double SegmentCurveFitter::CalculateRatioError(
    const std::vector<float>& new_curve,
    const std::vector<float>& target_curve) {
  double error = 0.0;
  HLOG_DEBUG << "new_curve.size() :" << new_curve.size()
             << ", target_curve.size()" << target_curve.size();
  if (new_curve.size() < 2) {
    error = 0.0;
    return error;
  }
  const float DimensionWeight[4] = {10, 100, 1000, 10000};
  double ori_value = 0.0;
  for (int i = 0; i < 2; ++i) {
    HLOG_DEBUG << " target_curve curve :" << target_curve[i] << " ,new_curve"
               << new_curve[i];
    error += DimensionWeight[i] * (std::abs(target_curve[i] - new_curve[i]));
    HLOG_DEBUG << " CalculateRatioError before error :"
               << std::abs(target_curve[i] - new_curve[i]);
    HLOG_DEBUG << " CalculateRatioError after error :" << error;

    ori_value += DimensionWeight[i] *
                 (std::abs(target_curve[i]) + std::abs(new_curve[i]));
  }
  error = error / ori_value;
  HLOG_DEBUG << " CalculateRatioError output error :" << error;

  return error;
}

bool SegmentCurveFitter::CurveFitting(
    const std::vector<perception_base::Point2DF>& point_set,
    LaneLinePolynomialPtr polynomial,
    const perception_base::LaneLineCurve& target_curve) {
  order_ = polynomial->order;
  HLOG_DEBUG << "polynomial->order:" << polynomial->order;
  if (order_ > max_order_) {
    HLOG_ERROR << "order > max_order.";
    return false;
  }
  int count = static_cast<int>(point_set.size());
  if (point_set.empty() || count < order_ + 1) {
    HLOG_ERROR << "point_set num < order + 1.";
    return false;
  }
  pos_vec_.clear();
  pos_vec_.resize(count);
  point_statistic_.Reset();
  linear_pos_vec_index_.clear();
  Eigen::Matrix<float, 2, 1> pos;
  // 纵向距离
  point_statistic_.x_min = point_set[0].x;
  point_statistic_.x_max = point_set[count - 1].x;
  point_statistic_.x_len = point_statistic_.x_max - point_statistic_.x_min;

  for (int i = 0; i < count; i++) {
    pos << point_set[i].x, point_set[i].y;
    pos_vec_[i] = pos;
    if (pos_vec_[i][0] > curve_dist_param.linear_xmin &&
        pos_vec_[i][0] <= curve_dist_param.linear_xmax) {
      linear_pos_vec_index_.emplace_back(i);
    }
  }

  if (do_normalization_) {
    DoNormalization(&pos_vec_);
    HLOG_DEBUG << "CurveFitting Norm Params:"
               << "x_mean:" << point_statistic_.x_mean
               << ",x_stdev:" << point_statistic_.x_stdev
               << ",y_mean:" << point_statistic_.y_mean
               << ",y_stdev:" << point_statistic_.y_stdev;
  }
  Candidate king_candidate;
  if (point_statistic_.x_min < curve_dist_param.fix_param_xmin) {
    if (!FixParamCurveFitting(&king_candidate)) {
      HLOG_ERROR << "FixParamCurveFitting return false.";
      return false;
    }
  } else {
    if (!MultiOrderCurveFitting(&king_candidate, false, target_curve)) {
      HLOG_ERROR << "MultiOrderCurveFitting return false.";
      return false;
    }
  }

  // if (FLAGS_enable_debug && king_candidate.polynomial_candidate) {
  //   std::stringstream coeffs_str;
  //   for (int i = king_candidate.polynomial_candidate->params.size() - 1; i >=
  //   0;
  //        i--) {
  //     coeffs_str << king_candidate.polynomial_candidate->params[i] << " ";
  //   }
  //   coeffs_str << "min: " << king_candidate.polynomial_candidate->min;
  //   coeffs_str << " max: " << king_candidate.polynomial_candidate->max;
  //   coeffs_str << " position: 0";
  //   DEBUG_COUT("king_candidate: polynomial_candidate: ", coeffs_str.str());
  // }

  // check the validity of laneline
  int inliers_count =
      static_cast<int>(king_candidate.inliers_pos_vec_indices.size());
  if (inliers_count == 0) {
    HLOG_ERROR << "Can't PolyCurve from points:" << count;
    return false;
  }
  float avg_dist = king_candidate.sum_dist / inliers_count;
  if (avg_dist > avg_dist_thresh_ - epsilon_) {
    HLOG_ERROR << "avg_dist larger than avg_dist_thresh_:" << avg_dist;
    return false;
  }
  // find xmin and xmax
  for (const auto& inliers_pos_vec_index :
       king_candidate.inliers_pos_vec_indices) {
    king_candidate.polynomial_candidate->min =
        std::min(pos_vec_[inliers_pos_vec_index](0),
                 king_candidate.polynomial_candidate->min);
    king_candidate.polynomial_candidate->max =
        std::max(pos_vec_[inliers_pos_vec_index](0),
                 king_candidate.polynomial_candidate->max);
  }
  // expand polynomial_candidate->params
  for (int i = 0; i < order_ - king_candidate.polynomial_candidate->order;
       ++i) {
    king_candidate.polynomial_candidate->params.emplace_back(0.f);
  }
  king_candidate.polynomial_candidate->order = order_;

  // Inverse transform coeff
  if (do_normalization_) {
    DoDenormalization(king_candidate.polynomial_candidate);
  }
  polynomial->min = king_candidate.polynomial_candidate->min;
  polynomial->max = king_candidate.polynomial_candidate->max;
  polynomial->params = king_candidate.polynomial_candidate->params;
  polynomial->inliers_indices = king_candidate.inliers_pos_vec_indices;

  return true;
}

bool SegmentCurveFitter::CurveFitting(
    const std::vector<perception_base::Point3DF>& point_set,
    LaneLinePolynomialPtr polynomial,
    const perception_base::LaneLineCurve& target_curve) {
  HLOG_DEBUG << "start do CurveFitting";
  int count = static_cast<int>(point_set.size());
  point_set_2d_.clear();
  point_set_2d_.resize(count);
  for (int i = 0; i < count; i++) {
    point_set_2d_[i].x = point_set[i].x;
    point_set_2d_[i].y = point_set[i].y;
  }
  if (!CurveFitting(point_set_2d_, polynomial, target_curve)) {
    return false;
  }

  HLOG_DEBUG << "finish do CurveFitting";
  return true;
}

bool SegmentCurveFitter::CurveFitting(
    const std::vector<perception_base::Point3DF>& point_set,
    LaneLinePolynomialPtr polynomial) {
  HLOG_DEBUG << "start do CurveFitting";
  int count = static_cast<int>(point_set.size());
  point_set_2d_.clear();
  point_set_2d_.resize(count);
  for (int i = 0; i < count; i++) {
    point_set_2d_[i].x = point_set[i].x;
    point_set_2d_[i].y = point_set[i].y;
  }
  if (!CurveFitting(point_set_2d_, polynomial)) {
    return false;
  }

  HLOG_DEBUG << "finish do CurveFitting";
  return true;
}

bool SegmentCurveFitter::FixParamCurveFitting(
    Candidate* king_candidate,
    const perception_base::LaneLineCurve& target_curve) {
  bool fix_flag = false;
  Candidate candidate;
  candidate.Reset();
  // Linear Fitting
  candidate.pos_vec_indices = linear_pos_vec_index_;
  candidate.polynomial_candidate = std::make_shared<LaneLinePolynomial>();
  candidate.polynomial_candidate->order = 1;
  if (!PolyFitProcess(&candidate)) {
    HLOG_ERROR << "PolyFitProcess return false.";
    return false;
  }
  // Check linear inliers ratio
  if (!CandidateStatics(&candidate)) {
    HLOG_ERROR << "CandidateStatics return false.";
    return false;
  }
  if (candidate.inliers_ratio > curve_dist_param.linear_select_ratio) {
    fix_flag = true;
    fix_params_ = candidate.polynomial_candidate->params;
    HLOG_DEBUG << "laneline fix params";
    for (const auto& param : fix_params_) {
      HLOG_DEBUG << "param:" << param;
    }
  }
  if (!MultiOrderCurveFitting(king_candidate, fix_flag)) {
    HLOG_ERROR << "FixParamCurveFitting::MultiOrderCurveFitting return false.";
    return false;
  }
  return true;
}

bool SegmentCurveFitter::FixParamCurveFitting(Candidate* king_candidate) {
  bool fix_flag = false;
  Candidate candidate;
  candidate.Reset();
  // Linear Fitting
  candidate.pos_vec_indices = linear_pos_vec_index_;
  candidate.polynomial_candidate = std::make_shared<LaneLinePolynomial>();
  candidate.polynomial_candidate->order = 1;
  if (!PolyFitProcess(&candidate)) {
    HLOG_ERROR << "PolyFitProcess return false.";
    return false;
  }
  // Check linear inliers ratio
  if (!CandidateStatics(&candidate)) {
    HLOG_ERROR << "CandidateStatics return false.";
    return false;
  }
  if (candidate.inliers_ratio > curve_dist_param.linear_select_ratio) {
    fix_flag = true;
    fix_params_ = candidate.polynomial_candidate->params;
    HLOG_DEBUG << "laneline fix params";
    for (const auto& param : fix_params_) {
      HLOG_DEBUG << "param:" << param;
    }
  }
  if (!MultiOrderCurveFitting(king_candidate, fix_flag)) {
    HLOG_ERROR << "FixParamCurveFitting::MultiOrderCurveFitting return false.";
    return false;
  }
  return true;
}

bool SegmentCurveFitter::MultiOrderCurveFitting(
    Candidate* king_candidate, bool fix_flag,
    const perception_base::LaneLineCurve& target_curve) {
  int order = order_;
  if (point_statistic_.x_len < curve_dist_param.linear_len) {
    order = 1;
  } else if (point_statistic_.x_len < curve_dist_param.quadratic_len) {
    order = 2;
  }
  HLOG_DEBUG << "laneline length:" << point_statistic_.x_len
             << "  order:" << order;
  int count = static_cast<int>(pos_vec_.size());
  pos_vec_random_index_.clear();
  pos_vec_random_index_.resize(count);
  for (int i = 0; i < count; ++i) {
    pos_vec_random_index_[i] = i;
  }
  pt_in_bin_ = std::ceil(static_cast<float>(count) / bin_num_);
  ransac_select_pt_num_ = std::min(
      ransac_max_select_pt_num_,
      std::max(order_ + 1, static_cast<int>(count * ransac_select_ratio_)));
  int max_iteration_num =
      static_cast<int>(ransac_epoch_num_ * count / ransac_select_pt_num_);

  Candidate candidate;
  HLOG_DEBUG << "LaneTrack: MultiOrderCurveFitting: start";
  for (int iter = 0; iter < max_iteration_num; iter++) {
    candidate.Reset();
    candidate.polynomial_candidate = std::make_shared<LaneLinePolynomial>();
    candidate.polynomial_candidate->order = order;
    if (fix_flag && iter < max_iteration_num - 1) {
      candidate.polynomial_candidate->params = fix_params_;
    }
    // candidate select point from bin
    if (!SelectPointBin(&candidate, count)) {
      HLOG_ERROR << "PolyFitProcess return false.";
      return false;
    }
    // poly fit process
    if (!PolyFitProcess(&candidate)) {
      HLOG_ERROR << "PolyFitProcess return false.";
      return false;
    }
    // calculate candidate statistic
    candidate.pos_vec_indices = pos_vec_random_index_;
    if (!CandidateStatics(&candidate)) {
      HLOG_ERROR << "CandidateStatics return false.";
      return false;
    }
    // if (FLAGS_enable_debug) {
    //   std::stringstream coeffs_str;
    //   for (int i = candidate.polynomial_candidate->params.size() - 1; i >= 0;
    //        i--) {
    //     coeffs_str << candidate.polynomial_candidate->params[i] << " ";
    //   }
    //   coeffs_str << "min: " << candidate.polynomial_candidate->min;
    //   coeffs_str << " max: " << candidate.polynomial_candidate->max;
    //   coeffs_str << " position: " << iter;
    //   DEBUG_COUT("iter_candidate: polynomial_candidate: ", coeffs_str.str());
    // }
    double errorRatio = CalculateRatioError(
        candidate.polynomial_candidate->params, target_curve.coeffs);

    candidate.inliers_ratio -= errorRatio - 1.0;
    HLOG_DEBUG << "iter :" << iter << " ,king_candidate->inliers_ratio :"
               << king_candidate->inliers_ratio
               << " candidate.inliers_ratio :" << candidate.inliers_ratio;
    if (candidate.inliers_ratio > king_candidate->inliers_ratio) {
      *king_candidate = candidate;
      HLOG_DEBUG << "king_candidate iter:" << iter
                 << " inliers_ratio:" << candidate.inliers_ratio;
    }
  }
  HLOG_DEBUG << "LaneTrack: MultiOrderCurveFitting: end";
  return true;
}

bool SegmentCurveFitter::MultiOrderCurveFitting(Candidate* king_candidate,
                                                bool fix_flag) {
  int order = order_;
  if (point_statistic_.x_len < curve_dist_param.linear_len) {
    order = 1;
  } else if (point_statistic_.x_len < curve_dist_param.quadratic_len) {
    order = 2;
  }
  HLOG_DEBUG << "laneline length:" << point_statistic_.x_len
             << "  order:" << order;
  int count = static_cast<int>(pos_vec_.size());
  pos_vec_random_index_.clear();
  pos_vec_random_index_.resize(count);
  for (int i = 0; i < count; ++i) {
    pos_vec_random_index_[i] = i;
  }
  pt_in_bin_ = std::ceil(static_cast<float>(count) / bin_num_);
  ransac_select_pt_num_ = std::min(
      ransac_max_select_pt_num_,
      std::max(order_ + 1, static_cast<int>(count * ransac_select_ratio_)));
  int max_iteration_num =
      static_cast<int>(ransac_epoch_num_ * count / ransac_select_pt_num_);

  Candidate candidate;
  for (int iter = 0; iter < max_iteration_num; iter++) {
    candidate.Reset();
    candidate.polynomial_candidate = std::make_shared<LaneLinePolynomial>();
    candidate.polynomial_candidate->order = order;
    if (fix_flag && iter < max_iteration_num - 1) {
      candidate.polynomial_candidate->params = fix_params_;
    }
    // candidate select point from bin
    if (!SelectPointBin(&candidate, count)) {
      HLOG_ERROR << "PolyFitProcess return false.";
      return false;
    }
    // poly fit process
    if (!PolyFitProcess(&candidate)) {
      HLOG_ERROR << "PolyFitProcess return false.";
      return false;
    }
    // calculate candidate statistic
    candidate.pos_vec_indices = pos_vec_random_index_;
    if (!CandidateStatics(&candidate)) {
      HLOG_ERROR << "CandidateStatics return false.";
      return false;
    }

    if (candidate.inliers_ratio > king_candidate->inliers_ratio) {
      *king_candidate = candidate;
      HLOG_DEBUG << "king_candidate iter:" << iter
                 << " inliers_ratio:" << candidate.inliers_ratio;
    }
  }
  return true;
}

bool SegmentCurveFitter::PolyFitProcess(Candidate* candidate) {
  int n = static_cast<int>(candidate->pos_vec_indices.size());

  int start_order =
      static_cast<int>(candidate->polynomial_candidate->params.size()) - 1;
  int order = candidate->polynomial_candidate->order;
  if (n < order + 1 || start_order > order) {
    HLOG_ERROR << "The number of points should be larger than the order.";
    return false;
  }
  // create data matrix
  int max_point_num = std::min(max_point_num_, n);
  auto A = A_.block(0, 0, max_point_num, order - start_order);
  auto y = y_.block(0, 0, max_point_num, 1);
  auto result = result_.block(0, 0, order - start_order, 1);
  A.setZero();
  y.setZero();
  result.setZero();

  for (int i = 0; i < max_point_num; ++i) {
    float x_pow =
        pow(pos_vec_[candidate->pos_vec_indices[i]](0), start_order + 1);
    for (int j = 0; j < order - start_order; ++j) {
      A(i, j) = x_pow;
      x_pow *= pos_vec_[candidate->pos_vec_indices[i]](0);
    }
    x_pow = 1.f;
    y(i, 0) = pos_vec_[candidate->pos_vec_indices[i]](1);
    for (int j = 0; j < start_order + 1; ++j) {
      y(i, 0) -= x_pow * candidate->polynomial_candidate->params[j];
      x_pow *= pos_vec_[candidate->pos_vec_indices[i]](0);
    }
  }
  result = A.householderQr().solve(y);
  for (int j = 0; j < order - start_order; ++j) {
    if (std::isnan(result(j, 0))) {
      HLOG_ERROR << "PolyFitProcess result nan.";
      return false;
    }
    candidate->polynomial_candidate->params.emplace_back(result(j, 0));
  }
  return true;
}

bool SegmentCurveFitter::CandidateStatics(Candidate* candidate) {
  int n = static_cast<int>(candidate->pos_vec_indices.size());
  for (int i = 0; i < n; i++) {
    float x = pos_vec_[candidate->pos_vec_indices[i]](0);
    float y = pos_vec_[candidate->pos_vec_indices[i]](1);

    HLOG_DEBUG << "CandidateStatics x:" << x << " ,y:" << y;

    float y_poly = candidate->polynomial_candidate->eval(x);
    float dist = std::fabs(y - y_poly);
    HLOG_DEBUG << "CandidateStatics y_poly:" << y_poly << " ,dist:" << dist;
    if (dist < max_dist_thresh_) {
      candidate->inliers_pos_vec_indices.emplace_back(
          candidate->pos_vec_indices[i]);
      candidate->sum_dist += dist;
    }
  }
  candidate->inliers_ratio =
      1.f * candidate->inliers_pos_vec_indices.size() / n;
  return true;
}

bool SegmentCurveFitter::SelectPointBin(Candidate* candidate, int count) {
  bin_offset_vec_.clear();
  bin_offset_vec_.resize(bin_num_);
  for (int i = 0; i < bin_num_; ++i) {
    int start = i * pt_in_bin_;
    int end = std::min((i + 1) * pt_in_bin_, count);
    if (start >= count) {
      break;
    }
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(pos_vec_random_index_.begin() + start,
                 pos_vec_random_index_.begin() + end, g);
  }
  int j = 0;
  for (int i = 0; i < ransac_select_pt_num_; ++i) {
    int start = j * pt_in_bin_;
    int idx = start + bin_offset_vec_[j];
    if (idx < count) {
      candidate->pos_vec_indices.emplace_back(pos_vec_random_index_[idx]);
      bin_offset_vec_[j] += 1;
    } else {
      --i;
    }
    j = j + 1;
    if (j >= bin_num_) {
      j %= bin_num_;
    }
  }
  return true;
}
std::string SegmentCurveFitter::Name() const { return "SegmentCurveFitter"; }
// Register plugin.
REGISTER_CURVE_FITTER(SegmentCurveFitter);
}  // namespace environment
}  // namespace mp
}  // namespace hozon
