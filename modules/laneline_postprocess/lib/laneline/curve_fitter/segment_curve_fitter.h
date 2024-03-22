// Copyright 2021 Hozon Inc. All Rights Reserved.
// File Name: segment_curve_fitter.h
// Author: YangYuhao (yangyuhao@hozon.com)
// Descriptions: segment curve fitter function

#pragma once

#include <cmath>
#include <string>
#include <unordered_set>
#include <vector>

#include "modules/laneline_postprocess/lib/laneline/curve_fitter/base_curve_fitter.h"
#include "modules/laneline_postprocess/lib/laneline/utils/laneline_polynomial.h"

namespace hozon {
namespace mp {
namespace environment {
namespace perception_base = hozon::perception::base;
struct CurveDistParams {
  float fix_param_xmin{0.f};
  float linear_select_ratio{0.f};
  float linear_xmin{0.f};
  float linear_xmax{10.f};
  float linear_len{20.f};
  float quadratic_len{40.f};
};
struct Candidate {
  Candidate() {
    pos_vec_indices.reserve(100);
    inliers_pos_vec_indices.reserve(100);
  }
  void Reset() {
    polynomial_candidate = nullptr;
    pos_vec_indices.clear();
    inliers_pos_vec_indices.clear();
    inliers_ratio = 0.f;
    sum_dist = 0.f;
  }
  LaneLinePolynomialPtr polynomial_candidate = nullptr;
  std::vector<int> pos_vec_indices;
  std::vector<int> inliers_pos_vec_indices;
  float inliers_ratio{0.f};
  float sum_dist{0.f};
};
enum class PolyOrder { LINEAR = 1, QUADRATIC = 2, CUBIC = 3 };
class SegmentCurveFitter : public BaseCurveFitter {
 public:
  SegmentCurveFitter() : BaseCurveFitter() {}

  virtual ~SegmentCurveFitter() {}

  bool Init(const BaseCurveFitterInitOptions& options =
                BaseCurveFitterInitOptions()) override;

  // @brief: Point lane curve fitter function
  bool CurveFitting(const std::vector<perception_base::Point2DF>& point_set,
                    LaneLinePolynomialPtr polynomial) override;
  bool CurveFitting(const std::vector<perception_base::Point3DF>& point_set,
                    LaneLinePolynomialPtr polynomial) override;
  bool CurveFitting(const std::vector<perception_base::Point2DF>& point_set,
                    LaneLinePolynomialPtr polynomial,
                    const perception_base::LaneLineCurve& target_curve);
  bool CurveFitting(const std::vector<perception_base::Point3DF>& point_set,
                    LaneLinePolynomialPtr polynomial,
                    const perception_base::LaneLineCurve& target_curve);
  std::string Name() const override;

 private:
  bool PolyFitProcess(Candidate* candidate);
  bool FixParamCurveFitting(Candidate* king_candidate,
                            const perception_base::LaneLineCurve& target_curve);
  bool FixParamCurveFitting(Candidate* king_candidate);
  bool MultiOrderCurveFitting(Candidate* king_candidate, bool fix_flag);
  bool MultiOrderCurveFitting(
      Candidate* king_candidate, bool fix_flag,
      const perception_base::LaneLineCurve& target_curve);
  bool CandidateStatics(Candidate* candidate);
  bool SelectPointBin(Candidate* candidate, int count);
  double CalculateRatioError(const std::vector<float>& new_curve,
                             const std::vector<float>& target_curve);

  int order_{3};
  float avg_dist_thresh_{0.1f};
  float max_dist_thresh_{1.f};
  float ransac_select_ratio_{0.2f};
  int ransac_select_pt_num_{10};
  int ransac_max_select_pt_num_{20};
  int ransac_epoch_num_{5};
  int bin_num_{5};
  int pt_in_bin_{0};

  CurveDistParams curve_dist_param;

  std::vector<perception_base::Point2DF> point_set_2d_;
  std::vector<Eigen::Matrix<float, 2, 1> > pos_vec_;
  std::vector<int> linear_pos_vec_index_;
  std::vector<int> pos_vec_random_index_;
  std::vector<int> bin_offset_vec_;

  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> A_;
  Eigen::Matrix<float, Eigen::Dynamic, 1> y_;
  Eigen::Matrix<float, Eigen::Dynamic, 1> result_;

  std::vector<float> fix_params_;
};

}  // namespace environment
}  // namespace mp
}  // namespace hozon
