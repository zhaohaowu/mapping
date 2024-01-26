// Copyright 2020 Hozon Inc. All Rights Reserved.
// File Name: base_curve_function.h
// Author: YangYuhao (yangyuhao@hozon.com)
// Descriptions: base curve function

#pragma once

#include <string>
#include <vector>

#include "modules/laneline_postprocess/lib/laneline/interface/base_init_options.h"
#include "modules/laneline_postprocess/lib/laneline/utils/laneline_polynomial.h"
#include "perception-base/base/laneline/base_laneline.h"
#include "perception-lib/lib/registerer/registerer.h"

namespace hozon {
namespace mp {
namespace environment {

using namespace hozon::perception;

struct BaseCurveFitterInitOptions : public BaseInitOptions {
  float max_dist_thresh{0.1f};
  float avg_dist_thresh{0.07f};
  float ransac_select_ratio{0.2f};
  int ransac_max_select_pt_num{20};
  int ransac_epoch_num{5};
  bool do_normalization{true};
  bool do_align_curve_fit{false};
  bool do_extend_point{true};

  float fix_param_xmin{0.f};
  float linear_select_ratio{0.8f};
  float linear_xmin{0.f};
  float linear_xmax{10.f};
  float linear_len{15.f};
  float quadratic_len{40.f};
};

struct PointStatistic {
  double x_sum{0.f};
  float x_mean{0.f};
  float x_stdev{1.0};
  float y_mean{0.f};
  float y_stdev{1.0};
  double y_sum{0.f};

  float x_min{0.f};
  float x_max{0.f};
  float x_len{0.f};
  void Reset() {
    x_sum = 0.0;
    y_sum = 0.0;
    x_mean = 0.f;
    x_stdev = 1.f;
    y_mean = 0.f;
    y_stdev = 1.f;
    x_min = 0.f;
    x_max = 0.f;
    x_len = 0.f;
  }
};

class BaseCurveFitter {
 public:
  BaseCurveFitter() = default;

  virtual ~BaseCurveFitter() = default;

  virtual bool Init(const BaseCurveFitterInitOptions& options =
                        BaseCurveFitterInitOptions()) = 0;

  // @brief: Point lane curve fitter function
  virtual bool CurveFitting(const std::vector<base::Point2DF>& point_set,
                            LaneLinePolynomialPtr polynomial) = 0;
  virtual bool CurveFitting(const std::vector<base::Point3DF>& point_set,
                            LaneLinePolynomialPtr polynomial) = 0;
  virtual bool CurveFitting(const std::vector<base::Point2DF>& point_set,
                            LaneLinePolynomialPtr polynomial,
                            const base::LaneLineCurve& target_curve);
  virtual bool CurveFitting(const std::vector<base::Point3DF>& point_set,
                            LaneLinePolynomialPtr polynomial,
                            const base::LaneLineCurve& target_curve);
  virtual std::string Name() const = 0;

 protected:
  // @brief: Normalization function
  void DoNormalization(std::vector<Eigen::Matrix<float, 2, 1>>* pos_vec);
  void DoDenormalization(LaneLinePolynomialPtr polynomial_candidate);
  const float epsilon_ = 0.000000001f;
  // Only support max_order=3
  const int max_order_ = 3;
  int order_ = 3;
  int max_point_num_ = 500;
  bool do_normalization_{false};
  PointStatistic point_statistic_;

  BaseCurveFitter(const BaseCurveFitter&) = delete;
  BaseCurveFitter& operator=(const BaseCurveFitter&) = delete;
};

PERCEPTION_REGISTER_REGISTERER(BaseCurveFitter);
#define REGISTER_CURVE_FITTER(name) \
  PERCEPTION_REGISTER_CLASS(BaseCurveFitter, name)

}  // namespace environment
}  // namespace mp
}  // namespace hozon
