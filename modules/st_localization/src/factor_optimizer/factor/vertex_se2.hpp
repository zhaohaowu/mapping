/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */

#pragma once

#include <ceres/ceres.h>

#include <Eigen/Core>

#include <Sophus/so2.hpp>

#include "factor_optimizer/factor/vertex.hpp"
#include "localization/data_type/smart_ptr.hpp"

namespace senseAD {
namespace localization {

class VertexSE2 : public Vertex {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  DEFINE_SMART_PTR(VertexSE2);

  // note that 3 dimension, order: x, y, heading
  explicit VertexSE2(uint64_t id) : Vertex(id, 3) {}

  bool Plus(const double* x, const double* delta,
            double* x_plus_delta) const override {
    Eigen::Map<const Eigen::Vector2d> t(x);
    double theta = x[2];

    Eigen::Map<const Eigen::Vector2d> delta_t(delta);
    double delta_theta = delta[2];

    // update state (t + SO2), note that is right multiply !
    Eigen::Map<Eigen::Vector2d> plused_t(x_plus_delta);
    double& plused_theta = x_plus_delta[2];
    Sophus::SO2 tmp_theta =
        Sophus::SO2::exp(theta) * Sophus::SO2::exp(delta_theta);
    plused_t = t + delta_t;
    plused_theta = tmp_theta.log();

    return true;
  }
};

}  // namespace localization
}  // namespace senseAD
