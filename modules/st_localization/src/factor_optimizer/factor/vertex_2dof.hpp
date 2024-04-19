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

class Vertex2DOF : public Vertex {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  DEFINE_SMART_PTR(Vertex2DOF);

  // note that 2 dimension, order: y, heading
  explicit Vertex2DOF(uint64_t id) : Vertex(id, 2) {}

  bool Plus(const double* x, const double* delta,
            double* x_plus_delta) const override {
    double y = x[0];
    double theta = x[1];

    double delta_y(delta[0]);
    double delta_theta = delta[1];

    // update state (y + SO2), note that is right multiply !
    double& plused_y = x_plus_delta[0];
    double& plused_theta = x_plus_delta[1];
    Sophus::SO2<double> tmp_theta =
        Sophus::SO2<double>::exp(theta) * Sophus::SO2<double>::exp(delta_theta);
    plused_y = y + delta_y;
    plused_theta = tmp_theta.log();

    return true;
  }
};

}  // namespace localization
}  // namespace senseAD
