/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Fang Yu <fangyu1@senseauto.com>
 */

#pragma once

#include <ceres/ceres.h>

#include <Eigen/Core>

#include "factor_optimizer/factor/vertex.hpp"
#include "localization/data_type/smart_ptr.hpp"

namespace senseAD {
namespace localization {

class VertexBias : public Vertex {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  DEFINE_SMART_PTR(VertexBias);

  // note that 6 dimension, head 3 is ba, tail 3 is bg
  explicit VertexBias(uint64_t id) : Vertex(id, 6) {}

  bool Plus(const double* x, const double* delta,
            double* x_plus_delta) const override {
    Eigen::Map<const Eigen::Matrix<double, 6, 1>> bias(x);
    Eigen::Map<const Eigen::Matrix<double, 6, 1>> delta_bias(delta);
    Eigen::Map<Eigen::Matrix<double, 6, 1>> plused_bias(x_plus_delta);
    plused_bias = bias + delta_bias;
    return true;
  }
};

}  // namespace localization
}  // namespace senseAD
