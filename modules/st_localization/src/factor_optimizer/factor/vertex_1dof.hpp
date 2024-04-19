/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhu Kaiying <zhukaiying@sensetime.com>
 */

#pragma once

#include <ceres/ceres.h>

#include <Eigen/Core>

#include "factor_optimizer/factor/vertex.hpp"
#include "localization/data_type/smart_ptr.hpp"

namespace senseAD {
namespace localization {

class Vertex1DOF : public Vertex {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  DEFINE_SMART_PTR(Vertex1DOF);

  explicit Vertex1DOF(uint64_t id) : Vertex(id, 1) {}

  bool Plus(const double* x, const double* delta,
            double* x_plus_delta) const override {
    *x_plus_delta = *x + *delta;
    return true;
  }
};

}  // namespace localization
}  // namespace senseAD
