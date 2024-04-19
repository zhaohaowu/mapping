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

class VertexVelocity : public Vertex {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  DEFINE_SMART_PTR(VertexVelocity);

  // note that 3 dimensional velocity
  explicit VertexVelocity(uint64_t id) : Vertex(id, 3) {}

  bool Plus(const double* x, const double* delta,
            double* x_plus_delta) const override {
    Eigen::Map<const Eigen::Vector3d> v(x);
    Eigen::Map<const Eigen::Vector3d> delta_v(delta);
    Eigen::Map<Eigen::Vector3d> plused_v(x_plus_delta);
    plused_v = v + delta_v;
    return true;
  }
};

}  // namespace localization
}  // namespace senseAD
