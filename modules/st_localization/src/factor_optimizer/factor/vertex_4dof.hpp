/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Fang Yu <fangyu1@senseauto.com>
 */

#pragma once

#include <ceres/ceres.h>

#include <Eigen/Core>

#include <Sophus/so2.hpp>

#include "factor_optimizer/factor/vertex.hpp"
#include "localization/data_type/smart_ptr.hpp"

namespace senseAD {
namespace localization {

class Vertex4DOF : public Vertex {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  DEFINE_SMART_PTR(Vertex4DOF);

  // note that 4 dimension, head 3 is position, tail 1 is heading
  explicit Vertex4DOF(uint64_t id) : Vertex(id, 4) {}

  bool Plus(const double* x, const double* delta,
            double* x_plus_delta) const override {
    Eigen::Map<const Eigen::Vector3d> t(x);
    double theta = x[3];

    Eigen::Map<const Eigen::Vector3d> delta_t(delta);
    double delta_rot = delta[3];

    // update state (t + SO2), note that is right multiply !
    Eigen::Map<Eigen::Vector3d> plused_t(x_plus_delta);
    double& plused_rot = x_plus_delta[3];
    Sophus::SO2 tmp_theta =
        Sophus::SO2::exp(theta) * Sophus::SO2::exp(delta_rot);
    plused_t = t + delta_t;
    plused_rot = tmp_theta.log();

    return true;
  }
};

}  // namespace localization
}  // namespace senseAD
