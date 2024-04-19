/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */

#pragma once

#include <ceres/ceres.h>

#include <Eigen/Core>

#include <Sophus/so3.hpp>

#include "factor_optimizer/factor/vertex.hpp"
#include "localization/data_type/smart_ptr.hpp"

namespace senseAD {
namespace localization {

class VertexSE3 : public Vertex {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  DEFINE_SMART_PTR(VertexSE3);

  // note that 6 dimension, head 3 is position, tail 3 is rotation
  explicit VertexSE3(uint64_t id) : Vertex(id, 6) {}

  bool Plus(const double* x, const double* delta,
            double* x_plus_delta) const override {
    Eigen::Map<const Eigen::Vector3d> t(x);
    Eigen::Map<const Eigen::Vector3d> rot(x + 3);

    Eigen::Map<const Eigen::Vector3d> delta_t(delta);
    Eigen::Map<const Eigen::Vector3d> delta_rot(delta + 3);

    // update state (t + SO3d), note that is right multiply !
    Eigen::Map<Eigen::Vector3d> plused_t(x_plus_delta);
    Eigen::Map<Eigen::Vector3d> plused_rot(x_plus_delta + 3);
    SO3d tmp_rot = SO3d::exp(rot) * SO3d::exp(delta_rot);
    plused_t = t + delta_t;
    plused_rot = tmp_rot.log();

    return true;
  }
};

}  // namespace localization
}  // namespace senseAD
