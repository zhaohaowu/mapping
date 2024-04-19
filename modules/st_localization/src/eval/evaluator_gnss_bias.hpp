/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming<zhaoming@senseauto.com>
 */

#pragma once

#include <Eigen/Core>

#include "eval/evaluator_base.hpp"

namespace senseAD {
namespace localization {

class EvaluatorGnssBias : public EvaluatorBase<Eigen::Vector2d> {
 public:
  EvaluatorGnssBias() = default;
  ~EvaluatorGnssBias() {}

  adLocStatus_t WriteResult(double timestamp,
                            const Eigen::Vector2d& data) override;
};

}  // namespace localization
}  // namespace senseAD
