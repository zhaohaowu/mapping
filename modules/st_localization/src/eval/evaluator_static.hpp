/*
 * Copyright (C) 2023 by SenseTime Group Limited. All rights reserved.
 * Lei Bing<leibing@senseauto.com>
 */

#pragma once

#include <Eigen/Core>

#include "eval/evaluator_base.hpp"

namespace senseAD {
namespace localization {

class EvaluatorStatic : public EvaluatorBase<double> {
 public:
  EvaluatorStatic() = default;
  ~EvaluatorStatic() {}

  adLocStatus_t WriteResult(double timestamp, const double& data) override;
};

}  // namespace localization
}  // namespace senseAD
