/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * lixin<lixin2@sensetime.com>
 * liwenqiang<liwenqiang1@sensetime.com>
 */

#pragma once

#include <Eigen/Core>

#include "eval/evaluator_base.hpp"

namespace senseAD {
namespace localization {

class EvaluatorLocalization : public EvaluatorBase<Eigen::Matrix4d> {
 public:
  EvaluatorLocalization() = default;
  ~EvaluatorLocalization() {}

  adLocStatus_t WriteResult(double timestamp,
                            const Eigen::Matrix4d& data) override;
};

}  // namespace localization
}  // namespace senseAD
