/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming<zhaoming@senseauto.com>
 */

#pragma once

#include <Eigen/Core>

#include "eval/evaluator_base.hpp"

namespace senseAD {
namespace localization {

class EvaluatorGnssStatus : public EvaluatorBase<uint16_t> {
 public:
  EvaluatorGnssStatus() = default;
  ~EvaluatorGnssStatus() {}

  adLocStatus_t WriteResult(double timestamp, const uint16_t& data) override;
};

}  // namespace localization
}  // namespace senseAD
