/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Fang Yu <fangyu1@senseauto.com>
 */

#pragma once

#include <Eigen/Core>

#include "eval/evaluator_base.hpp"

namespace senseAD {
namespace localization {

struct MotionEvalData {
  // motion output relative to current vehicle frame
  Eigen::Vector3d linear_speed;
  Eigen::Vector3d angular_speed;
  Eigen::Vector3d linear_acceleration;
};

class EvaluatorMotion : public EvaluatorBase<MotionEvalData> {
 public:
  EvaluatorMotion() = default;
  ~EvaluatorMotion() {}

  adLocStatus_t WriteResult(double timestamp,
                            const MotionEvalData& data) override;
};

}  // namespace localization
}  // namespace senseAD
