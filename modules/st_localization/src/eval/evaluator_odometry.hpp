/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Du Jiankui <dujiankui@senseauto.com>
 */

#pragma once

#include <Eigen/Core>

#include "eval/evaluator_base.hpp"

namespace senseAD {
namespace localization {

struct OdometryEvalData {
  double gravity_align_confidence;
  double roll_obs;
  double pitch_obs;
  double roll_cov;
  double pitch_cov;
  double velo_lat_cov;
  double velo_vert_cov;
};

class EvaluatorOdometry : public EvaluatorBase<OdometryEvalData> {
 public:
  EvaluatorOdometry() = default;
  ~EvaluatorOdometry() {}

  adLocStatus_t WriteResult(double timestamp,
                            const OdometryEvalData& data) override;
};

}  // namespace localization
}  // namespace senseAD
