/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming<zhaoming@senseauto.com>
 */

#pragma once

#include <Eigen/Core>

#include "eval/evaluator_base.hpp"

namespace senseAD {
namespace localization {

struct ImuIntrinsicEvalData {
  Eigen::Vector3d acc_bias{0, 0, 0};
  Eigen::Vector3d gyro_bias{0, 0, 0};
  Eigen::Vector3d acc_scale{0, 0, 0};
  Eigen::Vector3d gyro_scale{0, 0, 0};
};

class EvaluatorImuIntrinsic : public EvaluatorBase<ImuIntrinsicEvalData> {
 public:
  EvaluatorImuIntrinsic() = default;
  ~EvaluatorImuIntrinsic() {}

  adLocStatus_t WriteResult(double timestamp,
                            const ImuIntrinsicEvalData& data) override;
};

}  // namespace localization
}  // namespace senseAD
