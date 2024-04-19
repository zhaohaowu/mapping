/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * Wang Xiaofeng <wangxiaofeng@sensetime.com>
 */

#pragma once

#include "msf/msf_factor/msf_factor.hpp"

namespace senseAD {
namespace localization {
namespace msf {

class MSFFactorCAN : public MSFFactor {
 public:
  MSFFactorCAN() = default;

  virtual ~MSFFactorCAN() = default;

  virtual VectorXd EvaluateRes(const VectorXd& nominal_state) = 0;

  virtual void EvaluateHMat(const VectorXd& nominal_state) = 0;

  virtual void SetObservationCov(const MatrixXd& R) = 0;

  virtual void SetObservationState(const VectorXd& observation) = 0;

  virtual MatrixXd GetHMat() = 0;

  virtual MatrixXd GetVMat() = 0;

  // attitude is computed in aixs-angle, so state size - 1
  static constexpr int kTrueStateSize = kcStateSize - 1;
  static constexpr int kErrorStateSize = kcErrorStateSize;
};

}  // namespace msf
}  // namespace localization
}  // namespace senseAD
