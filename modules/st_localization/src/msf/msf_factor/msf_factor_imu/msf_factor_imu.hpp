/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * Wang Xiaofeng <wangxiaofeng@sensetime.com>
 */

#pragma once

#include <Eigen/Core>

#include "msf/msf_factor/msf_factor.hpp"

namespace senseAD {
namespace localization {
namespace msf {

class MSFFactorIMU : public MSFFactor {
 public:
  MSFFactorIMU() = default;

  virtual ~MSFFactorIMU() = default;

  virtual VectorXd EvaluateRes(const VectorXd& nominal_state) = 0;

  virtual void EvaluateHMat(const VectorXd& nominal_state) = 0;

  virtual void SetObservationCov(const MatrixXd& R) = 0;

  virtual void SetObservationState(const VectorXd& observation) = 0;

  virtual MatrixXd GetHMat() = 0;

  virtual MatrixXd GetVMat() = 0;

  static constexpr int kErrorStateSize = kiErrorStateSize;
};

}  // namespace msf
}  // namespace localization
}  // namespace senseAD
