/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * Wang Xiaofeng <wangxiaofeng@sensetime.com>
 */

#pragma once

#include <Eigen/Core>
#include <memory>

#include "common/msf_common.hpp"
#include "localization/data_type/base.hpp"

namespace senseAD {
namespace localization {
namespace msf {

using Eigen::MatrixXd;
using Eigen::VectorXd;

class MSFFactor {
 public:
  MSFFactor() = default;

  virtual ~MSFFactor() = default;

  virtual VectorXd EvaluateRes(const VectorXd& nominal_state) = 0;

  virtual void EvaluateHMat(const VectorXd& nominal_state) = 0;

  virtual void SetObservationCov(const MatrixXd& R) = 0;

  virtual void SetObservationState(const VectorXd& observation) = 0;

  MatrixXd GetObservationCov() const { return obs_cov_; }

  VectorXd GetObservationState() const { return obs_state_; }

  virtual MatrixXd GetHMat() = 0;

  virtual MatrixXd GetVMat() = 0;

  virtual void SetSource(STATE_SOURCE obs_source) { obs_source_ = obs_source; }

  virtual STATE_SOURCE GetSource() const { return obs_source_; }

 protected:
  MatrixXd obs_cov_;
  VectorXd obs_state_;

 private:
  STATE_SOURCE obs_source_ = NONE;
};

class MSFFactorFactory {
 public:
  static std::unique_ptr<MSFFactor> CreateMSFFactor(LocatorType front_type,
                                                    LocatorType back_type);
};

}  // namespace msf
}  // namespace localization
}  // namespace senseAD
