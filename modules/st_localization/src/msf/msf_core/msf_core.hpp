/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * Wang Xiaofeng <wangxiaofeng@sensetime.com>
 */

#pragma once

#include <Eigen/Core>
#include <memory>

#include <Sophus/se3.hpp>

#include "common/msf_common.hpp"
#include "localization/data_type/base.hpp"

namespace senseAD {
namespace localization {
namespace msf {

using Eigen::MatrixXd;
using Eigen::VectorXd;
class MSFFactor;

class MSFCore {
 public:
  MSFCore() = default;

  virtual ~MSFCore() = default;

  virtual adLocStatus_t SwitchOriginProc() = 0;

  virtual adLocStatus_t AddData(uint64_t time_ns, const VectorXd& data_reading,
                                bool do_state_predict) = 0;

  virtual adLocStatus_t StateUpdate(
      uint64_t timestamp, const std::shared_ptr<MSFFactor>& obs_factor) = 0;

  virtual adLocStatus_t GetLatestMaxErrorState(
      Eigen::Vector4d* error_state) = 0;

  virtual adLocStatus_t SearchNominalState(uint64_t time_ns,
                                           NavState* nav_state) = 0;
};

}  // namespace msf
}  // namespace localization
}  // namespace senseAD
