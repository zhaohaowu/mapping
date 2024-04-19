/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */

#pragma once

#include <Eigen/Core>
#include <memory>

#include <Sophus/se3.hpp>

#include "can/can_integrator.hpp"
#include "common/msf_common.hpp"
#include "localization/data_type/base.hpp"
#include "msf/msf_factor/msf_factor.hpp"
#include "msf/msf_fusion/msf_fusion.hpp"

namespace senseAD {
namespace localization {

class BaseLocator;

namespace msf {

class MSFCore;

class MSFFusionCAN : public MSFFusion {
 public:
  MSFFusionCAN() = delete;

  explicit MSFFusionCAN(std::shared_ptr<BaseLocator> locator,
                        const LocalizationParam& param);

  ~MSFFusionCAN();

  adLocStatus_t Init() override;

  adLocStatus_t AddData(uint64_t time_ns,
                        const VectorXd& data_reading) override;

  adLocStatus_t GetLatestMaxErrorState(Eigen::Vector4d* error_state) override;

  adLocStatus_t SearchNominalState(uint64_t time_ns,
                                   NavState* nav_state) override;

 private:
  adLocStatus_t SwitchOriginSubProc() override;

  // state update core
  adLocStatus_t StateUpdate(
      uint64_t timestamp,
      const std::shared_ptr<MSFFactor>& obs_factor) override;

  std::shared_ptr<MSFCore> main_filter_;
};

}  // namespace msf
}  // namespace localization
}  // namespace senseAD
