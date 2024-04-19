/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>

#include "localization/data_type/base.hpp"
#include "localization/data_type/dual_antenna_heading.hpp"
#include "localization/data_type/smart_ptr.hpp"
#include "localization/localization_param.hpp"
#include "system/localization_backend.hpp"

namespace senseAD {
namespace localization {

class LocalizationBackendHeading
    : public LocalizationBackend,
      public BackendAdapter<void, DualAntennaHeading> {
 public:
  DEFINE_PTR(LocalizationBackendHeading)

  LocalizationBackendHeading() = default;
  LocalizationBackendHeading(const LocalizationBackendHeading&) = delete;
  LocalizationBackendHeading& operator=(const LocalizationBackendHeading&) =
      delete;
  virtual ~LocalizationBackendHeading() = default;

  NavStatus GetCurrentLocatorStatus() const final {
    return NavStatus::HIGH_ACCURACY;
  }

 protected:
  adLocStatus_t InitLocator() final;

  adLocStatus_t InitLocEvaluator() final;

  adLocStatus_t InitDataBuffer() final;

  void SetThreadName() final;

  adLocStatus_t ResetSubData() final;

  adLocStatus_t SwitchOriginSubProc() final;

  adLocStatus_t LocatorProcess() final;

 private:
  // buffer heading data for calculating heaidng covariance
  static constexpr size_t heading_arr_size_ = 3;
  float32_t heading_arr_[heading_arr_size_] = {0};
};

}  // namespace localization
}  // namespace senseAD
