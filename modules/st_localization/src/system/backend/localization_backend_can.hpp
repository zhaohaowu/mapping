/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <atomic>

#include "localization/data_type/base.hpp"
#include "localization/data_type/smart_ptr.hpp"
#include "localization/data_type/vehicle_info.hpp"
#include "localization/localization_param.hpp"
#include "system/localization_backend.hpp"

namespace senseAD {
namespace localization {

class LocalizationBackendCAN : public LocalizationBackend,
                               public BackendAdapter<void, VehicleInfo> {
 public:
  DEFINE_PTR(LocalizationBackendCAN)

  LocalizationBackendCAN() = default;
  LocalizationBackendCAN(const LocalizationBackendCAN&) = delete;
  LocalizationBackendCAN& operator=(const LocalizationBackendCAN&) = delete;
  virtual ~LocalizationBackendCAN() = default;

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
  // record last fins timestamp for disorder check
  std::atomic<uint64_t> last_can_timestamp_{0};
};

}  // namespace localization
}  // namespace senseAD
