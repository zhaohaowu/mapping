/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <atomic>

#include "ins/ins_locator.hpp"
#include "localization/data_type/base.hpp"
#include "localization/data_type/ins.hpp"
#include "localization/data_type/smart_ptr.hpp"
#include "localization/localization_param.hpp"
#include "system/localization_backend.hpp"

namespace senseAD {
namespace localization {

class LocalizationBackendINS : public LocalizationBackend,
                               public BackendAdapter<INSLocator, Ins> {
 public:
  DEFINE_PTR(LocalizationBackendINS)

  LocalizationBackendINS() = default;
  LocalizationBackendINS(const LocalizationBackendINS&) = delete;
  LocalizationBackendINS& operator=(const LocalizationBackendINS&) = delete;
  virtual ~LocalizationBackendINS() = default;

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
  std::atomic<uint64_t> last_fins_timestamp_{0};
};

}  // namespace localization
}  // namespace senseAD
