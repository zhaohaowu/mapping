/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <list>
#include <memory>
#include <utility>

#include "gnss/gnss_locator.hpp"
#include "localization/data_type/base.hpp"
#include "localization/data_type/gnss.hpp"
#include "localization/data_type/smart_ptr.hpp"
#include "localization/localization_param.hpp"
#include "system/localization_backend.hpp"

namespace senseAD {
namespace localization {

class LocalizationBackendGNSS : public LocalizationBackend,
                                public BackendAdapter<GNSSLocator, Gnss> {
 public:
  DEFINE_PTR(LocalizationBackendGNSS)

  LocalizationBackendGNSS() = default;
  LocalizationBackendGNSS(const LocalizationBackendGNSS&) = delete;
  LocalizationBackendGNSS& operator=(const LocalizationBackendGNSS&) = delete;
  virtual ~LocalizationBackendGNSS() = default;

  NavStatus GetCurrentLocatorStatus() const final;

  // @brief: check whether need system restart through calculate nominal state
  // continue consistency with RTK
  bool CheckNeedRestart() const;

  // @brief: check whether need system switch origin through calculate the
  // distance from the origin
  bool CheckNeedSwitchOrigin() const;

 protected:
  adLocStatus_t InitLocator() final;

  adLocStatus_t InitLocEvaluator() final;

  adLocStatus_t InitDataBuffer() final;

  void SetThreadName() final;

  adLocStatus_t ResetSubData() final;

  adLocStatus_t SwitchOriginSubProc() final;

  adLocStatus_t LocatorProcess() final;

  void UpdateNavStatus(uint64_t timestamp, double confidence);

 private:
  static constexpr size_t kConfDataWindowSize = 5;  // seconds
  static constexpr size_t kChi2DataWindowSize = 8;  // seconds

  // last valid gnss timestamp
  mutable std::mutex last_valid_gnss_mutex_;
  std::pair<uint64_t, std::shared_ptr<Gnss>> last_valid_gnss_;

  // buffer gnss locator confidence and chi-square datas
  mutable std::mutex conf_mutex_;
  boost::circular_buffer<std::pair<uint64_t, double>> conf_list_;
  mutable std::mutex chi_square_mutex_;
  boost::circular_buffer<double> chi_square_list_;

  // GNSS localization status
  std::atomic<NavStatus> gnss_status_{NavStatus::FATAL_ACCURACY};
};

}  // namespace localization
}  // namespace senseAD
