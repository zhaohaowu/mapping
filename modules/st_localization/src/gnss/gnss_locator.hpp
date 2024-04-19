/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * Wang Xiaofeng <wangxiaofeng@sensetime.com>
 */

#pragma once

#include <Eigen/Core>
#include <memory>

#include "base_locator/base_locator.hpp"
#include "gnss_fault_diagnosis.hpp"
#include "localization/data_type/base.hpp"
#include "localization/data_type/gnss.hpp"
#include "localization/localization_param.hpp"

namespace senseAD {
namespace localization {

class EvaluatorGnssFault;

class GNSSLocator : public BaseLocator {
 public:
  GNSSLocator() = default;

  virtual ~GNSSLocator() = default;

  adLocStatus_t Init(const LocalizationParam& param) final;

  adLocStatus_t SetState(const NavState& nav_state) final;

  adLocStatus_t SetOdomState(const OdomState& odom_state);

  adLocStatus_t GetState(NavState* nav_state,
                         double* confidence = nullptr) final;

  adLocStatus_t Restart() final { return LOC_SUCCESS; }

  adLocStatus_t SwitchOriginProc();

  adLocStatus_t Process(const uint64_t timestamp,
                        std::shared_ptr<Gnss> raw_gnss);

  double GetLocatorChiSquare() const;

  static adLocStatus_t ProcessForGtMode(const uint64_t timestamp,
                                        std::shared_ptr<Gnss> raw_gnss,
                                        NavState* nav_state);

 private:
  adLocStatus_t SolveHeading(std::shared_ptr<Gnss> raw_gnss, double* heading,
                             double* heading_std);

  adLocStatus_t SolveHeadingByGnssPosition(const Gnss& last_gnss,
                                           const Gnss& cur_gnss,
                                           double* heading);

  adLocStatus_t SolveHeadingByGnssVelocity(std::shared_ptr<Gnss> raw_gnss,
                                           double* heading,
                                           double* heading_std);

 private:
  NavState init_pose_state_;   // init pose state
  OdomState init_odom_state_;  // init odom state
  NavState pose_state_;        // refined pose state
  double confidence_ = 0.0;    // location confidence for this frame
  double chi_square_ = 0.0;    // location chi square for this frame

  // gnss fault diagnostor instance
  std::unique_ptr<GNSSFaultDiagnostor> gnss_fault_diagnostor_ = nullptr;

  // gnss evaluator
  std::shared_ptr<EvaluatorGnssFault> gnss_fault_evaluator_ = nullptr;
};

}  // namespace localization
}  // namespace senseAD
