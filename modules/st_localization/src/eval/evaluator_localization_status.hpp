/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming<zhaoming@senseauto.com>
 */

#pragma once

#include <Eigen/Core>

#include "eval/evaluator_base.hpp"

namespace senseAD {
namespace localization {

struct NavStatusEvalData {
  int8_t nav_status{-1};
  int8_t frontend_status{-1};
  int8_t backend_status{-1};
  int8_t msf_status{-1};
  int8_t output_status{-1};
  int8_t gnss_status{-1};
  int8_t smm_status{-1};
  // extent other backend locator status...
};

class EvaluatorLocalizationStatus : public EvaluatorBase<NavStatusEvalData> {
 public:
  EvaluatorLocalizationStatus() = default;
  ~EvaluatorLocalizationStatus() {}

  adLocStatus_t WriteResult(double timestamp,
                            const NavStatusEvalData& data) override;
};

}  // namespace localization
}  // namespace senseAD
