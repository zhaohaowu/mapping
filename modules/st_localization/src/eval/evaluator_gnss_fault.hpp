/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Lei Bing<leibing@senseauto.com>
 */

#pragma once

#include <Eigen/Core>

#include "eval/evaluator_base.hpp"
#include "localization/data_type/gnss.hpp"

namespace senseAD {
namespace localization {

struct GnssFaultData {
  GnssFaultData() {}
  GnssFaultData(const int& ft, const float64_t& gd_rate,
                const float64_t& gd_consistency, const Gnss& gd)
      : fault_type(ft),
        gnss_dr_rate(gd_rate),
        gnss_dr_consistency(gd_consistency),
        gnss_data(gd) {}
  int fault_type;
  float64_t gnss_dr_rate;
  float64_t gnss_dr_consistency;
  Gnss gnss_data;
};

class EvaluatorGnssFault : public EvaluatorBase<GnssFaultData> {
 public:
  EvaluatorGnssFault() = default;
  ~EvaluatorGnssFault() {}

  adLocStatus_t WriteResult(double timestamp,
                            const GnssFaultData& data) override;
};

}  // namespace localization
}  // namespace senseAD
