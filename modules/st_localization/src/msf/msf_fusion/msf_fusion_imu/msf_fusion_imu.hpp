/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */

#pragma once

#include <Eigen/Core>
#include <map>
#include <memory>
#include <vector>

#include <Sophus/se3.hpp>

#include "common/msf_common.hpp"
#include "eval/evaluator_localziation.hpp"
#include "imu/sins.hpp"
#include "localization/data_type/base.hpp"
#include "msf/msf_core/msf_core_imu/msf_core_imu.hpp"
#include "msf/msf_factor/msf_factor.hpp"
#include "msf/msf_fusion/msf_fusion.hpp"

namespace senseAD {
namespace localization {

class BaseLocator;
class IMULocator;

namespace msf {
class MSFFusionIMU : public MSFFusion {
 public:
  MSFFusionIMU() = delete;

  MSFFusionIMU(std::shared_ptr<BaseLocator> locator,
               const LocalizationParam& param);

  ~MSFFusionIMU();

  adLocStatus_t Init() override;

  adLocStatus_t AddData(uint64_t time_ns,
                        const VectorXd& data_reading) override;

  adLocStatus_t GetLatestMaxErrorState(Eigen::Vector4d* error_state) override;

  adLocStatus_t SearchNominalState(uint64_t time_ns,
                                   NavState* nav_state) override;

 private:
  using FilterStatus = MSFCoreIMU::FilterStatus;
  using StateMeta = MSFCoreIMU::StateMeta;

  enum class SummaryStrategy {
    specific,  // choose the specific one as the output
    minCov,    // choose the minimum covariance local filter as the output
    summary    // summary of all the filters as the output
  };

  adLocStatus_t SwitchOriginSubProc() override;

  // state update core
  adLocStatus_t StateUpdate(
      uint64_t timestamp,
      const std::shared_ptr<MSFFactor>& obs_factor) override;

  ////////////////////////// create filter related ///////////////////////////
  adLocStatus_t CreateLocalFilter(
      const std::map<STATE_SOURCE, uint64_t>& obs_srcs,
      bool is_main_filter = true, bool different_Q = false);

  adLocStatus_t CreateMainFilter();

  adLocStatus_t CreateLocalFilters();

  adLocStatus_t CreateLocalIMUGNSSBaseFilter();

  adLocStatus_t CreateLocalIMUGNSSDQFilter();

  adLocStatus_t CreateLocalIMUSMMFilter();

  adLocStatus_t CreateLocalIMUSMMDQFilter();

  //////////////////////////// helper function  //////////////////////////////

  Eigen::Matrix4d ConvertNavStatePose(const StateMeta& state);

  StateMeta InformationFusion(const std::vector<StateMeta>& states_newest);

 private:
  LocalizationParam param_;

  // backend locator type
  std::vector<bool> back_locator_type_;

  // imu locator
  std::shared_ptr<IMULocator> locator_;

  // consider the diferent gnss device has different frequency
  uint64_t gnss_frequency_ = 1;

  // flag for first add data
  bool is_first_add_data_{true};

  // main filter, with local sub-filters
  std::shared_ptr<MSFCoreIMU> main_filter_;
  std::vector<std::shared_ptr<MSFCoreIMU>> local_filters_;

  // sins output is the sins state shared with imu_locator
  std::shared_ptr<SINS> sins_output_;

  // main and local filter summary output strategy
  SummaryStrategy summary_strategy_;

  // for offline evaluation of each filter (deprecated)
  // std::shared_ptr<EvaluatorLocalization> main_evaluator_;
  // std::vector<std::shared_ptr<EvaluatorLocalization>> local_evaluators_;
};

}  // namespace msf
}  // namespace localization
}  // namespace senseAD
