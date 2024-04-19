/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */

#pragma once

#include <Eigen/Core>
#include <atomic>
#include <list>
#include <memory>
#include <mutex>
#include <utility>

#include <Sophus/se3.hpp>

#include "common/msf_common.hpp"
#include "localization/data_type/base.hpp"

namespace senseAD {
namespace localization {
namespace msf {

class MSFFactor;
using Eigen::MatrixXd;
using Eigen::VectorXd;

class MSFFusion {
 public:
  MSFFusion() = default;

  virtual ~MSFFusion() = default;

  virtual adLocStatus_t Init() = 0;

  adLocStatus_t SwitchOriginProc();

  virtual adLocStatus_t AddData(uint64_t time_ns,
                                const VectorXd& data_reading) = 0;

  adLocStatus_t AddFactor(uint64_t timestamp,
                          const std::shared_ptr<MSFFactor>& obs_factor);

  adLocStatus_t StateUpdate();

  virtual adLocStatus_t GetLatestMaxErrorState(
      Eigen::Vector4d* error_state) = 0;

  virtual adLocStatus_t SearchNominalState(uint64_t time_ns,
                                           NavState* nav_state) = 0;

  bool IsStarted() { return is_started_; }

  void SetStart(bool start) { is_started_ = start; }

 protected:
  virtual adLocStatus_t SwitchOriginSubProc() = 0;

  virtual adLocStatus_t StateUpdate(
      uint64_t timestamp, const std::shared_ptr<MSFFactor>& obs_factor) = 0;

  int CompareTime(uint64_t base_time, uint64_t timestamp);

 protected:
  // msf fusion start flag
  std::atomic<bool> is_started_{false};

  // latest processed frontend timestamp
  std::atomic<uint64_t> latest_frontend_ts_{0};

  // window of backend observation factors for msf update, thread safety
  std::mutex factor_mutex_;
  std::list<std::pair<uint64_t, std::shared_ptr<MSFFactor>>> obs_factors_;

  // for thread safety when methods used in different threads
  std::mutex msf_core_mutex_;
};

}  // namespace msf
}  // namespace localization
}  // namespace senseAD
