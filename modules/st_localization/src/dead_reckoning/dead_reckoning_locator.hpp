/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Fang Yu <fangyu1@senseauto.com>
 */
#pragma once

#include <Eigen/Core>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

#include <Sophus/se3.hpp>
#include <boost/circular_buffer.hpp>

#include "localization/data_type/base.hpp"
#include "localization/data_type/imu.hpp"
#include "localization/data_type/vehicle_info.hpp"
#include "localization/localization_param.hpp"

namespace senseAD {
namespace localization {

class StaticDetector;
class EvaluatorOdometry;
class EvaluatorImuIntrinsic;

namespace dr {

class DRLocator {
 public:
  DRLocator() = default;
  virtual ~DRLocator() = default;

  // @brief: init locator
  virtual adLocStatus_t Init(const LocalizationParam& param) = 0;

  // @brief: get odom state
  virtual adLocStatus_t GetState(OdomState* odom_state,
                                 double* confidence = nullptr) = 0;

  // @brief: get current locator status
  virtual OdomStatus GetCurrentLocatorStatus() = 0;

  // @brief: set can data
  void SetCanData(uint64_t timestamp, const VehicleInfo& can_data) {
    std::lock_guard<std::mutex> lock(can_mutex_);
    can_data_list_.push_back(std::make_pair(timestamp, can_data));
  }

  // @brief: process imu data
  virtual adLocStatus_t Process(uint64_t timestamp,
                                std::shared_ptr<Imu> raw_imu) = 0;

 protected:
  // localization config parameters
  LocalizationParam param_;

  // buffer for can data
  static constexpr size_t kDataSizeUpperLimit = 10;
  mutable std::mutex can_mutex_;
  boost::circular_buffer<std::pair<uint64_t, VehicleInfo>> can_data_list_;

  // for static detect
  std::shared_ptr<StaticDetector> static_detector_;
  // for offline evaluation
  std::shared_ptr<EvaluatorImuIntrinsic> wio_imu_instrincs_;
};

}  // namespace dr
}  // namespace localization
}  // namespace senseAD
