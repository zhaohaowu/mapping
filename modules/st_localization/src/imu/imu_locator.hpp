/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * Wang Xiaofeng <wangxiaofeng@sensetime.com>
 */

#pragma once

#include <Eigen/Core>
#include <list>
#include <memory>

#include <Sophus/se3.hpp>
#include <boost/circular_buffer.hpp>

#include "base_locator/base_locator.hpp"
#include "common/msf_common.hpp"
#include "imu/sins.hpp"
#include "localization/data_type/base.hpp"
#include "localization/data_type/imu.hpp"
#include "localization/localization_param.hpp"

namespace senseAD {
namespace localization {

class IMULocator : public BaseLocator {
 public:
  IMULocator() = default;
  virtual ~IMULocator() = default;

  adLocStatus_t Init(const LocalizationParam& param) final;

  adLocStatus_t SetState(const NavState& nav_state) final;

  adLocStatus_t GetState(NavState* nav_state,
                         double* confidence = nullptr) final;

  adLocStatus_t Restart() final { return LOC_SUCCESS; }

  adLocStatus_t Process(uint64_t timestamp, std::shared_ptr<Imu> raw_imu);

  adLocStatus_t SetInitialState(double lat, double lon, double alt,
                                const Eigen::Vector3d& ypr,
                                const Eigen::Vector3d& velocity,
                                double gnss_bias);

  std::shared_ptr<SINS> GetSINS() { return sins_; }

 private:
  Eigen::Vector3d SmoothAccOutput(const Eigen::Vector3d& an);

 private:
  uint64_t timestamp_ = 0;
  IMUMeasurement imu_reading_;

  // flag for whether set initial state
  bool initial_state_set_ = false;

  // acceleraion buffer for output smoothness
  boost::circular_buffer<Eigen::Vector3d> an_list_;

  // strapdown inertial navigation main implement
  std::shared_ptr<SINS> sins_;
};

}  // namespace localization
}  // namespace senseAD
