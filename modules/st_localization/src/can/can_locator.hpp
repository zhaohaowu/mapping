/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * Wang Xiaofeng <wangxiaofeng@sensetime.com>
 */

#pragma once

#include <list>
#include <memory>

#include <Sophus/se3.hpp>

#include "base_locator/base_locator.hpp"
#include "can/can_integrator.hpp"
#include "localization/data_type/base.hpp"
#include "localization/data_type/vehicle_info.hpp"
#include "localization/localization_param.hpp"

namespace senseAD {
namespace localization {

class CANLocator : public BaseLocator {
 public:
  CANLocator() = default;

  virtual ~CANLocator() = default;

  adLocStatus_t Init(const LocalizationParam& param) final;

  adLocStatus_t SetState(const NavState& nav_state) final;

  adLocStatus_t GetState(NavState* nav_state,
                         double* confidence = nullptr) final;

  adLocStatus_t Restart() final { return LOC_SUCCESS; }

  adLocStatus_t Process(uint64_t timestamp,
                        std::shared_ptr<VehicleInfo> can_data);

  adLocStatus_t SetInitialState(const SE3d& Tw_v0);

  std::shared_ptr<CANIntegrator> GetIntegrator() { return can_integrator_; }

 private:
  adLocStatus_t CanIntegrator();

  adLocStatus_t GetAccOutput(Vector3d* linear_acc);

  std::shared_ptr<CANIntegrator> can_integrator_ = nullptr;

  VehicleInfo can_data_;
  VehicleInfo last_can_data_;

  // state in vehicle frame relative to ref frame
  SE3d vehicle_state_;
  // state in integration frame (rear wheel) relative to ref frame
  CANCoreState core_state_;

  bool initial_state_set_ = false;

  float64_t last_time_ = -1.0;
  uint64_t timestamp_ = 0;
  float64_t dt_ = 0.01;

  Vector3d curr_linear_velocity_ = Vector3d::Zero();
  Vector3d curr_angular_speed_ = Vector3d::Zero();
  Vector3d curr_linear_acc_ = Vector3d::Zero();

  // for acc output
  Vector3d last_vehicle_speed_ = Vector3d::Zero();
  std::list<Vector3d> acc_output_list_;

  // offline calibration
  float64_t can_yaw_rate_scale_ = 1.0;
  float64_t can_yaw_rate_offset_ = 0.0;
  float64_t can_velocity_scale_ = 1.0;
  float64_t can_velocity_offset_ = 0.0;

  // the minimum speed of vehicle, when speed is lower than this param,
  // ignore the yaw_rate value(set to zero)
  float64_t can_minimum_speed_ = 0.001;
};

}  // namespace localization
}  // namespace senseAD
