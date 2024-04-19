/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * Wang Xiaofeng <wangxiaofeng@sensetime.com>
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <list>
#include <memory>
#include <utility>

#include <Sophus/se3.hpp>

#include "base_locator/base_locator.hpp"
#include "can/can_integrator.hpp"
#include "common/msf_common.hpp"
#include "common/sorted_container.hpp"
#include "common/utility.hpp"
#include "localization/data_type/base.hpp"
#include "localization/localization_param.hpp"
#include "msf/msf_core/msf_core.hpp"
#include "msf/msf_factor/msf_factor.hpp"

namespace senseAD {
namespace localization {
namespace msf {

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::Vector4d;

// Multi-Sensor Fusion for CAN front end
class MSFCoreCAN : public MSFCore {
 public:
  MSFCoreCAN() = delete;

  explicit MSFCoreCAN(std::shared_ptr<BaseLocator> locator);

  ~MSFCoreCAN();

  // struct to store can data and time
  struct CANData {
    uint64_t t_ns;  // timestamp, unit: ns
    Vector3d v;     // linear velcity from vehicle
    Vector3d w;     // angular velocity from vehicle
  };

  // struct of specific time state
  struct StateMeta {
    uint64_t t_ns;            // timestamp of this state estimate
    CANData can_data;         // CAN data measurements
    CANCoreState core_state;  // core state estimation
    CANPMat P_cov;            // Error state covariance
  };

  // struct of observation factor
  struct ObservationFactor {
    uint64_t t_ns;                      // timestamp of this factor
    std::shared_ptr<MSFFactor> factor;  // observation factor
  };

  adLocStatus_t SwitchOriginProc() override;

  adLocStatus_t AddData(uint64_t time_ns, const VectorXd& can_reading,
                        bool do_state_predict) override;

  adLocStatus_t StateUpdate(
      uint64_t timestamp,
      const std::shared_ptr<MSFFactor>& obs_factor) override;

  adLocStatus_t GetLatestMaxErrorState(Eigen::Vector4d* error_state) override;

  adLocStatus_t SearchNominalState(uint64_t time_ns,
                                   NavState* nav_state) override;

 private:
  using STATE_ptr = std::shared_ptr<StateMeta>;
  using obs_ptr = std::shared_ptr<ObservationFactor>;
  // Type of the state buffer to contian all the states
  using StateBuffer_T = SortedContainer<StateMeta>;
  // Type of the observation buffer to contain all the observations
  using ObservationBuffer_T = SortedContainer<ObservationFactor>;

  adLocStatus_t GetState(uint64_t obs_time, STATE_ptr* state_ptr);

  adLocStatus_t RobustKalmanFilter(const std::shared_ptr<MSFFactor>& msf_factor,
                                   STATE_ptr* state_ptr);

  adLocStatus_t InjectionErrorState();

  Vector3d GetPosition() const {
    return nominal_state_.segment<3>(kcStatePosition);
  }

  adLocStatus_t SetPosition(const Vector3d& p) {
    nominal_state_.segment<3>(kcStatePosition) = p;
    return LOC_SUCCESS;
  }

  Vector4d GetQuatVec() const { return nominal_state_.segment<4>(kcStateQuat); }

  Quaterniond GetQuat() const {
    Vector4d q_vec = GetQuatVec();
    return Utility::HamiltonVecToEigenQ(q_vec);
  }

  adLocStatus_t SetQuat(const Vector4d& q_vec) {
    nominal_state_.segment<4>(kcStateQuat) = q_vec;
    return LOC_SUCCESS;
  }

  adLocStatus_t SetQuat(const Quaterniond& q) {
    Vector4d q_vec = Utility::EigenQtoHamiltonVec(q);
    SetQuat(q_vec);
    return LOC_SUCCESS;
  }

  Vector3d GetVelScale() const { return nominal_state_.segment<3>(kcStateVs); }

  adLocStatus_t SetVelScale(const Vector3d& vs) {
    nominal_state_.segment<3>(kcStateVs) = vs;
    return LOC_SUCCESS;
  }

  Vector3d GetOmegaBias() const { return nominal_state_.segment<3>(kcStateWb); }

  adLocStatus_t SetOmegaBias(const Vector3d& wb) {
    nominal_state_.segment<3>(kcStateWb) = wb;
    return LOC_SUCCESS;
  }

 private:
  std::shared_ptr<CANIntegrator> can_integ_impl_ = nullptr;
  // nominal state and error state vector
  CANCoreState nominal_state_;
  CANErrorState error_state_;

  // filter buffer containing all states information sorted by time
  StateBuffer_T state_buffer_;

  // all the history observations
  ObservationBuffer_T observation_buffer_;

  // chi-square test table
  double chi2_95s_[8] = {0.0, 3.84, 5.99, 7.81, 9.49, 11.07, 12.59, 14.07};
  double chi2_995s_[8] = {0.0, 7.88, 10.6, 12.84, 14.86, 16.75, 18.55, 20.28};
};

}  // namespace msf
}  // namespace localization
}  // namespace senseAD
