/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * Wang Xiaofeng<wangxiaofeng@sensetime.com>
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "common/msf_common.hpp"
#include "common/utility.hpp"
#include "localization/data_type/base.hpp"
#include "localization/localization_param.hpp"

namespace senseAD {
namespace localization {

using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

using CANFMat = Eigen::Matrix<double, kcErrorStateSize, kcErrorStateSize>;
using CANGMat = Eigen::Matrix<double, kcErrorStateSize, kcNoiseSize>;

class CANIntegrator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CANIntegrator() = default;

  adLocStatus_t Init(const LocalizationMSFParam& msf_param);

  static CANCoreState MakeNominalState(const Vector3d& p, const Quaterniond& q,
                                       const Vector3d& vs, const Vector3d& wb);

  static CANPMat MakeErrorStateCov(double p_var, double a_var, double vs_var,
                                   double wb_var);

  static CANQMat MakeSystemNoiseCov(double vel_mea_var, double omega_mea_var,
                                    double vel_scale_var,
                                    double omega_bias_var);

  void EulerIntegrator(const CANMeasurement& can_reading, double dt);

  void RK4Integrator(const CANMeasurement& can_reading_1,
                     const CANMeasurement& can_reading_2, double dt);

  adLocStatus_t CovPropagation(const CANMeasurement& can_reading,
                               const CANCoreState& core_state, double dt);

  Vector3d GetPosition() const {
    return nominal_state_.segment<3>(kcStatePosition);
  }

  void SetPosition(const Vector3d& P) {
    nominal_state_.segment<3>(kcStatePosition) = P;
  }

  Vector4d GetQuatVec() const { return nominal_state_.segment<4>(kcStateQuat); }

  Quaterniond GetQuat() const {
    Vector4d q_vec = GetQuatVec();
    return Utility::HamiltonVecToEigenQ(q_vec);
  }

  void SetQuat(const Quaterniond& q) {
    Vector4d q_vec = Utility::EigenQtoHamiltonVec(q);
    nominal_state_.segment<4>(kcStateQuat) = q_vec;
  }

  Vector3d GetVelocityScale() const {
    return nominal_state_.segment<3>(kcStateVs);
  }

  void SetVelocityScale(const Vector3d& VS) {
    nominal_state_.segment<3>(kcStateVs) = VS;
  }

  Vector3d GetAngularBias() const {
    return nominal_state_.segment<3>(kcStateWb);
  }

  void SetAngularBias(const Vector3d& WB) {
    nominal_state_.segment<3>(kcStateWb) = WB;
  }

  adLocStatus_t SetP(const CANPMat& P) {
    P_ = P;
    return LOC_SUCCESS;
  }

  CANPMat GetP() const { return P_; }

  adLocStatus_t SetCoreState(const CANCoreState& nominal_state) {
    nominal_state_ = nominal_state;
    return LOC_SUCCESS;
  }

  CANCoreState GetCoreState() const { return nominal_state_; }

  // In msf, fusion frame is choosen as vehicle frame
  // here, transform from the vehicle frame core state into rear wheel
  // (integration) frame core state
  adLocStatus_t SetVehicleCoreState(const CANCoreState& vehicle_core_state);

  CANCoreState GetVehicleCoreState() const;

 private:
  // error state covariance matrix
  CANPMat P_;
  // nominal state and error state vector
  CANCoreState nominal_state_;
  // about system input noises and perturbations
  CANQMat Q_;
};

}  // namespace localization
}  // namespace senseAD
