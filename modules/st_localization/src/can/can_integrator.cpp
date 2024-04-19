/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * Wang Xiaofeng<wangxiaofeng@sensetime.com>
 */

#include "can/can_integrator.hpp"

#include <Sophus/so3.hpp>

#include "common/transform_config.hpp"
#include "common/utility.hpp"
#include "localization/common/log.hpp"

namespace senseAD {
namespace localization {

adLocStatus_t CANIntegrator::Init(const LocalizationMSFParam& msf_param) {
  static auto SQR = [](double a) { return a * a; };

  // can linear velocity and angular speed measurement noise
  double sigma_vel = msf_param.can_sigma_vel;
  double sigma_omega = msf_param.can_sigma_omega;
  // can linear velocity scale and angular speed bias noise
  double sigma_vel_scale = msf_param.can_sigma_vel_scale;
  double sigma_omega_bias = msf_param.can_sigma_omega_bias;
  // initial error state estimation uncertainty
  double sigma_init_dPos = 0.01;
  double sigma_init_dAtt = 0.01;
  double sigma_init_dVS = 0.1;
  double sigma_init_dWB = 0.1;
  nominal_state_ = MakeNominalState(
      Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0),
      Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));

  P_ = MakeErrorStateCov(SQR(sigma_init_dPos), SQR(sigma_init_dAtt),
                         SQR(sigma_init_dVS), SQR(sigma_init_dWB));

  Q_ = MakeSystemNoiseCov(SQR(sigma_vel), SQR(sigma_omega),
                          SQR(sigma_vel_scale), SQR(sigma_omega_bias));

  return LOC_SUCCESS;
}

CANCoreState CANIntegrator::MakeNominalState(const Vector3d& p,
                                             const Quaterniond& q,
                                             const Vector3d& vs,
                                             const Vector3d& wb) {
  CANCoreState nominal_state;
  Vector4d q_hamilton = Utility::EigenQtoHamiltonVec(q);
  nominal_state << p, q_hamilton, vs, wb;
  return nominal_state;
}

CANPMat CANIntegrator::MakeErrorStateCov(double p_var, double a_var,
                                         double vs_var, double wb_var) {
  CANPMat P;
  P.setZero();
  P.block<3, 3>(kcErrorStatePosition, kcErrorStatePosition) = I33 * p_var;
  P.block<3, 3>(kcErrorStateQuat, kcErrorStateQuat) = I33 * a_var;
  P.block<3, 3>(kcErrorStateVs, kcErrorStateVs) = I33 * vs_var;
  P.block<3, 3>(kcErrorStateWb, kcErrorStateWb) = I33 * wb_var;
  return P;
}

CANQMat CANIntegrator::MakeSystemNoiseCov(double vel_mea_var,
                                          double omega_mea_var,
                                          double vel_scale_var,
                                          double omega_bias_var) {
  CANQMat Q;
  Q.setZero();
  Q.block<3, 3>(kcNoiseVelMea, kcNoiseVelMea) = I33 * vel_mea_var;
  Q.block<3, 3>(kcNoiseOmegaMea, kcNoiseOmegaMea) = I33 * omega_mea_var;
  Q.block<3, 3>(kcNoiseVelScale, kcNoiseVelScale) = I33 * vel_scale_var;
  Q.block<3, 3>(kcNoiseOmegaBias, kcNoiseOmegaBias) = I33 * omega_bias_var;
  return Q;
}

void CANIntegrator::EulerIntegrator(const CANMeasurement& can_reading,
                                    double dt) {
  Vector3d p = GetPosition();
  Quaterniond q = GetQuat();

  const Vector3d& v = can_reading.segment<3>(0);
  const Vector3d& w = can_reading.segment<3>(3);
  Matrix3d R = q.matrix();
  p = p + R * v * dt;
  Vector3d omega = w * dt;
  Quaterniond dq = SO3d::exp(omega).unit_quaternion();
  q = q * dq;
  q.normalize();

  // only update position and rotation
  SetPosition(p);
  SetQuat(q);

  // covariance propagation
  CovPropagation(can_reading, nominal_state_, dt);
}

void CANIntegrator::RK4Integrator(const CANMeasurement& can_reading_1,
                                  const CANMeasurement& can_reading_2,
                                  double dt) {
  Vector3d p = GetPosition();
  Quaterniond q = GetQuat();

  const Vector3d& v1 = can_reading_1.segment<3>(0);
  const Vector3d& w1 = can_reading_1.segment<3>(3);
  const Vector3d& v2 = can_reading_2.segment<3>(0);
  const Vector3d& w2 = can_reading_2.segment<3>(3);
  Vector3d v_dot = (v2 - v1) / dt;
  Vector3d w_dot = (w2 - w1) / dt;

  Matrix3d R0 = q.matrix();
  Vector3d omega = (w1 + w_dot * dt / 2) * dt / 2;
  Quaterniond dq = SO3d::exp(omega).unit_quaternion();
  Matrix3d R12 = (q * dq).matrix();
  omega = (w1 + w_dot * dt) * dt;
  dq = SO3d::exp(omega).unit_quaternion();
  Matrix3d R3 = (q * dq).matrix();

  Vector3d k1_v_dot = v_dot;
  Vector3d k1_p_dot = R0 * (v1 + k1_v_dot * 0.0);

  Vector3d k2_v_dot = v_dot;
  Vector3d k2_p_dot = R12 * (v1 + k2_v_dot * dt / 2);

  Vector3d k3_v_dot = v_dot;
  Vector3d k3_p_dot = R12 * (v1 + k3_v_dot * dt / 2);

  Vector3d k4_v_dot = v_dot;
  Vector3d k4_p_dot = R3 * (v1 + k4_v_dot * dt);

  q = R3;
  q.normalize();
  p = p + dt / 6 * (k1_p_dot + 2 * k2_p_dot + 2 * k3_p_dot + k4_p_dot);

  // only update position and rotation
  SetPosition(p);
  SetQuat(q);

  // covariance propagation
  CovPropagation(can_reading_2, nominal_state_, dt);
}

adLocStatus_t CANIntegrator::CovPropagation(const CANMeasurement& can_reading,
                                            const CANCoreState& core_state,
                                            double dt) {
  const Vector4d& q_vec = core_state.segment<4>(kcStateQuat);
  Matrix3d R = Utility::HamiltonVecToEigenQ(q_vec).matrix();
  // Note: can measurements had been compensated
  const Vector3d& v = can_reading.segment<3>(0);
  const Vector3d& w = can_reading.segment<3>(3);

  // vm: real velocity measure
  const Vector3d& vs = GetVelocityScale();
  const Eigen::Vector3d I_minus_vs = Eigen::Vector3d::Ones() - vs;
  const Vector3d vm = v.cwiseProduct(I_minus_vs.cwiseInverse().matrix());

  // Construct F Mat
  CANFMat F = CANFMat::Zero();
  F.block<3, 3>(kcErrorStatePosition, kcErrorStateQuat) = -R * SO3d::hat(v);
  F.block<3, 3>(kcErrorStatePosition, kcErrorStateVs) = -R * vm.asDiagonal();
  F.block<3, 3>(kcErrorStateQuat, kcErrorStateQuat) = -SO3d::hat(w);
  F.block<3, 3>(kcErrorStateQuat, kcErrorStateWb) = -I33;

  // Construce G Mat
  CANGMat G = CANGMat::Zero();
  G.block<3, 3>(kcErrorStatePosition, kcNoiseVelMea) = -R;
  G.block<3, 3>(kcErrorStateQuat, kcNoiseOmegaMea) = -I33;
  G.block<3, 3>(kcErrorStateVs, kcNoiseVelScale) = I33;
  G.block<3, 3>(kcErrorStateWb, kcNoiseOmegaBias) = I33;

  CANFMat Phi = CANFMat::Identity() + F * dt;
  CANFMat Qk = Phi * G * Q_ * G.transpose() * Phi.transpose() * dt;

  // update the a priori covariance matrix
  P_ = Phi * P_ * Phi.transpose() + Qk;
  P_ = (P_ + P_.transpose()) / 2;

  return LOC_SUCCESS;
}

adLocStatus_t CANIntegrator::SetVehicleCoreState(
    const CANCoreState& vehicle_core_state) {
  // note: MSF's nominal state is in vehicle frame relative to ref frame
  SE3d nominal_state_se3;
  nominal_state_se3.translation() =
      vehicle_core_state.segment<3>(kcStatePosition);
  Quaterniond q =
      Utility::HamiltonVecToEigenQ(vehicle_core_state.segment<4>(kcStateQuat));
  nominal_state_se3.so3() = SO3d(q);
  nominal_state_se3.normalize();
  SE3d core_state_se3;
  if (TransformConfig::FromVehicleToCAN(nominal_state_se3, &core_state_se3) !=
      LOC_SUCCESS) {
    LC_LERROR(CAN) << "Transform failed";
    return LOC_LOCALIZATION_ERROR;
  }

  SetPosition(core_state_se3.translation());
  SetQuat(core_state_se3.unit_quaternion());
  // copy linear speed scale and angular speed bias
  SetVelocityScale(vehicle_core_state.segment<3>(kcStateVs));
  SetAngularBias(vehicle_core_state.segment<3>(kcStateWb));

  return LOC_SUCCESS;
}

CANCoreState CANIntegrator::GetVehicleCoreState() const {
  SE3d core_state_se3;
  core_state_se3.translation() = GetPosition();
  Quaterniond q = GetQuat();
  core_state_se3.so3() = SO3d(q);
  core_state_se3.normalize();
  SE3d vehicle_state;
  if (TransformConfig::FromCANToVehicle(core_state_se3, &vehicle_state) !=
      LOC_SUCCESS) {
    LC_LERROR(CAN) << "Transform failed";
    return CANCoreState::Zero();
  }

  CANCoreState vehicle_core_state;
  vehicle_core_state.segment<3>(kcStatePosition) = vehicle_state.translation();
  vehicle_core_state.segment<4>(kcStateQuat) =
      Utility::EigenQtoHamiltonVec(vehicle_state.unit_quaternion());
  vehicle_core_state.segment<3>(kcStateVs) = GetVelocityScale();
  vehicle_core_state.segment<3>(kcStateWb) = GetAngularBias();

  return vehicle_core_state;
}

}  // namespace localization
}  // namespace senseAD
