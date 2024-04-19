/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * Wang Xiaofeng <wangxiaofeng@sensetime.com>
 */

#include "msf/msf_core/msf_core_can/msf_core_can.hpp"

#include <Sophus/so3.hpp>

#include "can/can_locator.hpp"
#include "common/utility.hpp"
#include "localization/common/log.hpp"

namespace senseAD {
namespace localization {
namespace msf {

MSFCoreCAN::MSFCoreCAN(std::shared_ptr<BaseLocator> locator) : MSFCore() {
  can_integ_impl_ =
      std::dynamic_pointer_cast<CANLocator>(locator)->GetIntegrator();
}

MSFCoreCAN::~MSFCoreCAN() {}

adLocStatus_t MSFCoreCAN::SwitchOriginProc() { return LOC_SUCCESS; }

adLocStatus_t MSFCoreCAN::AddData(uint64_t time_ns, const VectorXd& can_reading,
                                  bool do_state_predict) {
  CANData can_data;
  can_data.t_ns = time_ns;
  can_data.v = can_reading.segment<3>(0);
  can_data.w = can_reading.segment<3>(3);

  STATE_ptr new_state_ptr = std::make_shared<StateMeta>();
  new_state_ptr->t_ns = time_ns;
  new_state_ptr->can_data = can_data;
  new_state_ptr->core_state = can_integ_impl_->GetVehicleCoreState();
  new_state_ptr->P_cov = can_integ_impl_->GetP();
  state_buffer_.insert(new_state_ptr);
  state_buffer_.ClearOlderThan(1.0);
  nominal_state_ = can_integ_impl_->GetVehicleCoreState();

  return LOC_SUCCESS;
}

adLocStatus_t MSFCoreCAN::GetLatestMaxErrorState(Eigen::Vector4d* error_state) {
  // TODO(zhaoming): implement it!
  return LOC_LOCALIZATION_ERROR;
}

adLocStatus_t MSFCoreCAN::SearchNominalState(uint64_t time_ns,
                                             NavState* nav_state) {
  if (nav_state == nullptr) {
    LC_LERROR(MSF) << "nullptr";
    return LOC_LOCALIZATION_ERROR;
  }

  if (state_buffer_.empty()) {
    LC_LDEBUG(MSF) << "search failed, state buffer is empty";
    return LOC_LOCALIZATION_ERROR;
  }

  if (time_ns >= state_buffer_.GetLast()->t_ns) {
    LC_LDEBUG(MSF) << "search time stamp ahead";
    return LOC_LOCALIZATION_ERROR;
  }
  if (time_ns <= state_buffer_.GetFirst()->t_ns) {
    LC_LDEBUG(MSF) << "search time stamp behind";
    return LOC_LOCALIZATION_ERROR;
  }

  STATE_ptr search_state_ptr = nullptr;
  if (GetState(time_ns, &search_state_ptr) != LOC_SUCCESS) {
    LC_LERROR(MSF) << "search state failed";
    return LOC_LOCALIZATION_ERROR;
  }

  nav_state->timestamp = time_ns;

  // global vehicle pose
  Eigen::Vector3d t = search_state_ptr->core_state.segment<3>(kcStatePosition);
  Eigen::Vector4d q = search_state_ptr->core_state.segment<4>(kcStateQuat);
  nav_state->pose = SE3d(Utility::HamiltonVecToEigenQ(q), t);

  // TODO(zm): supplement other state quantities

  // global ENU covariance
  nav_state->pose_cov = search_state_ptr->P_cov.block<6, 6>(0, 0);

  return LOC_SUCCESS;
}

adLocStatus_t MSFCoreCAN::StateUpdate(
    uint64_t timestamp, const std::shared_ptr<MSFFactor>& obs_factor) {
  obs_ptr obs_factor_ptr = std::make_shared<ObservationFactor>();
  obs_factor_ptr->t_ns = timestamp;
  obs_factor_ptr->factor = obs_factor;

  // check if current observation is much too delayed
  if (obs_factor_ptr->t_ns < state_buffer_.GetFirst()->t_ns) {
    LC_LERROR(MSF) << "observation behinds too much.";
    return LOC_LOCALIZATION_ERROR;
  }

  // remove very old observations if necessary
  observation_buffer_.ClearOlderThan(0.5);
  ObservationBuffer_T::iterator_T iter_obs_curr =
      observation_buffer_.insert(obs_factor_ptr);
  ObservationBuffer_T::iterator_T iter_obs_end =
      observation_buffer_.IteratorEnd();

  // go through all the observations and apply them one by one
  for (; iter_obs_curr != iter_obs_end; ++iter_obs_curr) {
    uint64_t curr_obs_time = iter_obs_curr->second->t_ns;
    // get state that's corresponding to this observation
    STATE_ptr curr_state_ptr = nullptr;
    if (GetState(curr_obs_time, &curr_state_ptr) != LOC_SUCCESS) {
      LC_LERROR(MSF) << "get state failed, unused observations";
      continue;
    }

    // do observation update here with robust strategy
    RobustKalmanFilter(iter_obs_curr->second->factor, &curr_state_ptr);

    // returned state maybe a rough approximation with observation time
    // so using state time to search
    float64_t curr_state_time_f =
        static_cast<float64_t>(curr_state_ptr->t_ns) * kNanoSecToSec;
    // make sure to propagate to next observation or up to now if no more
    // observations, propagate from current state
    StateBuffer_T::iterator_T iter_state_curr =
        state_buffer_.GetIteratorAt(curr_state_time_f);
    StateBuffer_T::iterator_T iter_state_end;
    ObservationBuffer_T::iterator_T iter_obs_next = iter_obs_curr;
    iter_obs_next++;
    if (iter_state_curr == state_buffer_.IteratorEnd()) {
      LC_LERROR(MSF) << "why added new state can't find ?";
    }
    if (iter_obs_next == iter_obs_end) {
      iter_state_end = state_buffer_.IteratorEnd();
    } else {
      iter_state_end =
          state_buffer_.GetIteratorClosestAfter(iter_obs_next->first);
      if (iter_state_end != state_buffer_.IteratorEnd()) {
        ++iter_state_end;
      }
    }

    StateBuffer_T::iterator_T iter_state_next = iter_state_curr;
    ++iter_state_next;
    // propagate to selected state
    for (;
         iter_state_curr != iter_state_end && iter_state_next != iter_state_end;
         ++iter_state_curr, ++iter_state_next) {
      auto curr_state_ptr = iter_state_curr->second;
      auto next_state_ptr = iter_state_next->second;
      // state propagation to current state
      std::unique_ptr<CANIntegrator> can_integrator =
          std::make_unique<CANIntegrator>();
      can_integrator->SetVehicleCoreState(curr_state_ptr->core_state);
      can_integrator->SetP(curr_state_ptr->P_cov);
      CANMeasurement can_reading_1;
      can_reading_1 << curr_state_ptr->can_data.v, curr_state_ptr->can_data.w;
      CANMeasurement can_reading_2;
      can_reading_2 << next_state_ptr->can_data.v, next_state_ptr->can_data.w;
      float64_t dt =
          (next_state_ptr->t_ns - curr_state_ptr->t_ns) * kNanoSecToSec;
      assert(dt > 0.0);
      can_integrator->RK4Integrator(can_reading_1, can_reading_2, dt);
      next_state_ptr->core_state = can_integrator->GetVehicleCoreState();
      next_state_ptr->P_cov = can_integrator->GetP();
    }
  }

  // update can_integrator main
  auto newest_state_ptr = state_buffer_.GetLast();
  if (newest_state_ptr == state_buffer_.GetInvalid()) {
    LC_LERROR(MSF) << "newest state is invalid, impossible";
    return LOC_LOCALIZATION_ERROR;
  }

  can_integ_impl_->SetVehicleCoreState(newest_state_ptr->core_state);
  can_integ_impl_->SetP(newest_state_ptr->P_cov);
  nominal_state_ = newest_state_ptr->core_state;

  return LOC_SUCCESS;
}

adLocStatus_t MSFCoreCAN::GetState(uint64_t obs_time, STATE_ptr* state_ptr) {
  float64_t obs_time_f = static_cast<float64_t>(obs_time) * kNanoSecToSec;
  // search the adjacent state in position
  StateBuffer_T::iterator_T iter = state_buffer_.GetIteratorClosest(obs_time_f);

  if (iter == state_buffer_.IteratorEnd()) {
    LC_LERROR(MSF) << "No nearest state found";
    return LOC_LOCALIZATION_ERROR;
  }
  float64_t time_diff = std::fabs(iter->first - obs_time_f);
  if (time_diff > 0.100) {
    LC_LERROR(MSF) << "No suitable nearest state found";
    return LOC_LOCALIZATION_ERROR;
  }

  // if the time difference is too large, interpolate a new state
  // else searched state is the closest
  if (time_diff < 0.001) {
    (*state_ptr) = iter->second;
  } else {
    STATE_ptr last_state_ptr = state_buffer_.GetClosestBefore(obs_time_f);
    STATE_ptr next_state_ptr = state_buffer_.GetClosestAfter(obs_time_f);
    if (last_state_ptr == state_buffer_.GetInvalid()) {
      LC_LERROR(MSF) << "last state is invalid";
      return LOC_LOCALIZATION_ERROR;
    }
    if (next_state_ptr == state_buffer_.GetInvalid()) {
      LC_LERROR(MSF) << "next state is invalid";
      return LOC_LOCALIZATION_ERROR;
    }

    STATE_ptr curr_state_ptr = std::make_shared<StateMeta>();
    // prepare the new state
    curr_state_ptr->t_ns = obs_time;
    // linearly interpolate the prediction sensor readings
    float64_t last_time_f = last_state_ptr->t_ns * kNanoSecToSec;
    float64_t next_time_f = next_state_ptr->t_ns * kNanoSecToSec;
    float64_t dt = next_time_f - last_time_f;
    assert(dt > 0.0);
    float front_scale = (next_time_f - obs_time_f) / dt;
    float back_scale = (obs_time_f - last_time_f) / dt;
    curr_state_ptr->can_data.t_ns = obs_time;
    curr_state_ptr->can_data.v = last_state_ptr->can_data.v * front_scale +
                                 next_state_ptr->can_data.v * back_scale;
    curr_state_ptr->can_data.w = last_state_ptr->can_data.w * front_scale +
                                 next_state_ptr->can_data.w * back_scale;

    // state propagation to current state
    std::unique_ptr<CANIntegrator> can_integrator =
        std::make_unique<CANIntegrator>();
    can_integrator->SetVehicleCoreState(last_state_ptr->core_state);
    can_integrator->SetP(last_state_ptr->P_cov);
    CANMeasurement can_reading_1;
    can_reading_1 << last_state_ptr->can_data.v, last_state_ptr->can_data.w;
    CANMeasurement can_reading_2;
    can_reading_2 << curr_state_ptr->can_data.v, curr_state_ptr->can_data.w;
    can_integrator->RK4Integrator(can_reading_1, can_reading_2,
                                  obs_time_f - last_time_f);
    curr_state_ptr->core_state = can_integrator->GetVehicleCoreState();
    curr_state_ptr->P_cov = can_integrator->GetP();

    (*state_ptr) = curr_state_ptr;
    // insert to state buffer
    state_buffer_.insert(curr_state_ptr);
  }

  return LOC_SUCCESS;
}

adLocStatus_t MSFCoreCAN::RobustKalmanFilter(
    const std::shared_ptr<MSFFactor>& msf_factor, STATE_ptr* state_ptr) {
  // this state prediction
  const Eigen::VectorXd& predict_state = (*state_ptr)->core_state;
  CANPMat P = (*state_ptr)->P_cov;

  // Get H matrix
  msf_factor->EvaluateHMat(predict_state);
  Eigen::MatrixXd H = msf_factor->GetHMat();
  // update kalman gain
  Eigen::MatrixXd V = msf_factor->GetVMat();
  Eigen::MatrixXd S = H * P * H.transpose() + V;
  Eigen::MatrixXd S_inverse = S.inverse();
  Eigen::MatrixXd K = P * H.transpose() * S_inverse;

  // evaluate current residual
  VectorXd obs_error = msf_factor->EvaluateRes(predict_state);
  size_t dim = obs_error.rows();
  assert(dim <= 7);

  // outlier SMM observation rejection
  if (msf_factor->GetSource() == SMM) {
    // XY error
    Eigen::Vector2d obs_error_h = obs_error.segment<2>(0);
    double xy_chi2 = obs_error_h.dot(S_inverse.block<2, 2>(0, 0) * obs_error_h);
    if (xy_chi2 > 160) {
      LC_LERROR(MSF) << "outlier observation: "
                     << LocatorTypeToString(msf_factor->GetSource());
      return LOC_LOCALIZATION_ERROR;
    } else if (xy_chi2 > chi2_95s_[dim - 1]) {
      V *= 25;
      S = H * P * H.transpose() + V;
      S_inverse = S.inverse();
      K = P * H.transpose() * S_inverse;
      LC_LWARN(MSF) << "enlarged observation noise: ";
    }
  } else if (msf_factor->GetSource() == GNSS) {
    // XYZ error
    double xyz_chi2 = obs_error.dot(S_inverse * obs_error);
    if (xyz_chi2 > 160) {
      LC_LERROR(MSF) << "outlier observation: "
                     << LocatorTypeToString(msf_factor->GetSource());

      return LOC_LOCALIZATION_ERROR;
    } else if (xyz_chi2 > chi2_95s_[dim]) {
      V *= 20;
      S = H * P * H.transpose() + V;
      S_inverse = S.inverse();
      K = P * H.transpose() * S_inverse;
      LC_LWARN(MSF) << "enlarged observation noise: ";
    }
  }

  // injection of the observed error into the nominal state
  error_state_ = K * obs_error;
  nominal_state_ = predict_state;
  InjectionErrorState();
  (*state_ptr)->core_state = nominal_state_;

  // in case of much too small state uncertainty
  if (std::sqrt(P(0, 0)) < 0.002 || std::sqrt(P(1, 1)) < 0.002 ||
      std::sqrt(P(2, 2)) < 0.002) {
    return LOC_SUCCESS;
  }
  if (std::sqrt(P(3, 3)) < 0.001 || std::sqrt(P(4, 4)) < 0.001 ||
      std::sqrt(P(5, 5)) < 0.001) {
    return LOC_SUCCESS;
  }

  // update state covariance
  Eigen::Matrix<double, kcErrorStateSize, kcErrorStateSize> I;
  I.setIdentity();
  P = (I - K * H) * P * (I - K * H).transpose() + K * V * K.transpose();
  // to ensure symmetry
  P = (P + P.transpose()) / 2;
  (*state_ptr)->P_cov = P;

  return LOC_SUCCESS;
}

adLocStatus_t MSFCoreCAN::InjectionErrorState() {
  // update position
  Vector3d p = GetPosition();
  p += error_state_.segment<3>(kcErrorStatePosition);
  // update orientation
  Quaterniond dq =
      SO3d::exp(error_state_.segment<3>(kcErrorStateQuat)).unit_quaternion();
  Quaterniond q = GetQuat();
  q = q * dq;
  q.normalize();
  static auto sgn = [](double a) { return (a < 0.0 ? -1 : 1); };
  // update velocity scale
  Vector3d vs = GetVelScale();
  vs += error_state_.segment<3>(kcErrorStateVs);
  vs(1) = vs(2) = 0.0;
  // limit scale magnitude
  if (vs.norm() > 0.1) {
    vs(0) = sgn(vs(0)) * 0.1;
  }
  // update angular bias
  Vector3d wb = GetOmegaBias();
  wb += error_state_.segment<3>(kcErrorStateWb);
  wb(0) = wb(1) = 0.0;
  // limit bias magnitude
  if (wb.norm() > 0.01) {
    wb(2) = sgn(wb(2)) * 0.01;
  }

  SetPosition(p);
  SetQuat(q);
  SetVelScale(vs);
  SetOmegaBias(wb);
  error_state_.setZero();

  return LOC_SUCCESS;
}

}  // namespace msf
}  // namespace localization
}  // namespace senseAD
