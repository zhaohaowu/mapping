/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * Wang Xiaofeng <wangxiaofeng@sensetime.com>
 * Zhao Ming <zhaoming@senseauto.com>
 */

#include "msf/msf_core/msf_core_imu/msf_core_imu.hpp"

#include <algorithm>
#include <chrono>  // NOLINT
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <Sophus/so3.hpp>

#include "ad_common/config_utils/config_utils.hpp"
#include "ad_common/config_utils/macros.hpp"
#include "ad_time/ad_time.hpp"
#include "common/coordinate_converter.hpp"
#include "common/msf_serialization.hpp"
#include "common/path_util.hpp"
#include "common/transform_config.hpp"
#include "common/utility.hpp"
#include "eval/evaluator_gnss_bias.hpp"
#include "eval/evaluator_imu_intrinsic.hpp"
#include "imu/imu_locator.hpp"
#include "localization/common/log.hpp"
#include "msf/msf_factor/msf_factor_imu/msf_factor_imu_mockfull.hpp"

namespace senseAD {
namespace localization {
namespace msf {

class MSFFactorIMUMOCKFULL;

MSFCoreIMU::MSFCoreIMU(std::shared_ptr<BaseLocator> locator,
                       const LocalizationParam& param, bool is_main_filter,
                       bool different_Q)
    : MSFCore(),
      param_(param),
      is_main_filter_(is_main_filter),
      different_Q_(different_Q) {
  std::vector<bool> back_locator_type_ =
      FindLocatorTypeFromString(param_.common_param.back_locator_type);
  has_mm_backend_ = back_locator_type_[SMM];

  sins_.reset(new SINS(*(std::dynamic_pointer_cast<IMULocator>(locator)
                             ->GetSINS())));  // deep copy
  if (different_Q) UpdateSINSNoiseCov();
  if (param_.ci_param.enable_evaluation && is_main_filter_) {
    imu_evaluator_.reset(new EvaluatorImuIntrinsic);
    imu_evaluator_->Init(param_.ci_param.results_save_dir, "IMU_INTRINSIC",
                         param_.ci_param.testcase_id);
    gnss_bias_evaluator_.reset(new EvaluatorGnssBias);
    gnss_bias_evaluator_->Init(param_.ci_param.results_save_dir, "GNSS_BIAS",
                               param_.ci_param.testcase_id);
  }
}

MSFCoreIMU::~MSFCoreIMU() {}

adLocStatus_t MSFCoreIMU::AddSource(
    const std::map<STATE_SOURCE, uint64_t>& obs_srcs) {
  for (const auto& obs : obs_srcs) {
    obs_sources_.insert(obs.first);
    if (obs.first == GNSS || obs.first == SMM) {
      main_obs_freqs_.insert(obs);
    }
  }
  return LOC_SUCCESS;
}

void MSFCoreIMU::Restart() {
  if (different_Q_) UpdateSINSNoiseCov();

  // clear buffer
  state_buffer_.clear();
  observation_buffer_.clear();

  // reset state
  filter_status_ = FilterStatus::GOOD;

  last_zupt_obs_time_ = 0;
  during_vehicle_static_ = false;
  fixed_P_ = IMUPMat::Identity();

  last_main_obs_time_ = Time();
}

adLocStatus_t MSFCoreIMU::SwitchOriginProc() {
  // convert enu position to new origin
  for (auto iter = observation_buffer_.IteratorBegin();
       iter != observation_buffer_.IteratorEnd(); ++iter) {
    auto obs_factor = iter->second->factor;
    auto source = obs_factor->GetSource();
    if (source == SMM || source == MOCK) {
      Eigen::VectorXd state = obs_factor->GetObservationState();
      Eigen::Vector3d new_enu;
      CoordinateConverter::GetInstance()->ConvertENUToNewOrigin(
          state.segment<3>(0), &new_enu);
      state.segment<3>(0) = new_enu;
      obs_factor->SetObservationState(state);
    }
  }
  for (auto& item : smm_factors_) {
    Eigen::VectorXd state = item.second->GetObservationState();
    Eigen::Vector3d new_enu;
    CoordinateConverter::GetInstance()->ConvertENUToNewOrigin(
        state.segment<3>(0), &new_enu);
    state.segment<3>(0) = new_enu;
    item.second->SetObservationState(state);
  }
  if (zupt_factor_) {
    Eigen::VectorXd state = zupt_factor_->GetObservationState();
    Eigen::Vector3d new_enu;
    CoordinateConverter::GetInstance()->ConvertENUToNewOrigin(
        state.segment<3>(0), &new_enu);
    state.segment<3>(0) = new_enu;
    zupt_factor_->SetObservationState(state);
  }

  return LOC_SUCCESS;
}

adLocStatus_t MSFCoreIMU::AddData(uint64_t time_ns, const VectorXd& imu_reading,
                                  bool do_state_predict) {
  if (do_state_predict) {
    sins_->PVAUpdate(time_ns, imu_reading, true);
  }

  IMUData imu_data;
  imu_data.t_ns = time_ns;
  imu_data.a = imu_reading.segment<3>(0);
  imu_data.w = imu_reading.segment<3>(3);

  // add new state into state buffer
  STATE_ptr new_state_ptr = std::make_unique<StateMeta>();
  new_state_ptr->t_ns = time_ns;
  new_state_ptr->imu_data = imu_data;
  new_state_ptr->core_state = sins_->GetCoreState();
  new_state_ptr->P_cov = sins_->GetPMat();
  state_buffer_.insert(new_state_ptr);
  // remove very old states if necessary
  state_buffer_.ClearOlderThan(1.0);

  // save imu intrinsic evaluation results
  if (is_main_filter_ && imu_evaluator_) {
    ImuIntrinsicEvalData data;
    data.acc_bias = new_state_ptr->core_state.segment<3>(kiStateAccBias);
    data.gyro_bias = new_state_ptr->core_state.segment<3>(kiStateGyroBias);
    data.acc_scale = new_state_ptr->core_state.segment<3>(kiStateAccScale);
    data.gyro_scale = new_state_ptr->core_state.segment<3>(kiStateGyroScale);
    imu_evaluator_->WriteResult(new_state_ptr->t_ns * kNanoSecToSec, data);
    // frd 2 flu
    double gnss_bias = -new_state_ptr->core_state(kiStateGnssBias);
    double gnss_bias_std = std::sqrt(
        new_state_ptr->P_cov(kiErrorStateGnssBias, kiErrorStateGnssBias));
    Eigen::Vector2d gnss_bias_data(gnss_bias, gnss_bias_std);
    gnss_bias_evaluator_->WriteResult(new_state_ptr->t_ns * kNanoSecToSec,
                                      gnss_bias_data);
  }

  return LOC_SUCCESS;
}

adLocStatus_t MSFCoreIMU::StateUpdate(
    uint64_t timestamp, const std::shared_ptr<MSFFactor>& obs_factor) {
  if (!obs_sources_.count(obs_factor->GetSource())) {
    return LOC_LOCALIZATION_ERROR;
  }

  // check if current observation is much too delayed
  if (timestamp < state_buffer_.GetFirst()->t_ns) {
    LC_LDEBUG(MSF) << LocatorTypeToString(obs_factor->GetSource())
                   << " observation behinds too much, ts: " << timestamp;
    return LOC_LOCALIZATION_ERROR;
  }

  // check observation incoming timestamp
  senseAD::Time curr_obs_time = senseAD::time::Now();
  if (last_main_obs_time_.ToSec() > 1.0) {
    float64_t time_gap = curr_obs_time.ToSec() - last_main_obs_time_.ToSec();
    if (time_gap > 10.0) {
      LC_LERROR(MSF) << "main observation incoming too later, time gap: "
                     << time_gap;
    }
  }
  if (main_obs_freqs_.count(obs_factor->GetSource())) {
    last_main_obs_time_ = curr_obs_time;
  }

  obs_ptr obs_factor_ptr = std::make_shared<ObservationFactor>();
  obs_factor_ptr->t_ns = timestamp;
  obs_factor_ptr->factor = obs_factor;

  // check zupt, if has, re-make zupt factor and update related variables
  if (CheckZUPT(obs_factor_ptr) != LOC_SUCCESS) return LOC_LOCALIZATION_ERROR;

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
      LC_LDEBUG(MSF) << "get state failed, unused observations";
      continue;
    }

    // if the vehicle is static, only using zero velocity observation
    bool do_state_update = true;
    if (during_vehicle_static_ == true &&
        iter_obs_curr->second->factor->GetSource() != MOCK) {
      do_state_update = false;
    }
    // else don't use the zero velocity observation
    if (during_vehicle_static_ == false &&
        iter_obs_curr->second->factor->GetSource() == MOCK) {
      do_state_update = false;
    }

    // do observation update here with robust strategy
    if (do_state_update) {
      RobustKalmanFilter(curr_obs_time, iter_obs_curr->second->factor,
                         &curr_state_ptr);
    }

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
      LC_LDEBUG(MSF) << "why added new state can't find ?";
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
    SINS sins_tmp(*sins_);
    for (;
         iter_state_curr != iter_state_end && iter_state_next != iter_state_end;
         ++iter_state_curr, ++iter_state_next) {
      auto curr_state_ptr = iter_state_curr->second;
      auto next_state_ptr = iter_state_next->second;
      // state propagation to current state
      sins_tmp.SetCoreState(curr_state_ptr->core_state);
      sins_tmp.SetLastMeasurementTime(curr_state_ptr->t_ns);
      sins_tmp.SetPMat(curr_state_ptr->P_cov);
      IMUMeasurement imu_reading;
      imu_reading << next_state_ptr->imu_data.a, next_state_ptr->imu_data.w;
      sins_tmp.PVAUpdate(next_state_ptr->t_ns, imu_reading, true);
      next_state_ptr->core_state = sins_tmp.GetCoreState();
      next_state_ptr->P_cov = sins_tmp.GetPMat();
    }
  }

  // update sins
  auto newest_state_ptr = state_buffer_.GetLast();
  if (newest_state_ptr == state_buffer_.GetInvalid()) {
    LC_LERROR(MSF) << "newest state is invalid, impossible";
    return LOC_LOCALIZATION_ERROR;
  }
  sins_->SetCoreState(newest_state_ptr->core_state);
  sins_->SetPMat(newest_state_ptr->P_cov);

  return LOC_SUCCESS;
}

adLocStatus_t MSFCoreIMU::GetLatestMaxErrorState(Eigen::Vector4d* error_state) {
  std::lock_guard<std::mutex> lock(error_state_mutex_);
  if (error_state_buffer_.empty()) return LOC_LOCALIZATION_ERROR;
  Eigen::Vector4d max_err_state(0, 0, 0, 0);
  // search range: latest 1 second
  double time_point = error_state_buffer_.GetLast()->t_ns * 1e-9 - 1.0;
  auto iter = error_state_buffer_.GetIteratorClosestAfter(time_point);
  for (; iter != error_state_buffer_.IteratorEnd(); ++iter) {
    Eigen::Vector4d err = iter->second->error_state;
    for (int i = 0; i <= 3; ++i) {  // x, y, z, heading
      if (std::fabs(err(i)) > std::fabs(max_err_state(i)))
        max_err_state(i) = err(i);
    }
  }
  *error_state = max_err_state;
  return LOC_SUCCESS;
}

adLocStatus_t MSFCoreIMU::SearchNominalState(uint64_t time_ns,
                                             NavState* nav_state) {
  if (nav_state == nullptr) {
    LC_LERROR(MSF) << "nullptr";
    return LOC_LOCALIZATION_ERROR;
  }

  if (state_buffer_.empty()) {
    LC_LERROR(MSF) << "search failed, state buffer is empty";
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
    LC_LDEBUG(MSF) << "search state failed";
    return LOC_LOCALIZATION_ERROR;
  }

  nav_state->timestamp = time_ns;

  // global vehicle pose
  Vector3d sins_p = search_state_ptr->core_state.segment<3>(kiStatePosition);
  Quaterniond sins_a = Utility::HamiltonVecToEigenQ(
      search_state_ptr->core_state.segment<4>(kiStateAttitude));
  SE3d vehicle_pose;
  SINS::SINSPAtoGlobalSE3(sins_p, sins_a, &vehicle_pose);
  nav_state->pose = vehicle_pose;

  SE3d state_ned_vehicle;
  TransformConfig::FromRefENUToNED(vehicle_pose, &state_ned_vehicle, true);

  // velocity output relative to current vehicle frame
  Vector3d sins_v = search_state_ptr->core_state.segment<3>(kiStateVelocity);
  SO3d R = state_ned_vehicle.so3().inverse();
  nav_state->linear_speed = R.matrix() * sins_v;

  // angular output relative to current vehicle frame
  R = TransformConfig::GetTvb().so3();
  nav_state->angular_speed = R.matrix() * search_state_ptr->imu_data.w;

  // acceleration output relative to current vehicle frame
  R = state_ned_vehicle.so3().inverse();
  nav_state->linear_acceleration = R.matrix() * sins_->GetINSData().an;

  // global ENU covariance
  Eigen::Matrix<double, 6, 6> pose_cov = Eigen::Matrix<double, 6, 6>::Zero();
  Matrix3d Tpr = TransformConfig::GetTref().so3().matrix() * sins_->GetTpr();
  pose_cov.block<3, 3>(0, 0) =
      Tpr * search_state_ptr->P_cov.topLeftCorner(3, 3) * Tpr.transpose();
  pose_cov.block<3, 3>(3, 3) = search_state_ptr->P_cov.block<3, 3>(6, 6);
  nav_state->pose_cov = pose_cov;

  // gnss bias
  nav_state->gnss_bias = -search_state_ptr->core_state(kiStateGnssBias);
  nav_state->gnss_bias_cov =
      search_state_ptr->P_cov(kiErrorStateGnssBias, kiErrorStateGnssBias);

  return LOC_SUCCESS;
}

void MSFCoreIMU::UpdateSINSNoiseCov() {
  auto square = [](double a) { return a * a; };
  sins_->MakeSystemNoiseCov(square(param_.msf_param.dq_imu_sigma_acc),
                            square(param_.msf_param.dq_imu_sigma_gyro),
                            square(param_.msf_param.dq_imu_sigma_acc_bias),
                            square(param_.msf_param.dq_imu_sigma_gyro_bias),
                            square(param_.msf_param.dq_imu_sigma_acc_scale),
                            square(param_.msf_param.dq_imu_sigma_gyro_scale));
}

adLocStatus_t MSFCoreIMU::CheckZUPT(obs_ptr obs_factor) {
  if (obs_factor->factor->GetSource() == MOCK) {
    // if this is the start of the zero velocity duration
    if (during_vehicle_static_ == false) {
      STATE_ptr this_state_ptr = nullptr;
      if (GetState(obs_factor->t_ns, &this_state_ptr) != LOC_SUCCESS) {
        LC_LDEBUG(MSF) << "get state failed, unused zupt observations";
        return LOC_LOCALIZATION_ERROR;
      }
      IMUCoreState core_state = this_state_ptr->core_state;

      // re-make zupt factor
      const Vector3d& sins_p = core_state.segment<3>(kiStatePosition);
      const Vector4d& sins_q_vec = core_state.segment<4>(kiStateAttitude);
      Quaterniond sins_a = Utility::HamiltonVecToEigenQ(sins_q_vec);
      SE3d vehicle_pose;
      SINS::SINSPAtoGlobalSE3(sins_p, sins_a, &vehicle_pose);
      Eigen::Matrix<double, 10, 1> full_obs;
      full_obs.segment<3>(0) = vehicle_pose.translation();
      full_obs.segment<3>(3) = Vector3d::Zero();
      SO3d R = vehicle_pose.so3();
      Vector4d quat_vec = Utility::EigenQtoHamiltonVec(R.unit_quaternion());
      full_obs.segment<4>(6) = quat_vec;

      zupt_factor_ = msf::MSFFactorFactory::CreateMSFFactor(IMU, MOCK);
      if (zupt_factor_ == nullptr) {
        LC_LERROR(MSF) << "unsupported fusion mode !!!";
        return LOC_LOCALIZATION_ERROR;
      }
      zupt_factor_->SetObservationState(full_obs);
      Eigen::Matrix<double, 9, 9> full_obs_cov;
      full_obs_cov.setIdentity();
      full_obs_cov.block<3, 3>(0, 0) *= 1e-15;    // position
      full_obs_cov.block<3, 3>(3, 3) *= 1.0e-10;  // velocity
      full_obs_cov.block<3, 3>(6, 6) *= 1.0e-20;  // attitude
      zupt_factor_->SetObservationCov(full_obs_cov);
      zupt_factor_->SetSource(MOCK);
    }
    obs_factor->factor = zupt_factor_;
    during_vehicle_static_ = true;
    last_zupt_obs_time_ = obs_factor->t_ns;
  } else {
    uint64_t curr_obs_time = obs_factor->t_ns;
    float64_t dt = (static_cast<float64_t>(curr_obs_time) -
                    static_cast<float64_t>(last_zupt_obs_time_)) *
                   kNanoSecToSec;
    // if this is the end of the zero velocity duration, should be greater
    // than ZUPT frequency
    if (dt > 0.2) {
      during_vehicle_static_ = false;
      zupt_factor_.reset();
      zupt_factor_ = nullptr;
    }
  }

  return LOC_SUCCESS;
}

adLocStatus_t MSFCoreIMU::GetState(uint64_t obs_time, STATE_ptr* state_ptr) {
  float64_t obs_time_f = static_cast<float64_t>(obs_time) * kNanoSecToSec;
  // search the adjacent state in position
  StateBuffer_T::iterator_T iter = state_buffer_.GetIteratorClosest(obs_time_f);

  if (iter == state_buffer_.IteratorEnd()) {
    LC_LDEBUG(MSF) << "No nearest state found";
    return LOC_LOCALIZATION_ERROR;
  }
  float64_t time_diff = std::fabs(iter->first - obs_time_f);
  if (time_diff > 0.100) {
    LC_LDEBUG(MSF) << "No suitable nearest state found";
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
      LC_LDEBUG(MSF) << "last state is invalid";
      return LOC_LOCALIZATION_ERROR;
    }
    if (next_state_ptr == state_buffer_.GetInvalid()) {
      LC_LDEBUG(MSF) << "next state is invalid";
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
    curr_state_ptr->imu_data.t_ns = obs_time;
    curr_state_ptr->imu_data.a = last_state_ptr->imu_data.a * front_scale +
                                 next_state_ptr->imu_data.a * back_scale;
    curr_state_ptr->imu_data.w = last_state_ptr->imu_data.w * front_scale +
                                 next_state_ptr->imu_data.w * back_scale;

    // state propagation to current state
    SINS sins_tmp(*sins_);
    sins_tmp.SetCoreState(last_state_ptr->core_state);
    sins_tmp.SetLastMeasurementTime(last_state_ptr->t_ns);
    sins_tmp.SetPMat(last_state_ptr->P_cov);
    IMUMeasurement imu_reaading;
    imu_reaading << curr_state_ptr->imu_data.a, curr_state_ptr->imu_data.w;
    sins_tmp.PVAUpdate(curr_state_ptr->t_ns, imu_reaading, true);
    curr_state_ptr->core_state = sins_tmp.GetCoreState();
    curr_state_ptr->P_cov = sins_tmp.GetPMat();

    (*state_ptr) = curr_state_ptr;
    // insert to state buffer
    state_buffer_.insert(curr_state_ptr);
  }

  return LOC_SUCCESS;
}

adLocStatus_t MSFCoreIMU::RobustKalmanFilter(
    uint64_t timestamp, const std::shared_ptr<MSFFactor>& msf_factor,
    STATE_ptr* state_ptr) {
  // this factor's state prediction
  IMUCoreState predict_state = (*state_ptr)->core_state;
  IMUPMat P = (*state_ptr)->P_cov;

  // fix covariance gnss bias related part
  static auto FixGnssBiasCov = [](IMUPMat* P, int index) {
    double factor = std::sqrt(1e-12 / (*P)(index, index));
    P->row(index) *= factor;
    P->col(index) *= factor;
  };

  if (!has_mm_backend_) {
    FixGnssBiasCov(&P, kiErrorStateGnssBias);
  } else if (!param_.msf_param.enable_gnss_bias_estimate) {
    FixGnssBiasCov(&P, kiErrorStateGnssBias);
    // consistency strategy: make sure mapmatching has consistency with GNSS
    ConsistencyStrategy(timestamp, predict_state, msf_factor);
  }

  // factor evaluate H and residual
  const auto& source = msf_factor->GetSource();
  msf_factor->EvaluateHMat(predict_state);
  Eigen::MatrixXd H = msf_factor->GetHMat();
  Eigen::MatrixXd V = msf_factor->GetVMat();
  Eigen::VectorXd obs_error = msf_factor->EvaluateRes(predict_state);

  // robustness and adaptiveness strategy: make sure fusion robust to
  // observation covariance and adaptive to predict covariance
  RobustnessAndAdaptivenessStrategy(source, predict_state, obs_error, H, &V,
                                    &P);

  // update kalman gain and error state
  Eigen::MatrixXd P_HT(kiErrorStateSize, obs_error.rows());
  Eigen::MatrixXd K(kiErrorStateSize, obs_error.rows());
  P_HT.noalias() = P * H.transpose();
  K.noalias() = P_HT * (H * P_HT + V).inverse();

  if (source == MOCK) {
    // update PVA only if MOCK
    int row = kiErrorStateSize - kiErrorStateAccBias - 1;
    K.middleRows(kiErrorStateAccBias, row).setZero();
  }

  IMUErrorState error_state;
  error_state.noalias() = K * obs_error;

  // update posterior covariance
  IMUPMat I_KH;
  I_KH.noalias() = IMUPMat::Identity() - K * H;
  IMUPMat posterior_P;
  posterior_P.noalias() = I_KH * P * I_KH.transpose() + K * V * K.transpose();
  posterior_P = (posterior_P + posterior_P.transpose()) / 2;

  // smoothness strategy: make sure fusion state smooth enough
  IMUErrorState scaled_error_state;
  IMUCoreState posterior_state;
  SmoothnessStrategy(error_state, predict_state, P, &scaled_error_state,
                     &posterior_state, &posterior_P);

  // postprocess for error state
  PostProcessForErrorState(source, timestamp, scaled_error_state,
                           posterior_state, posterior_P);

  // update state and covariance, not update cov as vehicle is static
  (*state_ptr)->core_state = posterior_state;
  (*state_ptr)->P_cov = source == MOCK ? fixed_P_ : posterior_P;
  fixed_P_ = (*state_ptr)->P_cov;

  return LOC_SUCCESS;
}

void MSFCoreIMU::ConsistencyStrategy(uint64_t timestamp,
                                     const IMUCoreState& predict_state,
                                     std::shared_ptr<MSFFactor> msf_factor) {
  const auto& source = msf_factor->GetSource();
  if (source != GNSS && source != SMM) return;

  if (source == SMM) {  // SMM factor process flow
    if (smm_factors_.empty()) {
      smm_factors_.emplace_back(timestamp, msf_factor);
      return;
    }
    double time_gap = timestamp * 1e-9 - smm_factors_.back().first * 1e-9;
    if (time_gap <= 0) return;
    if (time_gap > 0.5) smm_factors_.clear();
    smm_factors_.emplace_back(timestamp, msf_factor);
    if (smm_factors_.size() > mmf_window_size_) smm_factors_.pop_front();
  } else if (source == GNSS && smm_factors_.size() > static_cast<size_t>(2)) {
    // GNSS factor process flow with smm factor consistency check
    double time_gap =
        std::fabs(timestamp * 1e-9 - smm_factors_.back().first * 1e-9);
    if (time_gap >= 0.5) return;  // avoid interpolate too large timegap

    int64_t closer_idx = 0;
    auto closer_factor = smm_factors_.front();
    for (int64_t i = smm_factors_.size() - 3; i >= 0; --i) {
      if (smm_factors_.back().first - smm_factors_[i].first >= 1e8) {
        closer_idx = i;
        closer_factor = smm_factors_[i];
        break;
      }
    }
    // interpolate smm position for this gnss factor
    Eigen::Vector3d closer_p =
        closer_factor.second->GetObservationState().segment<3>(0);
    Eigen::Vector3d closest_p =
        smm_factors_.back().second->GetObservationState().segment<3>(0);
    double factor = 1.0 * (timestamp - closer_factor.first) /
                    (smm_factors_.back().first - closer_factor.first);
    Eigen::Vector3d t_enu = factor * closest_p + (1.0 - factor) * closer_p;
    Eigen::Vector3d itp_position(t_enu(1), t_enu(0), -t_enu(2));
    // calculate interpolate smm lateral position variance
    double smm_ave_cov = 0;
    for (const auto& factor : smm_factors_) {
      smm_ave_cov += factor.second->GetObservationCov()(1, 1);
    }
    smm_ave_cov /= smm_factors_.size();
    double itp_lat_cov = std::min(
        smm_ave_cov, smm_factors_.back().second->GetObservationCov()(1, 1));
    Eigen::Matrix3d rot =
        Utility::HamiltonVecToEigenQ(predict_state.segment<4>(kiStateAttitude))
            .toRotationMatrix();
    // convert gnss related position and covariance
    PointENU_t enu;
    PointLLH_t lla;
    lla.lat = msf_factor->GetObservationState()(0) * r2d;
    lla.lon = msf_factor->GetObservationState()(1) * r2d;
    lla.height = msf_factor->GetObservationState()(2);
    CoordinateConverter::GetInstance()->LLA2ENU(lla, &enu);
    Eigen::Vector3d t_ned;
    t_ned << enu.y, enu.x, -enu.z;
    Eigen::Vector3d t_rfu_bv =
        TransformConfig::GetTvb().inverse().translation();
    Eigen::Vector3d tav =
        Eigen::Vector3d(t_rfu_bv(1), t_rfu_bv(0), -t_rfu_bv(2)) -
        TransformConfig::GetLeverArm();
    Eigen::Vector3d gnss_position = t_ned + rot * tav;
    Eigen::MatrixXd gnss_obs_cov = msf_factor->GetObservationCov();
    Eigen::Matrix3d local_gnss_cov =
        rot.transpose() * gnss_obs_cov.block<3, 3>(0, 0) * rot;
    double lat_res = (rot.transpose() * (gnss_position - itp_position))(1, 1);
    // calculate curve for compensate lateral error
    size_t mid_idx = (closer_idx + smm_factors_.size() - 1) / 2;
    Eigen::Vector3d mid_p =
        smm_factors_[mid_idx].second->GetObservationState().segment<3>(0);
    double curve_angle = std::fabs((mid_p - closer_p).dot(closest_p - mid_p)) /
                         (mid_p - closer_p).norm() / (closest_p - mid_p).norm();
    double turning_direction = ((mid_p - closer_p).cross(closest_p - mid_p))(2);
    turning_direction /= std::fabs(turning_direction);
    curve_angle = std::min(2 * std::acos(curve_angle), 1.0);
    double comp_err =
        turning_direction * std::tan(curve_angle) * (t_enu - closest_p).norm();
    lat_res = lat_res - comp_err;
    double lat_mah_dist =
        lat_res * lat_res / (itp_lat_cov + local_gnss_cov(1, 1));
    // mahalanobia distance check, punish gnss lateral related covariance
    if (lat_mah_dist < chi2_95s_[1]) return;
    double scale =
        (lat_res * lat_res / chi2_95s_[1] - itp_lat_cov) / local_gnss_cov(1, 1);
    LC_LDEBUG(MSF) << "MSF gnss consistency problem with SMM, param: "
                   << lat_mah_dist << " " << lat_res << "/" << comp_err << "/"
                   << curve_angle << " " << itp_lat_cov << " "
                   << local_gnss_cov(1, 1) << " " << scale;
    local_gnss_cov.row(1) *= std::sqrt(scale);
    local_gnss_cov.col(1) *= std::sqrt(scale);
    Eigen::Matrix3d glb_gnss_cov = rot * local_gnss_cov * rot.transpose();
    gnss_obs_cov.block<3, 3>(0, 0) = glb_gnss_cov;
    msf_factor->SetObservationCov(gnss_obs_cov);
  }
}

void MSFCoreIMU::RobustnessAndAdaptivenessStrategy(
    const STATE_SOURCE& source, const IMUCoreState& predict_state,
    const Eigen::VectorXd& obs_error, const Eigen::MatrixXd& H,
    Eigen::MatrixXd* V, IMUPMat* P) {
  if (source == MOCK) return;

  const double balance_factor = source == GNSS ? 3.0 : 1.0;
  // calculate mahalanobia distance for chi-square check
  Eigen::MatrixXd HPHT = H * (*P) * H.transpose();
  Eigen::MatrixXd S = HPHT + (*V);
  Eigen::MatrixXd S_inverse = S.inverse();
  size_t dim = source == SMM ? 2 : obs_error.rows();
  Eigen::MatrixXd mah_dist = obs_error.transpose() * S_inverse * obs_error;

  // check pass, treat as inlier observation
  if (mah_dist(0, 0) <= chi2_95s_[dim] * balance_factor) return;

  // only consider robustness problem for not main observation
  if (source != SMM && source != GNSS) {
    double scale_S = mah_dist(0, 0) / (chi2_95s_[dim] * balance_factor);
    // refer to << Adaptive Two-Stage Extended Kalman Filter >> paper
    double scale_V =
        ((scale_S - 1) * HPHT + scale_S * (*V)).trace() / (*V).trace();
    *V *= scale_V;
    return;
  }

  LC_LDEBUG(MSF) << "MSF maybe has robust or adaptive problem, source: "
                 << LocatorTypeToString(source);

  // robustness problem: inconsistency with observation covariance, rescaling
  // adaptiveness problem: inconsistency with predict P covariance, rescaling

  // for main observation, consider certain individual observation is
  // contaminated by outlier, check lontitude, lateral, altitude, heading.
  const double RA_factor = 100.0;
  enum ProblemType { Robustness, Adaptiveness };

  // check position part: longitude, lateral, altitude
  Eigen::Matrix3d Tpr = sins_->GetTpr();
  Eigen::Matrix3d Tpr_inv = Tpr.inverse();
  Eigen::Matrix3d Tpr_T = Tpr.transpose();
  Eigen::Matrix3d Tpr_invT = Tpr.transpose().inverse();
  Eigen::Matrix3d rot =
      Utility::HamiltonVecToEigenQ(predict_state.segment<4>(kiStateAttitude))
          .toRotationMatrix();
  Eigen::Matrix3d rot_inv = rot.transpose();
  Eigen::Vector3d local_p_obs_err = rot.transpose() * obs_error.segment<3>(0);
  Eigen::Matrix3d local_P = rot.transpose() * HPHT.topLeftCorner(3, 3) * rot;
  Eigen::Matrix3d local_V = rot.transpose() * (*V).topLeftCorner(3, 3) * rot;

  // check longitude part: adaptiveness or robustness problem
  ProblemType problem_type =
      local_P(0, 0) < RA_factor * local_V(0, 0) ? Robustness : Adaptiveness;
  double longitude_mah_dist =
      local_p_obs_err(0) * local_p_obs_err(0) / (local_P(0, 0) + local_V(0, 0));
  if (longitude_mah_dist > chi2_95s_[1] * balance_factor) {
    double scale_S = longitude_mah_dist / (chi2_95s_[1] * balance_factor);
    if (problem_type == Robustness) {
      double scale_V = scale_S + (scale_S - 1) * local_P(0, 0) / local_V(0, 0);
      LC_LDEBUG(MSF) << "MSF longitude robust problem, param: "
                     << longitude_mah_dist << " " << local_p_obs_err(0) << " "
                     << local_P(0, 0) << " " << local_V(0, 0) << " " << scale_V;
      // punish observation covariance at longitude dimension
      local_V.row(0) *= std::sqrt(scale_V);
      local_V.col(0) *= std::sqrt(scale_V);
      (*V).topLeftCorner(3, 3) = rot * local_V * rot.transpose();
    } else {
      double scale_P = scale_S + (scale_S - 1) * local_V(0, 0) / local_P(0, 0);
      double sqrt_scale_P = std::sqrt(scale_P);
      LC_LDEBUG(MSF) << "MSF longitude adaptive problem, param: "
                     << longitude_mah_dist << " " << local_p_obs_err(0) << " "
                     << local_P(0, 0) << " " << local_V(0, 0) << " " << scale_P;
      // rotate to local vehicle coordinate
      Eigen::Matrix3d pp = rot_inv * Tpr * P->topLeftCorner(3, 3) * Tpr_T * rot;
      pp.row(0) *= sqrt_scale_P;
      pp.col(0) *= sqrt_scale_P;
      (*P).topLeftCorner(3, 3) = Tpr_inv * rot * pp * rot_inv * Tpr_invT;
      for (int i = 1; i < 7; ++i) {
        Eigen::Matrix3d po = rot_inv * Tpr * P->block<3, 3>(0, 3 * i);
        pp.row(0) *= sqrt_scale_P;
        (*P).block<3, 3>(0, 3 * i) = Tpr_inv * rot * po;
        Eigen::Matrix3d op = P->block<3, 3>(3 * i, 0) * Tpr_T * rot;
        op.col(0) *= sqrt_scale_P;
        (*P).block<3, 3>(3 * i, 0) = op * rot_inv * Tpr_invT;
      }
    }
  }
  // check lateral part: adaptiveness or robustness problem
  problem_type =
      local_P(1, 1) < RA_factor * local_V(1, 1) ? Robustness : Adaptiveness;
  double lateral_mah_dist =
      local_p_obs_err(1) * local_p_obs_err(1) / (local_P(1, 1) + local_V(1, 1));
  if (lateral_mah_dist > chi2_95s_[1] * balance_factor) {
    double scale_S = lateral_mah_dist / (chi2_95s_[1] * balance_factor);
    if (problem_type == Robustness) {
      double scale_V = scale_S + (scale_S - 1) * local_P(1, 1) / local_V(1, 1);
      LC_LDEBUG(MSF) << "MSF lateral robust problem, param: "
                     << lateral_mah_dist << " " << local_p_obs_err(1) << " "
                     << local_P(1, 1) << " " << local_V(1, 1) << " " << scale_V;
      // punish observation covariance at lateral dimension
      local_V.row(1) *= std::sqrt(scale_V);
      local_V.col(1) *= std::sqrt(scale_V);
      (*V).topLeftCorner(3, 3) = rot * local_V * rot.transpose();
    } else {
      double scale_P = scale_S + (scale_S - 1) * local_V(1, 1) / local_P(1, 1);
      double sqrt_scale_P = std::sqrt(scale_P);
      LC_LDEBUG(MSF) << "MSF lateral adaptive problem, param: "
                     << lateral_mah_dist << " " << local_p_obs_err(1) << " "
                     << local_P(1, 1) << " " << local_V(1, 1) << " " << scale_P;
      // rotate to local vehicle coordinate
      Eigen::Matrix3d pp = rot_inv * Tpr * P->topLeftCorner(3, 3) * Tpr_T * rot;
      pp.row(1) *= sqrt_scale_P;
      pp.col(1) *= sqrt_scale_P;
      (*P).topLeftCorner(3, 3) = Tpr_inv * rot * pp * rot_inv * Tpr_invT;
      for (int i = 1; i < 7; ++i) {
        Eigen::Matrix3d po = rot_inv * Tpr * P->block<3, 3>(0, 3 * i);
        pp.row(1) *= sqrt_scale_P;
        (*P).block<3, 3>(0, 3 * i) = Tpr_inv * rot * po;
        Eigen::Matrix3d op = P->block<3, 3>(3 * i, 0) * Tpr_T * rot;
        op.col(1) *= sqrt_scale_P;
        (*P).block<3, 3>(3 * i, 0) = op * rot_inv * Tpr_invT;
      }
    }
  }

  // check altitude part: adaptiveness or robustness problem
  problem_type =
      local_P(2, 2) < RA_factor * local_V(2, 2) ? Robustness : Adaptiveness;
  double altitude_mah_dist =
      local_p_obs_err(2) * local_p_obs_err(2) / (local_P(2, 2) + local_V(2, 2));
  if (altitude_mah_dist > chi2_95s_[1] * balance_factor) {
    double scale_S = altitude_mah_dist / (chi2_95s_[1] * balance_factor);
    if (problem_type == Robustness) {
      double scale_V = scale_S + (scale_S - 1) * local_P(2, 2) / local_V(2, 2);
      LC_LDEBUG(MSF) << "MSF altitude robust problem, param: "
                     << altitude_mah_dist << " " << local_p_obs_err(2) << " "
                     << local_P(2, 2) << " " << local_V(2, 2) << " " << scale_V;
      // punish observation covariance at altitude dimension
      local_V.row(2) *= std::sqrt(scale_V);
      local_V.col(2) *= std::sqrt(scale_V);
      (*V).topLeftCorner(3, 3) = rot * local_V * rot.transpose();
    } else {
      double scale_P = scale_S + (scale_S - 1) * local_V(2, 2) / local_P(2, 2);
      LC_LDEBUG(MSF) << "MSF altitude adaptive problem, param: "
                     << altitude_mah_dist << " " << local_p_obs_err(2) << " "
                     << local_P(2, 2) << " " << local_V(2, 2) << " " << scale_P;
      (*P).row(kiErrorStatePosition + 2) *= std::sqrt(scale_P);
      (*P).col(kiErrorStatePosition + 2) *= std::sqrt(scale_P);
    }
  }

  // check rostation part(heading): : adaptiveness or robustness problem
  int id = source == SMM ? 5 : 3;  // 3 is for GNSS factor
  double hd_V = (*V)(id, id), hd_P = HPHT(id, id);
  double heading_mah_dist = obs_error(id) * obs_error(id) / (hd_P + hd_V);
  problem_type = hd_P < RA_factor * hd_V ? Robustness : Adaptiveness;
  if (heading_mah_dist > chi2_95s_[1] * balance_factor) {
    double scale_S = heading_mah_dist / (chi2_95s_[1] * balance_factor);
    if (problem_type == Robustness) {
      double scale_V = scale_S + (scale_S - 1) * hd_P / hd_V;
      LC_LDEBUG(MSF) << "MSF heading robust problem, param: "
                     << heading_mah_dist << " " << obs_error(id) << " " << hd_P
                     << " " << hd_V << " " << scale_V;
      (*V)(id, id) *= scale_V;  // punish obs covariance at heading dim
    } else {
      double scale_P = scale_S + (scale_S - 1) * hd_V / hd_P;
      LC_LDEBUG(MSF) << "MSF heading adaptive problem, param: "
                     << heading_mah_dist << " " << obs_error(id) << " " << hd_P
                     << " " << hd_V << " " << scale_P;
      (*P).row(kiErrorStateAttitude + 2) *= std::sqrt(scale_P);
      (*P).col(kiErrorStateAttitude + 2) *= std::sqrt(scale_P);
    }
  }
}

void MSFCoreIMU::SmoothnessStrategy(const IMUErrorState& error_state,
                                    const IMUCoreState& predict_state,
                                    const IMUPMat& predict_P,
                                    IMUErrorState* scaled_error_state,
                                    IMUCoreState* posterior_state,
                                    IMUPMat* posterior_P) {
  IMUErrorState beta_scale = IMUErrorState::Ones();

  // limit error state position amplitude
  Eigen::Matrix3d Tpr = sins_->GetTpr();
  Eigen::Matrix3d rot =
      Utility::HamiltonVecToEigenQ(predict_state.segment<4>(kiStateAttitude))
          .toRotationMatrix();

  Eigen::Matrix3d ps_cov =
      predict_P.block<3, 3>(kiErrorStatePosition, kiErrorStatePosition);
  Eigen::Matrix3d local_P =
      rot.transpose() * Tpr * ps_cov * Tpr.transpose() * rot;
  // note that not scaling for larger lateral std (LOW_ACCURACY)
  if (std::sqrt(local_P(1, 1)) < 0.3) {
    int I = kiErrorStatePosition;
    Eigen::Vector3d err_pos =
        (rot.transpose() * Tpr * error_state.segment<3>(I)).cwiseAbs();
    // using lateral dimension scaling position and velocity item
    double p_scale = err_pos(1) > 0.8 ? 1.0 : std::min(1.0, 0.05 / err_pos(1));
    beta_scale.segment<3>(I) *= p_scale;
    I = kiErrorStateVelocity;
    beta_scale.segment<3>(I) *= p_scale;
  }

  double rotz_cov = predict_P(kiErrorStateAttitude, kiErrorStateAttitude);
  if (std::sqrt(rotz_cov) < 0.05) {
    int I = kiErrorStateAttitude;
    Eigen::Vector3d err_rot = (error_state.segment<3>(I)).cwiseAbs();
    // using lateral dimension scaling attitude item
    double R_scale = err_rot(2) > 0.1 ? 1.0 : std::min(1.0, 0.01 / err_rot(2));
    beta_scale.segment<3>(I) *= R_scale;
  }

  // update scaled error state
  (*scaled_error_state).noalias() = error_state.cwiseProduct(beta_scale);

  std::stringstream strm;
  for (int i = 0; i < beta_scale.rows(); ++i) {
    if (beta_scale(i) < 1.0) strm << i << ":" << beta_scale(i) << "|";
  }
  if (strm.str() != "") LC_LDEBUG(MSF) << "MSF smooth scale: " << strm.str();

  // update posterior state and covariance
  IMUErrorState gamma_v = IMUErrorState::Ones() - beta_scale;
  IMUPMat gamma_m = gamma_v * gamma_v.transpose();

  SINS sins_curr(*sins_);
  sins_curr.SetCoreState(predict_state);
  sins_curr.StateCorrection(*scaled_error_state);
  *posterior_state = sins_curr.GetCoreState();
  *posterior_P = gamma_m.cwiseProduct(predict_P) +
                 (IMUPMat::Ones() - gamma_m).cwiseProduct(*posterior_P);
}

void MSFCoreIMU::PostProcessForErrorState(const STATE_SOURCE& source,
                                          uint64_t timestamp,
                                          const IMUErrorState& error_state,
                                          const IMUCoreState& state,
                                          const IMUPMat& P) {
  Eigen::Matrix3d rot =
      Utility::HamiltonVecToEigenQ(state.segment<4>(kiStateAttitude))
          .toRotationMatrix();
  Eigen::Matrix3d Tpr = sins_->GetTpr();

  // transform to FRD frame for error state
  Eigen::Vector4d err_xyzh;
  err_xyzh.head(3) = rot.transpose() * sins_->GetTpr() *
                     error_state.segment<3>(kiErrorStatePosition);
  err_xyzh(3) = error_state(kiErrorStateAttitude + 2);
  if (main_obs_freqs_.count(source)) {
    LC_LDEBUG(MSF) << "update error state: " << LocatorTypeToString(source)
                   << ", err: " << err_xyzh.transpose();
  }
  // transform to FRD frame for position covariance
  Eigen::Matrix3d ps_cov =
      P.block<3, 3>(kiErrorStatePosition, kiErrorStatePosition);
  Eigen::Matrix3d local_P =
      rot.transpose() * Tpr * ps_cov * Tpr.transpose() * rot;
  Eigen::Vector3d local_P_diag = local_P.diagonal();

  // record in window buffer for status check and estimated param saving check
  error_state_ptr err_ptr = std::make_shared<ErrorState>();
  err_ptr->t_ns = timestamp;
  err_ptr->error_state = err_xyzh;
  err_ptr->cov = local_P_diag;
  {
    std::lock_guard<std::mutex> lock(error_state_mutex_);
    error_state_buffer_.ClearOlderThan(3.0);
    error_state_buffer_.insert(err_ptr);
  }

  //////////////////// process for saving msf serial param ///////////////////

  // check if enable msf serial
  if (!param_.msf_param.enable_msf_serial) return;

  // check time interval
  static double last_saving_timestamp = timestamp * 1e-9;
  if (timestamp * 1e-9 - last_saving_timestamp < 60.0) return;

  // check velocity
  if (state.segment<3>(kiStateVelocity).norm() < 5.0) return;

  // check fusion consistency: error state and cov
  Eigen::Vector3d max_err_state(0, 0, 0);
  Eigen::Vector3d max_std(0, 0, 0);
  for (auto iter = error_state_buffer_.IteratorBegin();
       iter != error_state_buffer_.IteratorEnd(); ++iter) {
    Eigen::Vector4d err = iter->second->error_state.cwiseAbs();
    Eigen::Vector3d cov = iter->second->cov.cwiseAbs();
    for (int i = 0; i <= 2; ++i) {
      max_err_state(i) = std::max(max_err_state(i), err(i));
      max_std(i) = std::max(max_std(i), std::sqrt(cov(i)));
    }
  }
  if (max_err_state.norm() > 0.03 || max_std.norm() > 0.12) return;

  // saving msf estimated param (imu intrinsic and can scale etc)
  std::string msf_serial_path = param_.msf_param.msf_serial_save_path;
  std::string save_dir =
      msf_serial_path.substr(0, msf_serial_path.find_last_of("/"));
  if (!FileSystem::IsDir(save_dir)) {
    if (!FileSystem::MakeDirs(save_dir)) {
      LC_LERROR(MSF) << "make dir failed, path: " << save_dir;
      return;
    }
  }
  MsfSerialParam msf_serial_param;
  msf_serial_param.timestamp = timestamp;
  msf_serial_param.utc_time = senseAD::Time::ToString(timestamp);
  msf_serial_param.acc_bias_x = state(kiStateAccBias);
  msf_serial_param.acc_bias_y = state(kiStateAccBias + 1);
  msf_serial_param.acc_bias_z = state(kiStateAccBias + 2);
  msf_serial_param.gyro_bias_x = state(kiStateGyroBias);
  msf_serial_param.gyro_bias_y = state(kiStateGyroBias + 1);
  msf_serial_param.gyro_bias_z = state(kiStateGyroBias + 2);
  msf_serial_param.acc_scale_x = state(kiStateAccScale);
  msf_serial_param.acc_scale_y = state(kiStateAccScale + 1);
  msf_serial_param.acc_scale_z = state(kiStateAccScale + 2);
  msf_serial_param.gyro_scale_x = state(kiStateGyroScale);
  msf_serial_param.gyro_scale_y = state(kiStateGyroScale + 1);
  msf_serial_param.gyro_scale_z = state(kiStateGyroScale + 2);
  msf_serial_param.can_velocity_scale = 1.0 / (1.0 - state(kiStateWheelScale));
  if (senseAD::CONF_SUCCESS !=
      senseAD::common::utils::ConfigurationReader::WriteJSON(
          msf_serial_path, msf_serial_param)) {
    LC_LERROR(MSF) << "Failed to write msf serial param, path: "
                   << msf_serial_path;
    return;
  }

  last_saving_timestamp = timestamp * 1e-9;
  LC_LDEBUG(MSF) << "optim imu intrinsic param: "
                 << state.segment<12>(kiStateAccBias).transpose();
  LC_LDEBUG(MSF) << "optim can velocity scale param: "
                 << msf_serial_param.can_velocity_scale;
  LC_LDEBUG(MSF) << "Write msf serial param to file path: " << msf_serial_path;
}

}  // namespace msf
}  // namespace localization
}  // namespace senseAD
