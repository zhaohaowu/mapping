/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * Wang Xiaofeng <wangxiaofeng@sensetime.com>
 * Zhao Ming <zhaoming@senseauto.com>
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <deque>
#include <list>
#include <map>
#include <memory>
#include <set>
#include <utility>
#include <vector>

#include <Sophus/se3.hpp>

#include "ad_time/ad_time.hpp"
#include "common/msf_common.hpp"
#include "common/sorted_container.hpp"
#include "imu/sins.hpp"
#include "localization/data_type/base.hpp"
#include "localization/localization_param.hpp"
#include "msf/msf_core/msf_core.hpp"
#include "msf/msf_factor/msf_factor.hpp"

namespace senseAD {
namespace localization {

class EvaluatorImuIntrinsic;
class EvaluatorGnssBias;
class BaseLocator;

namespace msf {

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::Vector4d;

// Multi-Sensor Fusion for IMU front end
class MSFCoreIMU : public MSFCore {
 public:
  // each filter status
  enum class FilterStatus {
    GOOD,     // performs good, can be choosen as sole output
    NORMAL,   // performs normal, using fusion output is better
    TERRIBLE  // had divergenced, need to restart
  };

  // struct to store imu data and time
  struct IMUData {
    uint64_t t_ns;  // timestamp, unit: ns
    Vector3d a;     // linear acceleration from IMU
    Vector3d w;     // angular velocity from IMU
  };

  // struct of specific time state
  struct StateMeta {
    uint64_t t_ns;            // timestamp of this state estimate
    IMUData imu_data;         // IMU data measurements
    IMUCoreState core_state;  // core state estimation
    IMUPMat P_cov;            // Error state covariance
  };

  // struct of observation factor
  struct ObservationFactor {
    uint64_t t_ns;                      // timestamp of this factor
    std::shared_ptr<MSFFactor> factor;  // observation factor
  };

  MSFCoreIMU() = delete;

  MSFCoreIMU(std::shared_ptr<BaseLocator> locator,
             const LocalizationParam& param, bool is_main_filter = true,
             bool different_Q = false);

  ~MSFCoreIMU();

  adLocStatus_t AddSource(const std::map<STATE_SOURCE, uint64_t>& obs_srcs);

  void SetSINS(std::shared_ptr<SINS> sins) {
    *sins_ = *sins;  // deep copy
    if (different_Q_) UpdateSINSNoiseCov();
  }
  std::shared_ptr<SINS> GetSINS() { return sins_; }

  LocalizationParam GetParam() { return param_; }

  void InsertState(const StateMeta& state) {
    state_buffer_.insert(std::make_shared<StateMeta>(state));
  }
  StateMeta GetNewestState() { return *state_buffer_.GetLast(); }

  FilterStatus GetFilterStatus() const { return filter_status_; }

  void Restart();

  adLocStatus_t SwitchOriginProc() override;

  adLocStatus_t AddData(uint64_t time_ns, const VectorXd& imu_reading,
                        bool do_state_predict) override;

  adLocStatus_t StateUpdate(
      uint64_t timestamp,
      const std::shared_ptr<MSFFactor>& obs_factor) override;

  adLocStatus_t GetLatestMaxErrorState(Eigen::Vector4d* error_state) override;

  adLocStatus_t SearchNominalState(uint64_t time_ns,
                                   NavState* nav_state) override;

 private:
  // struct of error state (only contains x,y,heading)
  struct ErrorState {
    uint64_t t_ns;                // timestamp, unit: ns
    Eigen::Vector4d error_state;  // error state (x,y,z,heading)
    Eigen::Vector3d cov;          // covariance (x,y,z)
  };

  using STATE_ptr = std::shared_ptr<StateMeta>;
  using obs_ptr = std::shared_ptr<ObservationFactor>;
  using error_state_ptr = std::shared_ptr<ErrorState>;
  // Type of the state buffer to contian all the states
  using StateBuffer_T = SortedContainer<StateMeta>;
  // Type of the observation buffer to contain all the observations
  using ObservationBuffer_T = SortedContainer<ObservationFactor>;
  // Type of the error state buffer to contain all the states
  using ErrorStateBuffer_T = SortedContainer<ErrorState>;

  void UpdateSINSNoiseCov();

  adLocStatus_t CheckZUPT(obs_ptr obs_factor);

  // get state interface for state interpolate
  adLocStatus_t GetState(uint64_t obs_time, STATE_ptr* state_ptr);

  ////////////////// robust kalman filter related function ///////////////////

  adLocStatus_t RobustKalmanFilter(uint64_t timestamp,
                                   const std::shared_ptr<MSFFactor>& msf_factor,
                                   STATE_ptr* state_ptr);

  void ConsistencyStrategy(uint64_t timestamp,
                           const IMUCoreState& predict_state,
                           std::shared_ptr<MSFFactor> msf_factor);

  void RobustnessAndAdaptivenessStrategy(const STATE_SOURCE& source,
                                         const IMUCoreState& predict_state,
                                         const Eigen::VectorXd& obs_error,
                                         const Eigen::MatrixXd& H,
                                         Eigen::MatrixXd* V, IMUPMat* P);

  void SmoothnessStrategy(const IMUErrorState& error_state,
                          const IMUCoreState& predict_state,
                          const IMUPMat& predict_P,
                          IMUErrorState* scaled_error_state,
                          IMUCoreState* posterior_state, IMUPMat* posterior_P);

  void PostProcessForErrorState(const STATE_SOURCE& source, uint64_t timestamp,
                                const IMUErrorState& error_state,
                                const IMUCoreState& state, const IMUPMat& P);

 private:
  LocalizationParam param_;

  // sins instance, for multi-filters, it's a deep copy; for single-filter,
  // it's a shallow copy
  std::shared_ptr<SINS> sins_;

  // main filter flag
  bool is_main_filter_;

  // filter algorithm related variable
  bool different_Q_ = false;

  // filter status
  FilterStatus filter_status_ = FilterStatus::GOOD;

  // filter buffer containing all states information sorted by time
  StateBuffer_T state_buffer_;

  // all the history observations
  ObservationBuffer_T observation_buffer_;

  // all the history update error state (longitude, lateral, heading)
  std::mutex error_state_mutex_;
  ErrorStateBuffer_T error_state_buffer_;

  // observation factor related variable
  std::set<STATE_SOURCE> obs_sources_;
  std::map<STATE_SOURCE, uint64_t> main_obs_freqs_;

  // main obs timestamp record for local filter
  senseAD::Time last_main_obs_time_;

  // MM factor buffer for consistency check with gnss factor
  static constexpr size_t mmf_window_size_{10};
  bool has_mm_backend_ = false;
  std::deque<std::pair<uint64_t, std::shared_ptr<MSFFactor>>> smm_factors_;

  // zupt factor related variable
  uint64_t last_zupt_obs_time_ = 0;
  bool during_vehicle_static_ = false;
  IMUPMat fixed_P_ = IMUPMat::Identity();
  std::shared_ptr<MSFFactor> zupt_factor_ = nullptr;

  // for imu intrinsic evaluation
  std::shared_ptr<EvaluatorImuIntrinsic> imu_evaluator_;
  // for gnss bias evaluation
  std::shared_ptr<EvaluatorGnssBias> gnss_bias_evaluator_;

  // chi-square test table
  double chi2_75s_[8] = {0.0, 1.32, 2.77, 4.11, 5.39, 6.63, 7.84, 9.04};
  double chi2_95s_[8] = {0.0, 3.84, 5.99, 7.81, 9.49, 11.07, 12.59, 14.07};
};

}  // namespace msf
}  // namespace localization
}  // namespace senseAD
