/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Du Jiankui <dujiankui@senseauto.com>
 * Fang Yu <fangyu1@senseauto.com>
 */

#pragma once

#include <deque>
#include <list>
#include <memory>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Sophus/se3.hpp>

#include "common/msf_common.hpp"
#include "common/utility.hpp"
#include "dead_reckoning/dead_reckoning_locator.hpp"
#include "localization/common/log.hpp"

namespace senseAD {
namespace localization {
namespace dr {

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, kiErrorStateSizeDR, kiErrorStateSizeDR> FMat;
typedef Eigen::Matrix<double, kiErrorStateSizeDR, kiNoiseSizeDR> GMat;

class DRFilteringLocator : public DRLocator {
 public:
  DRFilteringLocator() = default;
  virtual ~DRFilteringLocator() = default;

  adLocStatus_t Init(const LocalizationParam& param) final;

  adLocStatus_t GetState(OdomState* odom_state,
                         double* confidence = nullptr) final;

  OdomStatus GetCurrentLocatorStatus() final;

  adLocStatus_t Process(uint64_t timestamp, std::shared_ptr<Imu> raw_imu) final;

 private:
  // @brief: init filter state and covariance, process noise
  adLocStatus_t FilterInit(uint64_t timestamp);

  // @brief: filter state predict with imu input
  adLocStatus_t FilterStatePredict(uint64_t timestamp,
                                   const IMUMeasurement& imu_frd);

  // @brief: filter covariance propagation with imu input
  adLocStatus_t FilterCovPropagation(double dt);

  // @brief: filter velocity update
  void FilterVelocityUpdate(uint64_t timestamp);

  // @brief: align gravity in body and navigation frame
  void FilterGravityAlign();

  // @brief: calculate vehicle longitudinal acceleration
  bool CalculateVehicleAccX(double* vehicle_acc_x);

  // @brief: vehicle static detection
  void VehicleStaticDetect(std::shared_ptr<Imu> raw_imu);

  // @brief: filter zupt update
  void FilterZeroVelocityUpdate();
  void FilterGyroBiasUpdate();
  void FilterZeroVelocityAngularRateUpdate();

  // @brief: kalman filter update step
  void FilterStateUpdate(const Eigen::VectorXd& Z, const Eigen::MatrixXd& H,
                         const Eigen::MatrixXd& R,
                         const std::vector<int>& state_dim_update);

  // @brief: transfer imu from RFU to FRD
  IMUMeasurement TransferIMURfuToFrd(std::shared_ptr<Imu> raw_imu);

  // @brief: query can data given timestamp
  adLocStatus_t QueryCanDataByTime(uint64_t timestamp,
                                   VehicleInfo* vehicle_info) const;

  // @brief: interpolate vehicle info
  void VehicleInfoInterp(const VehicleInfo& s_info, const VehicleInfo& e_info,
                         double factor, VehicleInfo* info) const;

  // @brief: calculate moving average and standard deviation for imu
  adLocStatus_t MovingImuStandardDeviation(const IMUMeasurement& imu_frd);

  // @brief: update max error state
  void UpdateMaxErrorState(uint64_t timestamp, const Vector6d& error_state);

  // @brief: update msf params
  void UpdateMSFParams();

  /////////////////////////// set & get interface ////////////////////////////

  Eigen::Vector3d GetPosition() const {
    return core_state_.segment<3>(kiStatePositionDR);
  }

  adLocStatus_t SetPosition(const Eigen::Vector3d& P) {
    core_state_.segment<3>(kiStatePositionDR) = P;
    return LOC_SUCCESS;
  }

  Eigen::Vector3d GetVelocity() const {
    return core_state_.segment<3>(kiStateVelocityDR);
  }

  adLocStatus_t SetVelocity(const Eigen::Vector3d& V) {
    core_state_.segment<3>(kiStateVelocityDR) = V;
    return LOC_SUCCESS;
  }

  Vector4d GetQuatVec() const {
    return core_state_.segment<4>(kiStateAttitudeDR);
  }

  Quaterniond GetQuat() const {
    return Utility::HamiltonVecToEigenQ(GetQuatVec());
  }

  adLocStatus_t SetQuat(const Eigen::Vector4d& q) {
    core_state_.segment<4>(kiStateAttitudeDR) = q;
    return LOC_SUCCESS;
  }

  adLocStatus_t SetQuat(const Eigen::Quaterniond& q) {
    Vector4d q_vec = Utility::EigenQtoHamiltonVec(q);
    SetQuat(q_vec);
    return LOC_SUCCESS;
  }

  Vector3d GetAccBias() const {
    return core_state_.segment<3>(kiStateAccBiasDR);
  }

  adLocStatus_t SetAccBias(const Eigen::Vector3d& ab) {
    core_state_.segment<3>(kiStateAccBiasDR) = ab;
    return LOC_SUCCESS;
  }

  Vector3d GetGyroBias() const {
    return core_state_.segment<3>(kiStateGyroBiasDR);
  }

  adLocStatus_t SetGyroBias(const Eigen::Vector3d& wb) {
    core_state_.segment<3>(kiStateGyroBiasDR) = wb;
    return LOC_SUCCESS;
  }

 private:
  bool is_inited_{false};       // flag for filter init
  uint64_t timestamp_ = 0;      // current imu time
  uint64_t last_imu_time_ = 0;  // last imu time
  double imu_dt_;               // imu sample period

  Eigen::Vector3d acc_corrected_;   // acc corrected in FRD by intrinsic
  Eigen::Vector3d gyro_corrected_;  // gyro corrected in FRD by intrinsic
  Eigen::Vector3d g_n_;             // gravity reaction in NED coordinate
  bool vehicle_static_{false};

  // data for moving imu standart deviation
  static constexpr size_t imu_deque_size_ = 100;
  boost::circular_buffer<IMUMeasurement> imu_deque_;
  IMUMeasurement imu_mean_;
  IMUMeasurement imu_square_sum_;
  IMUMeasurement imu_std_;

  // data for calculation acceleration from vehicle velocity
  static constexpr size_t kAccVelSize = 20;
  boost::circular_buffer<double> vel_list_;
  double curr_vehicle_vel_{0};

  // all the history update error state
  std::mutex error_state_mutex_;
  Vector6d max_error_state_;
  std::deque<std::pair<uint64_t, Vector6d>> error_state_buffer_;

  // msf related
  IMUErrorStateDR error_state_;   // strapdown inertial navigation error state
  IMUCoreStateDR core_state_;     // strapdown inertial navigation core state
  IMUPMatDR p_;                   // state covariance
  IMUQMatDR q_;                   // imu noise parameters
  FMat F_;                        // process matrix F
  GMat G_;                        // process matrix G
  Eigen::Vector3d Pn_posterior_;  // posterior vehicle position
  Eigen::Quaterniond Qnb_posterior_;  // posterior vehicle attitude
  double can_velocity_scale_{1.0};    // can velocity factor
};

}  // namespace dr
}  // namespace localization
}  // namespace senseAD
