/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * Wang Xiaofeng<wangxiaofeng@sensetime.com>
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <Sophus/se3.hpp>

#include "common/msf_common.hpp"
#include "common/utility.hpp"
#include "localization/data_type/base.hpp"

namespace senseAD {
namespace localization {

// using Sophus::SE3;
// using Sophus::SO3;

using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

using Vector1d = Eigen::Matrix<double, 1, 1>;
using Vector5d = Eigen::Matrix<double, 5, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector9d = Eigen::Matrix<double, 9, 1>;
using FMat = Eigen::Matrix<double, kiErrorStateSize, kiErrorStateSize>;
using GMat = Eigen::Matrix<double, kiErrorStateSize, kiNoiseSize>;

// define the World Geodetic System parameters
struct WGS84 {
  // Earth Nominal Mean Angular Velocity
  static constexpr double Wie = 7.292115e-5;
  // First eccentricity
  static constexpr double e = 0.0818191908426;
  static constexpr double e2 = e * e;
  // Semi-major Axis
  static constexpr double R = 6378137.0;
  // Semi-minor axis
  static constexpr double r = 6356752.0;
  // Flattening Factor of the Earth
  static constexpr double f = 1 / 298.257223563;
};

struct INSData {
  // previous corrected specific-force (b-frame) / angular rate (b-frame)
  Vector3d fbp, omgbp;
  // uncorrected specific-force (b-frame) / angular rate (b-frame)
  Vector3d fb0, omgb0;
  // corrected specific-force (b-frame) / angular rate (b-frame)
  Vector3d fb, omgb;
  // acceleration in n-frame
  Vector3d an;
};

// Strapdown Inertial Navigation System
class SINS {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static double g_norm_;  // Gravity magnitude

  SINS() = default;

  SINS(const SINS& sins);

  SINS& operator=(const SINS& sins);

  ///////////////////////// Core SINS Algorithm //////////////////////////////

  adLocStatus_t SetInitialState(double lat, double lon, double alt,
                                const Eigen::Vector3d& ypr,
                                const Vector3d& velocity, double gnss_bias);

  adLocStatus_t PVAUpdate(uint64_t timestamp, const IMUMeasurement& imu_reading,
                          bool with_cov_propagation);

  adLocStatus_t CovPropagation(double dt);

  adLocStatus_t CovPropagationSparse(double dt);

  adLocStatus_t StateCorrection(const IMUErrorState& X);

  ////////////////////////////////////////////////////////////////////////////

  void UpdateGeoParams();

  void RotscullCorr(const INSData& ins_data, double dt, Vector3d* dv,
                    Vector3d* da);

  adLocStatus_t SetLastMeasurementTime(uint64_t t_ns);

  ////////////////////// IMU core state set/get related //////////////////////

  IMUCoreState GetCoreState() const { return core_state_; }

  void SetCoreState(const IMUCoreState& core_state) {
    core_state_ = core_state;
    UpdateGeoParams();
  }

  Vector3d GetPosition() const {
    // latitude, longitude, geographic height
    return core_state_.segment<3>(kiStatePosition);
  }

  adLocStatus_t SetPosition(const Vector3d& P) {
    core_state_.segment<3>(kiStatePosition) = P;
    return LOC_SUCCESS;
  }

  Vector3d GetVelocity() const {
    // velocity in NED frame
    return core_state_.segment<3>(kiStateVelocity);
  }

  adLocStatus_t SetVelocity(const Vector3d& V) {
    core_state_.segment<3>(kiStateVelocity) = V;
    return LOC_SUCCESS;
  }

  Vector4d GetQuatVec() const {
    // attitude from body to NED frame (Cb_n --> Rnb)
    return core_state_.segment<4>(kiStateAttitude);
  }

  Quaterniond GetQuat() const {
    return Utility::HamiltonVecToEigenQ(GetQuatVec());
  }

  adLocStatus_t SetQuat(const Vector4d& q) {
    core_state_.segment<4>(kiStateAttitude) = q;
    return LOC_SUCCESS;
  }

  adLocStatus_t SetQuat(const Quaterniond& q) {
    Vector4d q_vec = Utility::EigenQtoHamiltonVec(q);
    SetQuat(q_vec);
    return LOC_SUCCESS;
  }

  Vector3d GetAccBias() const { return core_state_.segment<3>(kiStateAccBias); }

  adLocStatus_t SetAccBias(const Vector3d& ab) {
    core_state_.segment<3>(kiStateAccBias) = ab;
    return LOC_SUCCESS;
  }

  Vector3d GetGyroBias() const {
    return core_state_.segment<3>(kiStateGyroBias);
  }

  adLocStatus_t SetGyroBias(const Vector3d& wb) {
    core_state_.segment<3>(kiStateGyroBias) = wb;
    return LOC_SUCCESS;
  }

  Vector3d GetAccScale() const {
    return core_state_.segment<3>(kiStateAccScale);
  }

  adLocStatus_t SetAccScale(const Vector3d& as) {
    core_state_.segment<3>(kiStateAccScale) = as;
    return LOC_SUCCESS;
  }

  Vector3d GetGyroScale() const {
    return core_state_.segment<3>(kiStateGyroScale);
  }

  adLocStatus_t SetGyroScale(const Vector3d& gs) {
    core_state_.segment<3>(kiStateGyroScale) = gs;
    return LOC_SUCCESS;
  }

  Vector1d GetGnssBias() const {
    return core_state_.segment<1>(kiStateGnssBias);
  }

  adLocStatus_t SetGnssBias(const Vector1d& gb) {
    core_state_.segment<1>(kiStateGnssBias) = gb;
    return LOC_SUCCESS;
  }

  Vector1d GetWheelScale() const {
    return core_state_.segment<1>(kiStateWheelScale);
  }

  adLocStatus_t SetWheelScale(const Vector1d& ws) {
    core_state_.segment<1>(kiStateWheelScale) = ws;
    return LOC_SUCCESS;
  }

  ////////////////////////////////////////////////////////////////////////////

  IMUPMat GetPMat() const { return P_; }

  adLocStatus_t SetPMat(const IMUPMat& P) {
    P_ = P;
    return LOC_SUCCESS;
  }

  IMUQMat GetQMat() const { return Q_; }

  adLocStatus_t SetQMat(const IMUQMat& Q) {
    Q_ = Q;
    return LOC_SUCCESS;
  }

  Matrix3d GetTpr() const { return Tpr_; }

  INSData GetINSData() const { return ins_data_; }

  void MakeErrorStateCov(double p_v, double v_v, double a_v, double ab_v,
                         double gb_v, double as_v, double gs_v);

  void MakeSystemNoiseCov(double am_n, double gm_n, double ab_n, double gb_n,
                          double as_n, double gs_n);

  void InitSystemMatrix();

  static adLocStatus_t SINSPAtoGlobalSE3(const Vector3d& sins_p,
                                         const Quaterniond& sins_a,
                                         SE3d* vehicle_pose);

  static adLocStatus_t GlobalSE3toSINSPA(const SE3d& vehicle_pose,
                                         Vector3d* sins_p, Quaterniond* sins_a);

  static IMUMeasurement TransferIMURfuToFrd(const IMUMeasurement& imu_data);

 protected:
  // imu sample period
  double dt_;
  // last imu time
  uint64_t last_time_ = 0;

 private:
  // strapdown inertial navigation core state
  IMUCoreState core_state_;
  // error state
  IMUErrorState error_state_;
  // radius of curvature in meridian
  double Rn_;
  // radius of curvature in prime vertical
  double Re_;
  // navigation frame's rotation velocity w.r.t earth
  Vector3d Wen_n_;
  // earth frame's rotation velocity w.r.t inertial frame
  Vector3d Wie_n_;
  // navigation frame's rotation veloticy w.r.t inertial frame
  Vector3d Win_n_;
  // Gravity vector in NED
  Vector3d g_n_;
  // radians-to-meters matrix
  Matrix3d Tpr_ = Matrix3d::Zero();
  // imu noise parameters
  IMUQMat Q_;
  // system matrix F
  FMat F_;
  // system matrix G
  GMat G_;
  // whether system matrix inited
  bool system_mat_inited_ = false;
  // state covariance
  IMUPMat P_;
  // INS data
  INSData ins_data_;
  // do sculling compensation or not
  bool do_rotscull_corr_ = true;
};

}  // namespace localization
}  // namespace senseAD
