/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * Wang Xiaofeng<wangxiaofeng@sensetime.com>
 */

#include "imu/sins.hpp"

#include <algorithm>

#include "common/coordinate_converter.hpp"
#include "common/transform_config.hpp"
#include "common/utility.hpp"

namespace senseAD {
namespace localization {

double SINS::g_norm_;

SINS::SINS(const SINS& sins)
    : core_state_(sins.core_state_),
      // error_state_(sins.error_state_), // not used
      Rn_(sins.Rn_),
      Re_(sins.Re_),
      Wen_n_(sins.Wen_n_),
      Wie_n_(sins.Wie_n_),
      Win_n_(sins.Win_n_),
      g_n_(sins.g_n_),
      dt_(sins.dt_),
      last_time_(sins.last_time_),
      Tpr_(sins.Tpr_),
      Q_(sins.Q_),
      F_(sins.F_),
      G_(sins.G_),
      P_(sins.P_),
      ins_data_(sins.ins_data_),
      do_rotscull_corr_(sins.do_rotscull_corr_),
      system_mat_inited_(sins.system_mat_inited_) {
  g_norm_ = sins.g_norm_;
}

SINS& SINS::operator=(const SINS& sins) {
  this->core_state_ = sins.core_state_;
  // this->error_state_ = sins.error_state_; // not used
  this->Rn_ = sins.Rn_;
  this->Re_ = sins.Re_;
  this->Wen_n_ = sins.Wen_n_;
  this->Wie_n_ = sins.Wie_n_;
  this->Win_n_ = sins.Win_n_;
  this->g_n_ = sins.g_n_;
  this->dt_ = sins.dt_;
  this->last_time_ = sins.last_time_;
  this->Tpr_ = sins.Tpr_;
  this->Q_ = sins.Q_;
  this->F_ = sins.F_;
  this->G_ = sins.G_;
  this->P_ = sins.P_;
  this->ins_data_ = sins.ins_data_;
  this->do_rotscull_corr_ = sins.do_rotscull_corr_;
  this->system_mat_inited_ = sins.system_mat_inited_;
  this->g_norm_ = sins.g_norm_;
  return *this;
}

adLocStatus_t SINS::SetInitialState(double lat, double lon, double alt,
                                    const Eigen::Vector3d& ypr,
                                    const Vector3d& velocity,
                                    double gnss_bias) {
  // Set initial lla information
  Vector3d lla;
  lla << lat, lon, alt;
  // Set initial attitude information
  // Rotation of body w.r.t NED
  Matrix3d Cb_n;
  Quaterniond Qb_n;
  Cb_n = Utility::ypr2R(ypr);
  Qb_n = Cb_n;
  Qb_n.normalize();
  // default initialization with error
  // Position in lla
  core_state_.segment<3>(kiStatePosition) = lla;
  // Velocity in NED frame
  core_state_.segment<3>(kiStateVelocity) = velocity;
  // Attitude w.r.t NED frame
  core_state_.segment<4>(kiStateAttitude) = Utility::EigenQtoHamiltonVec(Qb_n);
  // acc bias
  core_state_.segment<3>(kiStateAccBias) = TransformConfig::GetAccelBias();
  // gyro bias
  core_state_.segment<3>(kiStateGyroBias) = TransformConfig::GetGyroBias();
  // acc scale
  core_state_.segment<3>(kiStateAccScale) = TransformConfig::GetAccelScale();
  // gyro scale
  core_state_.segment<3>(kiStateGyroScale) = TransformConfig::GetGyroScale();
  // gnss bias
  core_state_(kiStateGnssBias) = gnss_bias;
  // wheel scale
  core_state_(kiStateWheelScale) =
      1.0 - 1.0 / TransformConfig::GetOdomVelocityScale();

  // clear error state
  error_state_.setZero();
  // update LLA related parameters
  UpdateGeoParams();

  return LOC_SUCCESS;
}

adLocStatus_t SINS::PVAUpdate(uint64_t timestamp,
                              const IMUMeasurement& imu_reading,
                              bool with_cov_propagation) {
  // get the time gap
  if (last_time_ == 0) {
    dt_ = 0.01;
  } else {
    dt_ = timestamp * kNanoSecToSec - last_time_ * kNanoSecToSec;
  }
  assert(dt_ >= 0.0);
  if (dt_ == 0.0) {
    return LOC_SUCCESS;
  }
  last_time_ = timestamp;

  IMUMeasurement imu_reading_frd = TransferIMURfuToFrd(imu_reading);

  // Update navigation earth parameters
  UpdateGeoParams();

  // get acc and gyro bias
  Vector3d acc_b = GetAccBias();
  Vector3d gyro_b = GetGyroBias();
  // get acc and gyro scale
  Vector3d acc_s = GetAccScale();
  Vector3d gyro_s = GetGyroScale();
  // get imu measurements correction
  Vector3d acc_m = imu_reading_frd.segment<3>(0);
  Vector3d gyro_m = imu_reading_frd.segment<3>(3);
  ins_data_.fb0 = acc_m;
  ins_data_.omgb0 = gyro_m;
  // imu sensor error model correction
  Vector3d delta_f_b = acc_b + acc_m.asDiagonal() * acc_s;
  Vector3d delta_w_ib_b = gyro_b + gyro_m.asDiagonal() * gyro_s;
  ins_data_.fb = (acc_m - delta_f_b);
  ins_data_.omgb = (gyro_m - delta_w_ib_b);

  // rotational and sculling motion correction
  Vector3d dvs = Vector3d::Zero(), das = Vector3d::Zero();
  if (do_rotscull_corr_) {
    RotscullCorr(ins_data_, dt_, &dvs, &das);
  }
  ins_data_.fbp = ins_data_.fb;
  ins_data_.omgbp = ins_data_.omgb;
  do_rotscull_corr_ = true;

  // velocity increment (m/s)
  Vector3d dv = ins_data_.fb * dt_ + dvs;
  // angle increment (rad)
  Vector3d da = ins_data_.omgb * dt_ + das;

  Quaterniond Qb_n = GetQuat();
  Matrix3d Cb_n = Qb_n.toRotationMatrix();
  Vector3d lla = GetPosition();
  Vector3d Vn = GetVelocity();

  // Velocity Update
  // specific force integration
  Vector3d zeta = Win_n_ * dt_;
  Matrix3d Cn_n = Matrix3d::Identity() - SO3d::hat(zeta);
  Vector3d dv1 = Cn_n * Cb_n * dv;
  // Gravity and Coriolis acceleration integration
  Vector3d corAcc = (2 * Wie_n_ + Wen_n_).cross(Vn);
  Vector3d dv2 = (g_n_ - corAcc) * dt_;
  Vector3d Vn_curr = Vn + dv1 + dv2;
  Vector3d Vmid = 0.5 * (Vn + Vn_curr);
  ins_data_.an = (dv1 + dv2) / dt_;
  // using middle velocity update geo parameters
  SetVelocity(Vmid);
  UpdateGeoParams();
  zeta = Win_n_ * dt_;
  // Update Velocity
  SetVelocity(Vn_curr);

  // attitude Update
  Vector3d sigma = da;
  double sigma_norm = sigma.norm();
  double Qbb_r = cos(sigma_norm / 2);
  Vector3d Qbb_i = sin(sigma_norm / 2) / sigma_norm * sigma;
  Quaterniond Qbb(Qbb_r, Qbb_i(0), Qbb_i(1), Qbb_i(2));
  Qbb.normalize();
  Qb_n = Qb_n * Qbb;
  double zeta_norm = zeta.norm();
  double Qnn_r = cos(zeta_norm / 2);
  Vector3d Qnn_i = -sin(zeta_norm / 2) / zeta_norm * zeta;
  Quaterniond Qnn(Qnn_r, Qnn_i(0), Qnn_i(1), Qnn_i(2));
  Qnn.normalize();
  Qb_n = Qnn * Qb_n;
  Qb_n.normalize();
  SetQuat(Qb_n);

  // Position Update, latitude, longitude and altitude
  lla(0) += Vmid(0) / Rn_ * dt_;
  lla(1) += Vmid(1) / (Re_ * cos(lla(0))) * dt_;
  lla(2) += -Vmid(2) * dt_;
  SetPosition(lla);

  // error state covariance propagation
  if (with_cov_propagation) {
    // CovPropagation(dt_);
    CovPropagationSparse(dt_);
  }

  return LOC_SUCCESS;
}

adLocStatus_t SINS::CovPropagation(double dt) {
  // Get state
  Quaterniond Qb_n =
      Utility::HamiltonVecToEigenQ(core_state_.segment<4>(kiStateAttitude));
  Matrix3d Cb_n = Qb_n.toRotationMatrix();
  Vector3d lla = core_state_.segment<3>(kiStatePosition);
  Vector3d V_n = core_state_.segment<3>(kiStateVelocity);
  const Vector3d& fb = ins_data_.fb;
  const Vector3d& omgb = ins_data_.omgb;

  // Error state System model
  Vector3d f_n = Cb_n * fb;
  const double cosLat = cos(lla(0));
  const double sinLat = sin(lla(0));
  const double sinLat2 = sinLat * sinLat;
  const double tanLat = tan(lla(0));
  const double cosLat_inv = 1 / cosLat;
  const double cosLat_inv_2 = cosLat_inv * cosLat_inv;
  const double sr = std::sqrt(1.0 - WGS84::e2 * sinLat2);
  const double Rn = WGS84::R * (1 - WGS84::e2) / (sr * sr * sr) + lla(2);
  const double Re = WGS84::R / sr + lla(2);
  const double Vn = V_n(0);
  const double Ve = V_n(1);
  const double Vd = V_n(2);
  const double Wie = WGS84::Wie;
  const double Re_inv = 1 / Re;
  const double Rn_inv = 1 / Rn;
  const double Re_inv_2 = Re_inv * Re_inv;
  const double Rn_inv_2 = Rn_inv * Rn_inv;
  Vector3d Wie_n(Wie * cosLat, 0.0, -Wie * sinLat);
  Vector3d Wen_n(Ve * Re_inv, -Vn * Rn_inv, -Ve * tanLat * Re_inv);
  Vector3d Win_n = Wie_n + Wen_n;

  // Position, Velocity, Attitude, Acc Bias, Gyro Bias, Acc Scale, Gyro Scale
  Matrix3d Frr, Frv, Fre, Fvr, Fvv, Fve, Fer, Fev, Fee;
  Frr << 0, 0, -Vn * Rn_inv_2, Ve * sinLat * Re_inv * cosLat_inv_2, 0,
      -Ve * Re_inv_2 * cosLat_inv, 0, 0, 0;
  Frv << Rn_inv, 0, 0, 0, Re_inv * cosLat_inv, 0, 0, 0, -1;
  Fre = Matrix3d::Zero();

  Fvr << -Ve * (2 * Wie * cosLat + Ve * Re_inv * cosLat_inv_2), 0,
      Ve * Ve * tanLat * Re_inv_2 - Vn * Vd * Rn_inv_2,
      2 * Wie * (Vn * cosLat - Vd * sinLat) + Vn * Ve * Re_inv * cosLat_inv_2,
      0, -Ve * Re_inv_2 * (Vn * tanLat + Vd), 2 * Wie * Ve * sinLat, 0,
      Vn * Vn * Rn_inv_2 + Ve * Ve * Re_inv_2;
  Fvv << Vd * Rn_inv, -2 * (Wie * sinLat + Ve * Re_inv * tanLat), Vn * Rn_inv,
      2 * Wie * sinLat + Ve * Re_inv * tanLat, (Vd + Vn * tanLat) * Re_inv,
      2 * Wie * cosLat + Ve * Re_inv, -2 * Vn * Rn_inv,
      -2 * (Wie * cosLat + Ve * Re_inv), 0;
  Fve = SO3d::hat(f_n);

  Fer << -Wie * sinLat, 0, -Ve * Re_inv_2, 0, 0, Vn * Rn_inv_2,
      -Wie * cosLat - Ve * Re_inv * cosLat_inv_2, 0, Ve * tanLat * Re_inv_2;
  Fev << 0, Re_inv, 0, -Rn_inv, 0, 0, 0, -tanLat * Re_inv, 0;
  Fee << -SO3d::hat(Win_n);

  if (!system_mat_inited_) InitSystemMatrix();

  // update F Mat
  F_.block<3, 3>(kiErrorStatePosition, kiErrorStatePosition) = Frr;
  F_.block<3, 3>(kiErrorStatePosition, kiErrorStateVelocity) = Frv;
  F_.block<3, 3>(kiErrorStatePosition, kiErrorStateAttitude) = Fre;
  F_.block<3, 3>(kiErrorStateVelocity, kiErrorStatePosition) = Fvr;
  F_.block<3, 3>(kiErrorStateVelocity, kiErrorStateVelocity) = Fvv;
  F_.block<3, 3>(kiErrorStateVelocity, kiErrorStateAttitude) = Fve;
  F_.block<3, 3>(kiErrorStateVelocity, kiErrorStateAccBias) = Cb_n;
  F_.block<3, 3>(kiErrorStateVelocity, kiErrorStateAccScale) =
      Cb_n * (fb.asDiagonal());
  F_.block<3, 3>(kiErrorStateAttitude, kiErrorStatePosition) = Fer;
  F_.block<3, 3>(kiErrorStateAttitude, kiErrorStateVelocity) = Fev;
  F_.block<3, 3>(kiErrorStateAttitude, kiErrorStateAttitude) = Fee;
  F_.block<3, 3>(kiErrorStateAttitude, kiErrorStateGyroBias) = -Cb_n;
  F_.block<3, 3>(kiErrorStateAttitude, kiErrorStateGyroScale) =
      -Cb_n * (omgb.asDiagonal());

  // update G Mat
  G_.block<3, 3>(kiErrorStateVelocity, kiNoiseAccMea) = Cb_n;
  G_.block<3, 3>(kiErrorStateAttitude, kiNoiseGyroMea) = -Cb_n;

  // local used variables
  static FMat Fdt;
  static FMat phi;
  static GMat phi_G;
  static FMat Qk;
  Fdt.noalias() = F_ * dt;
  phi.noalias() = FMat::Identity() + Fdt + 0.5 * Fdt * Fdt;
  phi_G.noalias() = phi * G_;
  Qk.noalias() = phi_G * Q_ * phi_G.transpose() * dt;

  // predict the a priori state vector
  // NOTE: always zero, comment for speed up
  // error_state_ = phi * error_state_;

  // update the a priori covariance matrix
  P_ = phi * P_ * phi.transpose() + Qk;
  P_ = (P_ + P_.transpose()) / 2;

  // penalize covariance for long-term imu data loss, note that avoid penalize
  // bias and scale related part
  double penalized_factor = std::sqrt(std::max(1.0, dt / 0.04));
  P_.middleRows(kiErrorStatePosition, 9) *= penalized_factor;
  P_.middleCols(kiErrorStatePosition, 9) *= penalized_factor;

  return LOC_SUCCESS;
}

adLocStatus_t SINS::CovPropagationSparse(double dt) {
  // Get state
  Quaterniond Qb_n =
      Utility::HamiltonVecToEigenQ(core_state_.segment<4>(kiStateAttitude));
  Matrix3d Cb_n = Qb_n.toRotationMatrix();
  Vector3d lla = core_state_.segment<3>(kiStatePosition);
  Vector3d V_n = core_state_.segment<3>(kiStateVelocity);
  const Vector3d& fb = ins_data_.fb;
  const Vector3d& omgb = ins_data_.omgb;

  // Error state System model
  Vector3d f_n = Cb_n * fb;
  const double cosLat = cos(lla(0));
  const double sinLat = sin(lla(0));
  const double sinLat2 = sinLat * sinLat;
  const double tanLat = tan(lla(0));
  const double cosLat_inv = 1 / cosLat;
  const double cosLat_inv_2 = cosLat_inv * cosLat_inv;
  const double sr = std::sqrt(1.0 - WGS84::e2 * sinLat2);
  const double Rn = WGS84::R * (1 - WGS84::e2) / (sr * sr * sr) + lla(2);
  const double Re = WGS84::R / sr + lla(2);
  const double Vn = V_n(0);
  const double Ve = V_n(1);
  const double Vd = V_n(2);
  const double Wie = WGS84::Wie;
  const double Re_inv = 1 / Re;
  const double Rn_inv = 1 / Rn;
  const double Re_inv_2 = Re_inv * Re_inv;
  const double Rn_inv_2 = Rn_inv * Rn_inv;
  Vector3d Wie_n(Wie * cosLat, 0.0, -Wie * sinLat);
  Vector3d Wen_n(Ve * Re_inv, -Vn * Rn_inv, -Ve * tanLat * Re_inv);
  Vector3d Win_n = Wie_n + Wen_n;

  // Position, Velocity, Attitude, Acc Bias, Gyro Bias, Acc Scale, Gyro Scale
  Matrix3d Frr, Frv, Fre, Fvr, Fvv, Fve, Fer, Fev, Fee;
  Frr << 0, 0, -Vn * Rn_inv_2, Ve * sinLat * Re_inv * cosLat_inv_2, 0,
      -Ve * Re_inv_2 * cosLat_inv, 0, 0, 0;
  Frv << Rn_inv, 0, 0, 0, Re_inv * cosLat_inv, 0, 0, 0, -1;
  Fre = Matrix3d::Zero();

  Fvr << -Ve * (2 * Wie * cosLat + Ve * Re_inv * cosLat_inv_2), 0,
      Ve * Ve * tanLat * Re_inv_2 - Vn * Vd * Rn_inv_2,
      2 * Wie * (Vn * cosLat - Vd * sinLat) + Vn * Ve * Re_inv * cosLat_inv_2,
      0, -Ve * Re_inv_2 * (Vn * tanLat + Vd), 2 * Wie * Ve * sinLat, 0,
      Vn * Vn * Rn_inv_2 + Ve * Ve * Re_inv_2;
  Fvv << Vd * Rn_inv, -2 * (Wie * sinLat + Ve * Re_inv * tanLat), Vn * Rn_inv,
      2 * Wie * sinLat + Ve * Re_inv * tanLat, (Vd + Vn * tanLat) * Re_inv,
      2 * Wie * cosLat + Ve * Re_inv, -2 * Vn * Rn_inv,
      -2 * (Wie * cosLat + Ve * Re_inv), 0;
  Fve = SO3d::hat(f_n);

  Fer << -Wie * sinLat, 0, -Ve * Re_inv_2, 0, 0, Vn * Rn_inv_2,
      -Wie * cosLat - Ve * Re_inv * cosLat_inv_2, 0, Ve * tanLat * Re_inv_2;
  Fev << 0, Re_inv, 0, -Rn_inv, 0, 0, 0, -tanLat * Re_inv, 0;
  Fee << -SO3d::hat(Win_n);

  // NOTE: no need to change code here, if newly augmented states are modelled
  // as white noise process, and have no correlation with PVA

  // F, block wise calculation
  // F = [Fm | Fma          ]
  //     [0  | Fa (constant)]
  // Motion - P,V,A
  const int pva_dim = kiErrorStateAccBias;
  // Augmentation - Ba,Bg,Sa,Sg,GnssBias,WheelScale
  const int aug_dim = kiErrorStateSize - pva_dim;

  // init constant
  using MatM = Eigen::Matrix<double, pva_dim, pva_dim>;
  using MatA = Eigen::Matrix<double, aug_dim, aug_dim>;
  using MatMA = Eigen::Matrix<double, pva_dim, aug_dim>;
  using MatAM = Eigen::Matrix<double, aug_dim, pva_dim>;
  static constexpr double CORRETIME = 1.0e10;
  static constexpr double neg_inv_corrtime = -1.0 / CORRETIME;
  static bool const_inited = false;
  static MatM Im;
  static MatA Ia;
  static MatA Fa;
  static IMUPMat P_noise_base;
  if (!const_inited) {
    const_inited = true;
    Im.setIdentity();
    Ia.setIdentity();
    Fa.noalias() = neg_inv_corrtime * Ia;  // constant
    P_noise_base.setZero();
    P_noise_base.block<kiNoiseSize, kiNoiseSize>(kiErrorStateVelocity,
                                                 kiErrorStateVelocity) = Q_;
  }

  // Fm
  MatM Fm;
  Fm << Frr, Frv, Fre, Fvr, Fvv, Fve, Fer, Fev, Fee;
  // Fma
  MatMA Fma;
  Fma.setZero();
  int kiAugBase = kiErrorStateAccBias;
  Fma.block<3, 3>(kiErrorStateVelocity, kiErrorStateAccBias - kiAugBase) = Cb_n;
  Fma.block<3, 3>(kiErrorStateAttitude, kiErrorStateGyroBias - kiAugBase) =
      -Cb_n;
  Fma.block<3, 3>(kiErrorStateVelocity, kiErrorStateAccScale - kiAugBase)
      .noalias() = Cb_n * (fb.asDiagonal());
  Fma.block<3, 3>(kiErrorStateAttitude, kiErrorStateGyroScale - kiAugBase)
      .noalias() = -Cb_n * (omgb.asDiagonal());
  // Fm * Fma, need for phi
  MatMA FmFma;
  FmFma.setZero();
  Eigen::Matrix<double, pva_dim, 3> Fm_vel, Fm_att;
  Fm_vel << Frv, Fvv, Fev;
  Fm_att << Fre, Fve, Fee;
  FmFma.middleCols(kiErrorStateAccBias - kiAugBase, 3).noalias() =
      Fm_vel * Cb_n;
  FmFma.middleCols(kiErrorStateGyroBias - kiAugBase, 3).noalias() =
      -Fm_att * Cb_n;
  FmFma.middleCols(kiErrorStateAccScale - kiAugBase, 3).noalias() =
      FmFma.middleCols(kiErrorStateAccBias - kiAugBase, 3) * fb.asDiagonal();
  FmFma.middleCols(kiErrorStateGyroScale - kiAugBase, 3).noalias() =
      FmFma.middleCols(kiErrorStateGyroBias - kiAugBase, 3) * omgb.asDiagonal();
  // Fma * Fa, need for phi
  MatMA FmaFa;
  FmaFa.noalias() = Fma * neg_inv_corrtime;

  // phi, block wise calculation
  static FMat phi = FMat::Zero();
  double dt2 = dt * dt;
  phi.topLeftCorner(pva_dim, pva_dim).noalias() =
      Im + dt * Fm + 0.5 * dt2 * Fm * Fm;
  phi.topRightCorner(pva_dim, aug_dim).noalias() =
      dt * Fma + 0.5 * dt2 * (FmFma + FmaFa);
  double phi_a_scale = 1.0 + dt * neg_inv_corrtime +
                       0.5 * dt2 * neg_inv_corrtime * neg_inv_corrtime;
  phi.bottomRightCorner(aug_dim, aug_dim).noalias() = Ia * phi_a_scale;

  // P_noise, block wise calculation
  static IMUPMat P_noise;
  P_noise.noalias() = P_noise_base * dt;
  // transform imu acc and gyro noise from body to navigation frame
  // no need, as R * sigma * I * R.transpose() = sigma * I
  // Eigen::Matrix3d P_acc_noise =
  //     P_noise.block<3, 3>(kiErrorStateVelocity, kiErrorStateVelocity);
  // Eigen::Matrix3d P_gyro_noise =
  //     P_noise.block<3, 3>(kiErrorStateAttitude, kiErrorStateAttitude);
  // P_noise.block<3, 3>(kiErrorStateVelocity, kiErrorStateVelocity) =
  //     Cb_n * P_acc_noise * Cb_n.transpose();
  // P_noise.block<3, 3>(kiErrorStateAttitude, kiErrorStateAttitude) =
  //     Cb_n * P_gyro_noise * Cb_n.transpose();

  // update the a priori covariance matrix
  P_.noalias() = phi * (P_ + P_noise).eval() * phi.transpose();
  P_ = 0.5 * (P_ + P_.transpose());

  // penalize covariance for long-term imu data loss, note that avoid penalize
  // bias and scale related part
  double penalized_factor = std::sqrt(std::max(1.0, dt / 0.04));
  P_.middleRows(kiErrorStatePosition, 9) *= penalized_factor;
  P_.middleCols(kiErrorStatePosition, 9) *= penalized_factor;

  return LOC_SUCCESS;
}

adLocStatus_t SINS::StateCorrection(const IMUErrorState& X) {
  Vector3d lla = GetPosition();
  Quaterniond Qb_n = GetQuat();
  Matrix3d Cb_n = Qb_n.toRotationMatrix();
  Vector3d V_n = GetVelocity();
  Vector3d acc_b = GetAccBias();
  Vector3d gyro_b = GetGyroBias();
  Vector3d acc_s = GetAccScale();
  Vector3d gyro_s = GetGyroScale();
  Vector1d gnss_b = GetGnssBias();
  Vector1d wheel_s = GetWheelScale();

  // error state feed back
  // Rectify lla position
  lla -= X.segment<3>(kiErrorStatePosition);

  // Rectify velocity and NED Position
  V_n -= X.segment<3>(kiErrorStateVelocity);

  // Rectify rotation
  Vector3d epi = X.segment<3>(kiErrorStateAttitude);
  Cb_n = (Matrix3d::Identity() + SO3d::hat(epi)) * Cb_n;
  Qb_n = Cb_n;
  Qb_n.normalize();

  // bias
  Vector3d acc_bias = acc_b + X.segment<3>(kiErrorStateAccBias);
  Vector3d gyro_bias = gyro_b + X.segment<3>(kiErrorStateGyroBias);
  // scale
  Vector3d acc_scale = acc_s + X.segment<3>(kiErrorStateAccScale);
  Vector3d gyro_scale = gyro_s + X.segment<3>(kiErrorStateGyroScale);

  // gnss bias
  Vector1d gnss_bias = gnss_b - X.segment<1>(kiErrorStateGnssBias);

  // wheel scale
  Vector1d wheel_scale = wheel_s - X.segment<1>(kiErrorStateWheelScale);

  // core state update
  SetPosition(lla);
  SetVelocity(V_n);
  SetQuat(Qb_n);
  SetAccBias(acc_bias);
  SetAccScale(acc_scale);
  SetGyroBias(gyro_bias);
  SetGyroScale(gyro_scale);
  SetGnssBias(gnss_bias);
  SetWheelScale(wheel_scale);

  // Update navigation earth parameters
  UpdateGeoParams();

  return LOC_SUCCESS;
}

void SINS::UpdateGeoParams() {
  // Update Geographical parameter that's related to LLA
  Vector3d lla = GetPosition();
  double sinLat = sin(lla(0));
  double cosLat = cos(lla(0));
  double sinLat2 = sinLat * sinLat;
  double sr = std::sqrt(1.0 - WGS84::e2 * sinLat2);
  Rn_ = WGS84::R * (1 - WGS84::e2) / (sr * sr * sr) + lla(2);
  Re_ = WGS84::R / sr + lla(2);
  // earth rotation rate vector
  Wie_n_ = Vector3d(WGS84::Wie * cosLat, 0.0, -WGS84::Wie * sinLat);
  // navigation frame transport rate
  const Vector3d& Vn = GetVelocity();
  const double& vn = Vn(0);
  const double& ve = Vn(1);
  Wen_n_ = Vector3d(ve / Re_, -vn / Rn_, -ve * tan(lla(0)) / Re_);
  Win_n_ = Wie_n_ + Wen_n_;
  // update gravity accroding to WGS84  Ellipsoidal Gravity Formula
  g_norm_ = gravity_norm * (1 + 0.00193185265241 * sinLat2) /
            pow(1 - WGS84::e2 * sinLat2, 0.5);
  g_n_ = Vector3d(0, 0, g_norm_);
  // position from radians-to-meters (only used in residual calculation)
  Tpr_(0, 0) = Rn_;
  Tpr_(1, 1) = Re_ * cosLat;
  Tpr_(2, 2) = -1;
}

void SINS::RotscullCorr(const INSData& ins_data, double dt, Vector3d* dv,
                        Vector3d* da) {
  if (nullptr == dv || nullptr == da) {
    return;
  }
  Vector3d dap, dvp;
  dap = ins_data.omgbp * dt;
  dvp = ins_data.fbp * dt;

  Vector3d dak, dvk;
  dak = ins_data.omgb * dt;
  dvk = ins_data.fb * dt;

  Vector3d dv1, dv2, dv3;
  dv1 = SO3d::hat(dak) * dvk;
  dv2 = SO3d::hat(dap) * dvk;
  dv3 = SO3d::hat(dvp) * dak;

  double domg = dak.norm();
  double domg2 = domg * domg;
  double domg4 = domg2 * domg2;
  double a1, a2;
  if (std::fabs(domg) > 1.0e-6) {
    a1 = (1.0 - cos(domg)) / domg2;
    a2 = 1.0 / domg2 * (1.0 - sin(domg) / domg);
  } else {
    a1 = 0.5 - domg2 / 24.0 + domg4 / 720.0;
    a2 = 1.0 / 6.0 - domg2 / 120.0 + domg4 / 5040.0;
  }

  Vector3d dv4;
  dv4 = SO3d::hat(dak) * dv1;

  // velocity compensation
  (*dv) = a1 * dv1 + a2 * dv4 + 1.0 / 12.0 * (dv2 + dv3);
  // attitude compensation
  (*da) = SO3d::hat(dap) * dak * 1 / 12.0;
}

adLocStatus_t SINS::SetLastMeasurementTime(uint64_t t_ns) {
  last_time_ = t_ns;
  // do not do suclling compensation this time as last imu data is not right
  do_rotscull_corr_ = false;
  return LOC_SUCCESS;
}

void SINS::MakeErrorStateCov(double p_v, double v_v, double a_v, double ab_v,
                             double gb_v, double as_v, double gs_v) {
  P_.setZero();
  P_.block<3, 3>(kiErrorStatePosition, kiErrorStatePosition) = I33 * p_v;
  P_(2, 2) = 0.1 * 0.1;
  P_.block<3, 3>(kiErrorStateVelocity, kiErrorStateVelocity) = I33 * v_v;
  P_.block<3, 3>(kiErrorStateAttitude, kiErrorStateAttitude) = I33 * a_v;
  P_.block<3, 3>(kiErrorStateAccBias, kiErrorStateAccBias) = I33 * ab_v;
  P_.block<3, 3>(kiErrorStateGyroBias, kiErrorStateGyroBias) = I33 * gb_v;
  P_.block<3, 3>(kiErrorStateAccScale, kiErrorStateAccScale) = I33 * as_v;
  P_.block<3, 3>(kiErrorStateGyroScale, kiErrorStateGyroScale) = I33 * gs_v;
  P_(kiErrorStateGnssBias, kiErrorStateGnssBias) = 1.0 * 1.0;
  P_(kiErrorStateWheelScale, kiErrorStateWheelScale) = 0.1 * 0.1;
}

void SINS::MakeSystemNoiseCov(double am_n, double gm_n, double ab_n,
                              double gb_n, double as_n, double gs_n) {
  Q_.setZero();
  Q_.block<3, 3>(kiNoiseAccMea, kiNoiseAccMea) = I33 * am_n;
  Q_.block<3, 3>(kiNoiseGyroMea, kiNoiseGyroMea) = I33 * gm_n;
  Q_.block<3, 3>(kiNoiseAccBias, kiNoiseAccBias) = I33 * ab_n;
  Q_.block<3, 3>(kiNoiseGyroBias, kiNoiseGyroBias) = I33 * gb_n;
  Q_.block<3, 3>(kiNoiseAccScale, kiNoiseAccScale) = I33 * as_n;
  Q_.block<3, 3>(kiNoiseGyroScale, kiNoiseGyroScale) = I33 * gs_n;
  Q_(kiNoiseGnssBias, kiNoiseGnssBias) = 0.01 * 0.01;
  Q_(kiNoiseWheelScale, kiNoiseWheelScale) = 1.0e-5 * 1.0e-5;
}

void SINS::InitSystemMatrix() {
  // init constant part of system matrix F and G
  // model bias and scale stochastic process as random walk
  static constexpr double CORRETIME = 1.0e10;
  F_.setZero();
  F_.block<3, 3>(kiErrorStateAccBias, kiErrorStateAccBias) = -I33 / CORRETIME;
  F_.block<3, 3>(kiErrorStateGyroBias, kiErrorStateGyroBias) = -I33 / CORRETIME;
  F_.block<3, 3>(kiErrorStateAccScale, kiErrorStateAccScale) = -I33 / CORRETIME;
  F_.block<3, 3>(kiErrorStateGyroScale, kiErrorStateGyroScale) =
      -I33 / CORRETIME;
  F_(kiErrorStateGnssBias, kiErrorStateGnssBias) = -1.0 / CORRETIME;
  F_(kiErrorStateWheelScale, kiErrorStateWheelScale) = -1.0 / CORRETIME;

  G_.setZero();
  G_.block<3, 3>(kiErrorStateAccBias, kiNoiseAccBias) = I33;
  G_.block<3, 3>(kiErrorStateGyroBias, kiNoiseGyroBias) = I33;
  G_.block<3, 3>(kiErrorStateAccScale, kiNoiseAccScale) = I33;
  G_.block<3, 3>(kiErrorStateGyroScale, kiNoiseGyroScale) = I33;
  G_(kiErrorStateGnssBias, kiNoiseGnssBias) = 1.0;
  G_(kiErrorStateWheelScale, kiNoiseWheelScale) = 1.0;

  system_mat_inited_ = true;
}

adLocStatus_t SINS::SINSPAtoGlobalSE3(const Vector3d& sins_p,
                                      const Quaterniond& sins_a,
                                      SE3d* vehicle_pose) {
  PointENU_t enu;
  PointLLH_t lla;
  lla.lat = sins_p(0) * r2d;
  lla.lon = sins_p(1) * r2d;
  lla.height = sins_p(2);
  CoordinateConverter::GetInstance()->LLA2ENU(lla, &enu);
  Vector3d t_ned;
  t_ned << enu.y, enu.x, -enu.z;
  SE3d state_ned_frd = SE3d(sins_a, t_ned);
  Eigen::Vector3d ypr;
  ypr << M_PI_2, 0.0, M_PI;
  // SE3d Tfrd_rfu = SE3(SO3(ypr), Vector3d::Zero()); old Sophus
  SE3d Tfrd_rfu = SE3d(SO3d::exp(ypr), Vector3d::Zero());
  SE3d state_ned_rfu = state_ned_frd * Tfrd_rfu;
  // set vehicle frame centered
  SE3d state_ned_vehicle;
  TransformConfig::FromBodyToVehicle(state_ned_rfu, &state_ned_vehicle);
  // relative the HdMap ENU
  SE3d state_enu_vehicle;
  TransformConfig::FromRefNEDToENU(state_ned_vehicle, &state_enu_vehicle, true);
  state_enu_vehicle.normalize();
  *vehicle_pose = state_enu_vehicle;

  return LOC_SUCCESS;
}

adLocStatus_t SINS::GlobalSE3toSINSPA(const SE3d& vehicle_pose,
                                      Vector3d* sins_p, Quaterniond* sins_a) {
  // vehicle pose is vehicle frame relative to ENU frame
  SE3d state_ned_vehicle;
  TransformConfig::FromRefENUToNED(vehicle_pose, &state_ned_vehicle, true);
  // To body(Right-Forward-Up)
  SE3d state_ned_rfu;
  TransformConfig::FromVehicleToBody(state_ned_vehicle, &state_ned_rfu);
  // To body(Forward-Right-Down)
  Eigen::Vector3d ypr;
  ypr << M_PI_2, 0.0, M_PI;
  // SE3d Trfu_frd = SE3d(SO3d(ypr), Vector3d::Zero()); old Sophus
  SE3d Trfu_frd = SE3d(SO3d::exp(ypr), Vector3d::Zero());
  SE3d state_ned_frd = state_ned_rfu * Trfu_frd;
  state_ned_frd.normalize();
  (*sins_a) = state_ned_frd.so3().unit_quaternion();

  PointENU_t enu;
  enu.x = state_ned_frd.translation()(1);
  enu.y = state_ned_frd.translation()(0);
  enu.z = -state_ned_frd.translation()(2);
  PointLLH_t lla;
  CoordinateConverter::GetInstance()->ENU2LLA(enu, &lla);
  (*sins_p) << lla.lat * d2r, lla.lon * d2r, lla.height;

  return LOC_SUCCESS;
}

IMUMeasurement SINS::TransferIMURfuToFrd(const IMUMeasurement& imu_reading) {
  IMUMeasurement imu_reading_frd;
  imu_reading_frd(0) = imu_reading(1);
  imu_reading_frd(1) = imu_reading(0);
  imu_reading_frd(2) = -imu_reading(2);

  imu_reading_frd(3) = imu_reading(4);
  imu_reading_frd(4) = imu_reading(3);
  imu_reading_frd(5) = -imu_reading(5);
  return imu_reading_frd;
}

}  // namespace localization
}  // namespace senseAD
