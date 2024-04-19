/*
 * Copyright (C) 2019 by SenseTime Group Limited. All rights reserved.
 * Wang Xiaofeng<wangxiaofeng@sensetime.com>
 */

#pragma once

#include <Eigen/Core>

namespace senseAD {
namespace localization {

static constexpr double kNanoSecToSec = 1.0e-9;

static Eigen::Matrix3d I33 = Eigen::Matrix3d::Identity();
static Eigen::Matrix3d O33 = Eigen::Matrix3d::Zero();
static constexpr double r2d = 180 / M_PI;
static constexpr double d2r = M_PI / 180;

static constexpr double gravity_norm = 9.7803253359;

/////////////////////////////// CAN related ////////////////////////////////////

static constexpr int kcReadingSize = 6;

// define nominal state id (P, Q, Vs, Wb)
static constexpr int kcStatePosition = 0;
static constexpr int kcStateQuat = kcStatePosition + 3;
static constexpr int kcStateVs = kcStateQuat + 4;
static constexpr int kcStateWb = kcStateVs + 3;
static constexpr int kcStateSize = kcStateWb + 3;

// define error state id (dP, dQ, dVs, dWb)
static constexpr int kcErrorStatePosition = 0;
static constexpr int kcErrorStateQuat = kcErrorStatePosition + 3;
static constexpr int kcErrorStateVs = kcErrorStateQuat + 3;
static constexpr int kcErrorStateWb = kcErrorStateVs + 3;
static constexpr int kcErrorStateSize = kcErrorStateWb + 3;

// define noise id (Vmea_n, Wmea_n, Vs_n, Wb_n)
static constexpr int kcNoiseVelMea = 0;
static constexpr int kcNoiseOmegaMea = kcNoiseVelMea + 3;
static constexpr int kcNoiseVelScale = kcNoiseOmegaMea + 3;
static constexpr int kcNoiseOmegaBias = kcNoiseVelScale + 3;
static constexpr int kcNoiseSize = kcNoiseOmegaBias + 3;

using CANCoreState = Eigen::Matrix<double, kcStateSize, 1>;
using CANMeasurement = Eigen::Matrix<double, kcReadingSize, 1>;
using CANErrorState = Eigen::Matrix<double, kcErrorStateSize, 1>;
using CANPMat = Eigen::Matrix<double, kcErrorStateSize, kcErrorStateSize>;
using CANQMat = Eigen::Matrix<double, kcNoiseSize, kcNoiseSize>;

/////////////////////////////// IMU related ////////////////////////////////////

static constexpr int kiReadingSize = 6;

// define nominal state id
// P, V, A, AccBias, GyroBias, AccScale, GyroScale, GnssBias(lateral, m)
static constexpr int kiStatePosition = 0;
static constexpr int kiStateVelocity = kiStatePosition + 3;
static constexpr int kiStateAttitude = kiStateVelocity + 3;
static constexpr int kiStateAccBias = kiStateAttitude + 4;
static constexpr int kiStateGyroBias = kiStateAccBias + 3;
static constexpr int kiStateAccScale = kiStateGyroBias + 3;
static constexpr int kiStateGyroScale = kiStateAccScale + 3;
static constexpr int kiStateGnssBias = kiStateGyroScale + 3;
static constexpr int kiStateWheelScale = kiStateGnssBias + 1;
static constexpr int kiStateSize = kiStateWheelScale + 1;

// define error state id
// dP, dV, dA, dAccBias, dGyroBias, dAccScale, dGyroScale, dGnssBias(lateral, m)
static constexpr int kiErrorStatePosition = 0;
static constexpr int kiErrorStateVelocity = kiErrorStatePosition + 3;
static constexpr int kiErrorStateAttitude = kiErrorStateVelocity + 3;
static constexpr int kiErrorStateAccBias = kiErrorStateAttitude + 3;
static constexpr int kiErrorStateGyroBias = kiErrorStateAccBias + 3;
static constexpr int kiErrorStateAccScale = kiErrorStateGyroBias + 3;
static constexpr int kiErrorStateGyroScale = kiErrorStateAccScale + 3;
static constexpr int kiErrorStateGnssBias = kiErrorStateGyroScale + 3;
static constexpr int kiErrorStateWheelScale = kiErrorStateGnssBias + 1;
static constexpr int kiErrorStateSize = kiErrorStateWheelScale + 1;

// define noise id
// AccMea_n, GyroMea_n, AccBias_n, GyroBias_n, AccScale_n, GyroScale_n,
// GnssBias_n(lateral, m)
static constexpr int kiNoiseAccMea = 0;
static constexpr int kiNoiseGyroMea = kiNoiseAccMea + 3;
static constexpr int kiNoiseAccBias = kiNoiseGyroMea + 3;
static constexpr int kiNoiseGyroBias = kiNoiseAccBias + 3;
static constexpr int kiNoiseAccScale = kiNoiseGyroBias + 3;
static constexpr int kiNoiseGyroScale = kiNoiseAccScale + 3;
static constexpr int kiNoiseGnssBias = kiNoiseGyroScale + 3;
static constexpr int kiNoiseWheelScale = kiNoiseGnssBias + 1;
static constexpr int kiNoiseSize = kiNoiseWheelScale + 1;

using IMUCoreState = Eigen::Matrix<double, kiStateSize, 1>;
using IMUMeasurement = Eigen::Matrix<double, kiReadingSize, 1>;
using IMUErrorState = Eigen::Matrix<double, kiErrorStateSize, 1>;
using IMUPMat = Eigen::Matrix<double, kiErrorStateSize, kiErrorStateSize>;
using IMUQMat = Eigen::Matrix<double, kiNoiseSize, kiNoiseSize>;

//////////////////////////////// DR related ////////////////////////////////////

// define nominal state id for DR, (P, V, A, AccBias, GyroBias)
static constexpr int kiStatePositionDR = 0;
static constexpr int kiStateVelocityDR = kiStatePositionDR + 3;
static constexpr int kiStateAttitudeDR = kiStateVelocityDR + 3;
static constexpr int kiStateAccBiasDR = kiStateAttitudeDR + 4;
static constexpr int kiStateGyroBiasDR = kiStateAccBiasDR + 3;
static constexpr int kiStateSizeDR = kiStateGyroBiasDR + 3;

// define error state id for DR, (dP, dV, dA, dAccBias, dGyroBias)
static constexpr int kiErrorStatePositionDR = 0;
static constexpr int kiErrorStateVelocityDR = kiErrorStatePositionDR + 3;
static constexpr int kiErrorStateAttitudeDR = kiErrorStateVelocityDR + 3;
static constexpr int kiErrorStateAccBiasDR = kiErrorStateAttitudeDR + 3;
static constexpr int kiErrorStateGyroBiasDR = kiErrorStateAccBiasDR + 3;
static constexpr int kiErrorStateSizeDR = kiErrorStateGyroBiasDR + 3;

// define noise id for DR, (AccMea_n, GyroMea_n, AccBias_n, GyroBias_n)
static constexpr int kiNoiseAccMeaDR = 0;
static constexpr int kiNoiseGyroMeaDR = kiNoiseAccMeaDR + 3;
static constexpr int kiNoiseAccBiasDR = kiNoiseGyroMeaDR + 3;
static constexpr int kiNoiseGyroBiasDR = kiNoiseAccBiasDR + 3;
static constexpr int kiNoiseSizeDR = kiNoiseGyroBiasDR + 3;

typedef Eigen::Matrix<double, kiStateSizeDR, 1> IMUCoreStateDR;
typedef Eigen::Matrix<double, kiErrorStateSizeDR, 1> IMUErrorStateDR;
typedef Eigen::Matrix<double, kiErrorStateSizeDR, kiErrorStateSizeDR> IMUPMatDR;
typedef Eigen::Matrix<double, kiNoiseSizeDR, kiNoiseSizeDR> IMUQMatDR;

}  // namespace localization
}  // namespace senseAD
