/******************************************************************************
 * Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 * Licensed Hozon
 ******************************************************************************/
#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// #include "data_types/common/types.h"
#include "depend/perception-base/base/location/location.h"

namespace hozon {
namespace mp {
namespace dr {

struct WheelOdometry {
  double x;  // meter
  double y;
  double z;

  double qw;
  double qx;
  double qy;
  double qz;

  WheelOdometry(double _x, double _y, double _z, double _qw, double _qx,
                double _qy, double _qz)
      : x(_x), y(_y), z(_z), qw(_qw), qx(_qx), qy(_qy), qz(_qz) {}
  WheelOdometry() : x(0), y(0), z(0), qw(1), qx(0), qy(0), qz(0) {}
};

struct OdometryData {
  double imu_stamp;
  double chassis_stamp;
  OdometryData() = default;

  double timestamp;
  int chassis_seq;
  int imu_seq;
  // double pub_time;
  // hozon::perception::base::Pose odometry;
  WheelOdometry odometry;
  Eigen::Vector3d loc_vel;
  Eigen::Vector3d loc_omg;
  Eigen::Vector3d loc_acc;
  int gear = 100;

  // ins data
  int gpsStatus = -1;        // 定位状态
  double latitude;           // 纬度 Unit: deg
  double longitude;          // 经度 Unit: deg
  double altitude;           // 海拔高度 Unit: meter
  Eigen::Vector3d attitude;  // 三轴姿态(roll, ptich, yaw), Unit: rad  不需要

  // imu sync_stamp
  double sync_stamp;
};

// hozon
struct WheelDataHozon {
  WheelDataHozon() = default;
  double timestamp;
  double recv_time;
  int seq;
  // 脉冲数
  double front_left_wheel;
  double front_right_wheel;
  double rear_left_wheel;
  double rear_right_wheel;
  // 车轮方向
  int front_left_dir;
  int front_right_dir;
  int rear_left_dir;
  int rear_right_dir;

  // 车轮速度
  double front_left_speed;
  double front_right_speed;
  double rear_left_speed;
  double rear_right_speed;
  int gear;
  // 方向盘信息
  double steer_info;
  // rolling counter
  int rolling_counter;
};

struct ImuDataHozon {
  double timestamp;
  double recv_time;
  int seq;
  Eigen::Vector3d acc_measurement;  // g
  Eigen::Vector3d gyr_measurement;  // Angular velocity in degree/s.

  // ins data
  int gpsStatus;             // 定位状态
  double latitude;           // 纬度 Unit: deg
  double longitude;          // 经度 Unit: deg
  double altitude;           // 海拔高度 Unit: meter
  Eigen::Vector3d attitude;  // 三轴姿态(roll, ptich, yaw), Unit: rad
  Eigen::Vector3d gyo_bias;
  Eigen::Vector3d acc_bias;
  Eigen::Vector3d mounting_error;
  Eigen::Vector3d sdPosition;  // 经纬高标准差 Unit: meter

  // sync stamp
  double sync_stamp;
};
using ImuDataConstPtr = std::shared_ptr<const ImuDataHozon>;
using ImuDataPtr = std::shared_ptr<ImuDataHozon>;

struct InsDataHozon {
  double timestamp;
  double gpsSec;             // GPS秒 pvatb 华测 && ifs  戴世
  double latitude;           // 纬度 Unit: deg
  double longitude;          // 经度 Unit: deg
  double altitude;           // 海拔高度 Unit: meter
  Eigen::Vector3d attitude;  // 三轴姿态(roll, ptich, yaw), Unit: rad  不需要
  Eigen::Vector3d linearVelocity;      // 导航系下的速度
  Eigen::Vector3d augularVelocity;     // 车身系下的角速度
  Eigen::Vector3d linearAcceleration;  // 车身系下的线加速度
  double heading;                      // ENU 坐标系, 偏北多少(顺时针)
  Eigen::Vector3d mountingError;       // 安装角度误差
  int sysStatus;                       // 组合状态
  int gpsStatus;                       // 定位状态
  Eigen::Vector3d rpy;
  Eigen::Vector3d lla;
};

template <size_t R, size_t C>
using MatRC_t = Eigen::Matrix<double, R, C>;
using Mat22_t = Eigen::Matrix2d;
using Mat33_t = Eigen::Matrix3d;
using Mat44_t = Eigen::Matrix4d;
using Mat55_t = MatRC_t<5, 5>;
using Mat66_t = MatRC_t<6, 6>;
using Mat77_t = MatRC_t<7, 7>;
using Mat34_t = MatRC_t<3, 4>;
using MatX_t = Eigen::MatrixXd;

// Eigen vector types

template <size_t R>
using VecR_t = Eigen::Matrix<double, R, 1>;
using Vec2_t = Eigen::Vector2d;
using Vec3_t = Eigen::Vector3d;
using Vec4_t = Eigen::Vector4d;
using Vec5_t = VecR_t<5>;
using Vec6_t = VecR_t<6>;
using Vec7_t = VecR_t<7>;
using VecX_t = Eigen::VectorXd;

}  // namespace dr
}  // namespace mp
}  // namespace hozon
