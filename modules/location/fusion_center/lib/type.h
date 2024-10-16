/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： type.h
 *   author     ： zhaohaowu
 *   date       ： 2024.09
 ******************************************************************************/

#include <Eigen/Dense>
#include <memory>

#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Geometry/Quaternion.h"
namespace hozon {
namespace mp {
namespace loc {
namespace fc {
#define DEFINE_PTR(Type) typedef std::shared_ptr<Type> Ptr;
#define DEFINE_CONST_PTR(Type) typedef std::shared_ptr<const Type> ConstPtr;

struct FcBase {
  double timestamp = 0.0;
};

enum MeasureType { UNKNOWN = 0, CHASSIS = 1, INS = 2, MM = 3, STATIC = 4 };

struct Measure : public FcBase {
  MeasureType measure_type = UNKNOWN;
  Eigen::Vector3d gcj_position;
  Eigen::Vector3d enu_position;
  Eigen::Quaterniond enu_q;
  Eigen::Vector3d linear_velocity_vrf;
  int rtk_state = -1;
  double second = -1;
  unsigned int week = -1;
  unsigned int warn_info = -1;
  Eigen::Vector3d sd_position;
  double ins_heading = -1;
  DEFINE_PTR(Measure);
  DEFINE_CONST_PTR(Measure);
};

struct Predict : public FcBase {
  Eigen::Vector3d angular_velocity_vrf;
  Eigen::Vector3d linear_acceleration_vrf;
  DEFINE_PTR(Predict);
  DEFINE_CONST_PTR(Predict);
};

struct Dr : FcBase {
  Eigen::Vector3d dr_position;
  Eigen::Quaterniond dr_q;
  DEFINE_PTR(Dr);
  DEFINE_CONST_PTR(Dr);
};

struct FcState : FcBase {
  Eigen::Vector3d gcj_position;
  // 状态
  Eigen::Vector3d p;
  Eigen::Vector3d v;
  Eigen::Quaterniond q;
  // 误差状态
  Eigen::Matrix<double, 9, 1> dx;
  Eigen::Matrix<double, 9, 9> P;
  Eigen::Matrix<double, 9, 9> Q;
  Eigen::Matrix<double, 6, 6> R_mm;
  Eigen::Matrix<double, 6, 6> R_ins;
  Eigen::Matrix<double, 3, 3> R_chassis;

  // 定位状态质量
  int state = -1;
  int rtk_state = -1;
  double second = -1;
  unsigned int week = -1;
  double ins_sd_position = -1;  // 标准差
  double ins_height = -1;
  double ins_heading = -1;

  // dr信息
  Eigen::Vector3d p_dr;
  Eigen::Quaterniond q_dr;
  double pitch_dr;
  double yaw_dr;

  // 线速度
  Eigen::Vector3d linear_velocity_vrf;

  // 角速度
  Eigen::Vector3d angular_velocity_vrf;

  // 线加速度
  Eigen::Vector3d linear_acceleration_vrf;

  // 可视化ins偏差估计
  Eigen::Vector3d p_ins_estimate;
  Eigen::Quaterniond q_ins_estimate;

  DEFINE_PTR(FcState);
  DEFINE_CONST_PTR(FcState);
};

struct Option {
  // imu预测
  double imu_noise_v_x = 1e-12;
  double imu_noise_v_y = 1e-12;
  double imu_noise_v_z = 1e-12;
  double imu_noise_ori_x = 1e-12;
  double imu_noise_ori_y = 1e-12;
  double imu_noise_ori_z = 1e-12;
  // mm观测
  bool use_mm = false;
  double mm_noise_x = 1e-10;
  double mm_noise_y = 1e-10;
  double mm_noise_z = 1e-10;
  double mm_noise_ori_x = 1e-10;
  double mm_noise_ori_y = 1e-10;
  double mm_noise_ori_z = 1e-10;
  // ins观测
  bool use_ins = false;
  double ins_noise_x = 1e-10;
  double ins_noise_y = 1e-10;
  double ins_noise_z = 1e-10;
  double ins_noise_ori_x = 1e-10;
  double ins_noise_ori_y = 1e-10;
  double ins_noise_ori_z = 1e-10;
  // chassis观测
  bool use_chassis = false;
  double chassis_noise_v_x = 1e-10;
  double chassis_noise_v_y = 1e-10;
  double chassis_noise_v_z = 1e-10;
};

struct Offset {
  Eigen::Matrix4d T_offset;
  bool init = true;
  Eigen::Vector3d pos;
  double time;
};

}  // namespace fc
}  // namespace loc
}  // namespace mp
}  // namespace hozon
