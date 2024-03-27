/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： eskf.h
 *   author     ： zhangyu0435
 *   date       ： 2023.10
 ******************************************************************************/
#pragma once

#include <memory>
#include <string>
#include <Sophus/se3.hpp>
#include "modules/location/fusion_center/lib/defines.h"

namespace hozon {
namespace mp {
namespace loc {
namespace fc {

enum StateType { UNINIT = 0, INIT = 1, PREDICT = 2, UPDATE = 3 };

struct State {
  double ticktime = -1;
  StateType type = StateType::UNINIT;
  NodeType meas_type = NodeType::NONE;

  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Vector3d v = Eigen::Vector3d::Zero();
  Eigen::Vector3d ori = Eigen::Vector3d::Zero();  // 旋转向量（非欧拉角）
  Eigen::Vector3d b_g = Eigen::Vector3d::Zero();
  Eigen::Vector3d b_a = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q;  // 四元数
  Sophus::SO3<double> R;

  uint32_t sys_status = 0;
  uint32_t rtk_status = 0;
};

struct Options {
  /// 开关
  bool update_b_a;  // 是否更新加速度零偏
  bool update_b_g;  // 是否更新陀螺仪零偏

  /// 过程噪声Q
  double Q_velx_std = 1;  // ins的速度测量标准差
  double Q_vely_std = 1;
  double Q_velz_std = 1;
  double Q_orix_std = 0.1;  // ins的旋转角测量标准差
  double Q_oriy_std = 0.1;
  double Q_oriz_std = 0.1;
  double Q_bias_gyro_std = 1e-6;  // 陀螺仪零偏标准差
  double Q_bias_acce_std = 1e-4;  // 加速度计零偏标准差

  /// INS 观测参数1 (rtk=4)
  double R_ins_posx_std = 0.01;  // INS位置噪声
  double R_ins_posy_std = 0.01;
  double R_ins_posz_std = 0.01;
  double R_ins_orix_std = 0.01;  // INS旋转角噪声
  double R_ins_oriy_std = 0.01;
  double R_ins_oriz_std = 0.01;

  /// INS 观测参数2 (rtk=3)
  double ins_pos_noise2 = 0.1;     // GNSS位置噪声
  double ins_height_noise2 = 0.1;  // GNSS高度噪声
  double ins_ang_noise2 = 1.0;     // GNSS旋转噪声

  /// DR 观测参数 (待补充)
  /// MM 观测参数
  double R_mm_posx_std = 0.01;  // MM位置噪声
  double R_mm_posy_std = 0.01;
  double R_mm_posz_std = 0.01;
  double R_mm_orix_std = 0.01;  // MM旋转角噪声
  double R_mm_oriy_std = 0.01;
  double R_mm_oriz_std = 0.01;
};

class ESKF {
 public:
  ESKF() = default;
  ~ESKF() = default;

  bool Init(const std::string &configfile);
  bool Predict(const Node &cur_pre_data);
  void Correct(const Node &cur_meas_data);
  void StateInit(const std::shared_ptr<Node>& node);
  State GetState() { return X_; }
  Eigen::Matrix<double, 3, 3> JlSO3(const Eigen::Matrix<double, 3, 1>& w);
  Eigen::Matrix<double, 3, 3> JrSO3(const Eigen::Matrix<double, 3, 1>& w);
  Eigen::Matrix<double, 3, 3> SkewMatrix(Eigen::Vector3d v);

 private:
  void UpdateAndReset();

 private:
  /// P = Fx*P*Fx + Fi*Qi*Fi
  using Mat15T = Eigen::Matrix<double, 15, 15>;
  using Mat6T = Eigen::Matrix<double, 6, 6>;
  using Mat3T = Eigen::Matrix<double, 3, 3>;
  using Vec6d = Eigen::Matrix<double, 6, 1>;
  using SO3 = Sophus::SO3d;
  Mat15T P_;
  Mat15T Fx_;
  Mat15T Q_;  // Q = Fi*Qi*Fi
  /// K = P*H'*inv(H*P*H'+R)
  Eigen::Matrix<double, 15, 6> K_;
  Eigen::Matrix<double, 6, 15> H_;
  Mat6T R_ins_;
  Mat6T R_ins2_;
  Mat6T R_mm_;
  // Mat6T R_dr_ = R_dr_.setZero();
  /// X_dx = K*(y-h(X))
  Eigen::Matrix<double, 15, 1> X_dx_;
  /// X = X + X_dx
  State X_;
  /// 噪声与零偏
  Options options_;
};

}  // namespace fc
}  // namespace loc
}  // namespace mp
}  // namespace hozon
