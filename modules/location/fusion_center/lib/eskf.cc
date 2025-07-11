/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： eskf.cc
 *   author     ： zhangyu0435
 *   date       ： 2023.10
 ******************************************************************************/
#include "modules/location/fusion_center/lib/eskf.h"
#include <yaml-cpp/yaml.h>

#include <boost/filesystem.hpp>

#include "modules/util/include/util/mapping_log.h"

namespace hozon {
namespace mp {
namespace loc {
namespace fc {

bool ESKF::Init(const std::string& configfile) {
  boost::filesystem::path path(configfile);
  if (!boost::filesystem::exists(path)) {
    HLOG_ERROR << "eskf configfile error:" << configfile << " not exist";
    return false;
  }
  YAML::Node config = YAML::LoadFile(configfile);
  // 参数读取
  options_.Q_velx_std = config["Q_velx_std"].as<double>();
  options_.Q_vely_std = config["Q_vely_std"].as<double>();
  options_.Q_velz_std = config["Q_velz_std"].as<double>();
  options_.Q_orix_std = config["Q_orix_std"].as<double>();
  options_.Q_oriy_std = config["Q_oriy_std"].as<double>();
  options_.Q_oriz_std = config["Q_oriz_std"].as<double>();
  options_.Q_bias_gyro_std = config["Q_bias_gyro_std"].as<double>();
  options_.Q_bias_acce_std = config["Q_bias_acce_std"].as<double>();
  options_.R_ins_posx_std = config["R_ins_posx_std"].as<double>();
  options_.R_ins_posy_std = config["R_ins_posy_std"].as<double>();
  options_.R_ins_posz_std = config["R_ins_posz_std"].as<double>();
  options_.R_ins_orix_std = config["R_ins_orix_std"].as<double>();
  options_.R_ins_oriy_std = config["R_ins_oriy_std"].as<double>();
  options_.R_ins_oriz_std = config["R_ins_oriz_std"].as<double>();
  options_.ins_pos_noise2 = config["ins_pos_noise2"].as<double>();
  options_.ins_ang_noise2 = config["ins_ang_noise2"].as<double>();
  options_.R_mm_posx_std = config["R_mm_posx_std"].as<double>();
  options_.R_mm_posy_std = config["R_mm_posy_std"].as<double>();
  options_.R_mm_posz_std = config["R_mm_posz_std"].as<double>();
  options_.R_mm_orix_std = config["R_mm_orix_std"].as<double>();
  options_.R_mm_oriy_std = config["R_mm_oriy_std"].as<double>();
  options_.R_mm_oriz_std = config["R_mm_oriz_std"].as<double>();
  options_.update_b_a = config["update_b_a"].as<bool>();
  options_.update_b_g = config["update_b_g"].as<bool>();

  // H,Q,R初始化
  H_.setZero();
  H_.template block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();  // p部分
  H_.template block<3, 3>(3, 6) = Eigen::Matrix3d::Identity();  // q部分

  Q_.setZero();
  Q_(3, 3) = std::pow(options_.Q_velx_std, 2);
  Q_(4, 4) = std::pow(options_.Q_vely_std, 2);
  Q_(5, 5) = std::pow(options_.Q_velz_std, 2);
  Q_(6, 6) = std::pow(options_.Q_orix_std, 2);
  Q_(7, 7) = std::pow(options_.Q_oriy_std, 2);
  Q_(8, 8) = std::pow(options_.Q_oriz_std, 2);
  Q_.template block<3, 3>(9, 9) =
      Eigen::Matrix3d::Identity() * std::pow(options_.Q_bias_gyro_std, 2);
  Q_.template block<3, 3>(12, 12) =
      Eigen::Matrix3d::Identity() * std::pow(options_.Q_bias_acce_std, 2);

  R_ins_.setZero();
  R_ins_(0, 0) = std::pow(options_.R_ins_posx_std, 2);
  R_ins_(1, 1) = std::pow(options_.R_ins_posy_std, 2);
  R_ins_(2, 2) = std::pow(options_.R_ins_posz_std, 2);
  R_ins_(3, 3) = std::pow(options_.R_ins_orix_std, 2);
  R_ins_(4, 4) = std::pow(options_.R_ins_oriy_std, 2);
  R_ins_(5, 5) = std::pow(options_.R_ins_oriz_std, 2);

  R_ins2_.setZero();
  R_ins2_.template block<3, 3>(0, 0) =
      Eigen::Matrix3d::Identity() * options_.ins_pos_noise2;
  R_ins2_.template block<3, 3>(3, 3) =
      Eigen::Matrix3d::Identity() * options_.ins_ang_noise2;

  R_mm_.setZero();
  R_mm_(0, 0) = std::pow(options_.R_mm_posx_std, 2);
  R_mm_(1, 1) = std::pow(options_.R_mm_posy_std, 2);
  R_mm_(2, 2) = std::pow(options_.R_mm_posz_std, 2);
  R_mm_(3, 3) = std::pow(options_.R_mm_orix_std, 2);
  R_mm_(4, 4) = std::pow(options_.R_mm_oriy_std, 2);
  R_mm_(5, 5) = std::pow(options_.R_mm_oriz_std, 2);

  // 变量初始化
  X_dx_.setZero();
  Fx_.setZero();
  K_.setZero();
  P_ = Mat15T::Identity();
  HLOG_INFO << "ESKF Yaml Init Success!";

  return true;
}

void ESKF::StateInit(const std::shared_ptr<Node>& node) {
  X_.ticktime = node->ticktime;
  X_.type = StateType::INIT;
  X_.meas_type = NodeType::NONE;
  X_.p = node->enu;
  X_.v = node->velocity;
  X_.ori = node->orientation;
  X_.b_a = node->b_a;
  X_.b_g = node->b_g;
  X_.q = node->quaternion;
  X_.R = SO3::exp(node->orientation);
  X_.sys_status = node->sys_status;
  X_.rtk_status = node->rtk_status;
}

bool ESKF::Predict(const Node& cur_pre_data) {
  double dt = cur_pre_data.ticktime - X_.ticktime;
  // 1.判断时间间隔
  if (dt < 0 || dt > 0.1) {
    HLOG_ERROR << "INS时间戳的时间间隔不对,delta_t = " << dt;
    return false;
  }

  X_.b_a = cur_pre_data.b_a * 9.8 / 1000;
  X_.b_g = cur_pre_data.b_g / 57.3;
  X_.b_a = Eigen::Vector3d::Zero();
  X_.b_g = Eigen::Vector3d::Zero();

  // 2.predict nominal state
  const auto cur_acc = cur_pre_data.linear_accel * 9.8;
  const auto cur_ang_v = cur_pre_data.angular_velocity / 57.3;

  X_.type = StateType::PREDICT;
  X_.meas_type = NodeType::NONE;
  X_.p = X_.p + X_.v * dt + 0.5 * (X_.R * (cur_acc - X_.b_a)) * dt * dt;
  X_.v = X_.v + (X_.R * (cur_acc - X_.b_a)) * dt;
  X_.R = X_.R * SO3::exp((cur_ang_v - X_.b_g) * dt);
  X_.ori = X_.R.log();

  // 3.计算F(err_state)
  Fx_ = Mat15T::Identity();
  Fx_.template block<3, 3>(0, 3) = Mat3T::Identity() * dt;
  Fx_.template block<3, 3>(3, 6) =
      -X_.R.matrix() * SO3::hat(cur_acc - X_.b_a) * dt;
  // Fx_.template block<3, 3>(3, 12) = -X_.R.matrix() * dt;
  Fx_.template block<3, 3>(6, 6) =
      SO3::exp(-(cur_ang_v - X_.b_g) * dt).matrix();
  // Fx_.template block<3, 3>(6, 9) = -Mat3T::Identity() * dt;

  // 4.predict cov (err_state, dX_ = 0)  P = Fx*P*Fx + Fi*Qi*Fi
  P_ = Fx_ * P_.eval() * Fx_.transpose() + Q_;

  // 5.其他赋值
  X_.sys_status = cur_pre_data.sys_status;
  X_.rtk_status = cur_pre_data.rtk_status;
  X_.ticktime = cur_pre_data.ticktime;

  HLOG_DEBUG << "eskf predict............";

  // LOC_INFO << "X_.p" << X_.p;
  // LOC_INFO << "X_.v" << X_.v;
  // LOC_INFO << "X_.R" << X_.R.matrix();
  // LOC_INFO << "X_.p" << X_.p;
  // LOC_INFO << "X_.v" << X_.v;
  // LOC_INFO << "Fx_" << Fx_;
  // LOC_INFO << "P_" << P_;
  return true;
}

void ESKF::Correct(const Node& cur_meas_data) {
  // 1.通过测量传感器来源选择H,R,MeasType
  X_.type = StateType::UPDATE;
  X_.meas_type = cur_meas_data.type;
  Mat6T R;
  R.setZero();
  switch (cur_meas_data.type) {
    case NodeType::INS:
      R = R_ins_;
      break;
    default:
      double pe_cov_coef = 1.0;
      if (cur_meas_data.pe_cov_coef > 1.0) {
        pe_cov_coef = cur_meas_data.pe_cov_coef;
      }
      if (cur_meas_data.pe_cov_coef > 100.0) {
        pe_cov_coef = 100.0;
      }
      R = pe_cov_coef * R_mm_;
      // roll，pitch初始就设置比较大，因此不调整roll，pitch
      R(3, 3) = R_mm_(3, 3);
      R(4, 4) = R_mm_(4, 4);
      break;
  }

  // 2.计算H
  H_.template block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();  // p部分
  // Mat3T yacobi_rot = JrSO3(cur_meas_data.orientation);
  // H_.template block<3, 3>(3, 6) = yacobi_rot.inverse();  // q部分
  H_.template block<3, 3>(3, 6) = Eigen::Matrix3d::Identity();  // q部分
  // 3.计算y_diff(观测-nominal)(INS需要更新速度、位姿，MM更新位姿即可)(c此处需要进行if判断)
  Vec6d y_diff;
  y_diff.setZero();
  SO3 meas_rot = SO3::exp(cur_meas_data.orientation);
  SO3 X_rot = SO3::exp(X_.ori);
  y_diff.template block<3, 1>(0, 0) = cur_meas_data.enu - X_.p;
  y_diff.template block<3, 1>(3, 0) = ((X_rot.inverse() * meas_rot)).log();
  // std::cout << "cur_meas_data.enu:" << cur_meas_data.enu << std::endl;
  // std::cout << "X_.p:" << X_.p << std::endl;
  // std::cout << "y_diff:" << y_diff << std::endl;
  // 速度观测待更新---------------------

  // 4.update dx、conv（err_state）K =
  // P*H'*inv(H*P*H'+R)-----后期要根据不同测量源选择R矩阵
  K_ = P_ * H_.transpose() * ((H_ * P_ * H_.transpose() + R).inverse());
  X_dx_ = K_ * y_diff;
  P_ = (Mat15T::Identity() - K_ * H_) * P_;
  static int count = 0;
  ++count;
  if (count >= 100) {
    HLOG_INFO << "meas_ticktime: " << cur_meas_data.ticktime
              << ",meas_type: " << X_.meas_type << ",ydiff_pos1:" << y_diff(0)
              << ",ydiff_pos2:" << y_diff(1) << ",ydiff_pos3:" << y_diff(2)
              << ",ydiff_ori1:" << y_diff(3) << ",ydiff_ori2:" << y_diff(4)
              << ",ydiff_ori3:" << y_diff(5) << ",Kydiff_pos1:" << X_dx_(0)
              << ",Kydiff_pos2:" << X_dx_(1) << ",Kydiff_pos3:" << X_dx_(2)
              << ",Kydiff_ori1:" << X_dx_(6) << ",Kydiff_ori2:" << X_dx_(7)
              << ",Kydiff_ori3:" << X_dx_(8) << ",P_pos_1:" << P_(0, 0)
              << ",P_pos_2:" << P_(1, 1) << ",P_pos_3:" << P_(2, 2)
              << ",P_angle_1:" << P_(6, 6) << ",P_angle_1:" << P_(7, 7)
              << ",P_angle_1:" << P_(8, 8);
    count = 0;
  }

  // 5.更新名义状态，并重置X_dx_
  UpdateAndReset();
  HLOG_DEBUG << "eskf update............X_.meas_type: " << X_.meas_type;
}

void ESKF::UpdateAndReset() {
  // 1.真实状态 = 名义状态 + 误差状态
  X_.p += X_dx_.template block<3, 1>(0, 0);
  X_.v += X_dx_.template block<3, 1>(3, 0);
  X_.R = X_.R * SO3::exp(X_dx_.template block<3, 1>(6, 0));
  X_.ori = X_.R.log();
  if (options_.update_b_g) {
    X_.b_g += X_dx_.template block<3, 1>(9, 0);
  }
  if (options_.update_b_a) {
    X_.b_a += X_dx_.template block<3, 1>(12, 0);
  }
  // std::cout << "X_.p2:" << X_.p << std::endl;
  // 2.重置误差状态
  Mat15T J = Mat15T::Identity();
  J.template block<3, 3>(6, 6) =
      Mat3T::Identity() - 0.5 * SO3::hat(X_dx_.template block<3, 1>(6, 0));
  P_ = J * P_ * J.transpose();
  X_dx_.setZero();
  X_.cov = P_;
}

Eigen::Matrix<double, 3, 3> ESKF::JlSO3(const Eigen::Matrix<double, 3, 1>& w) {
  double theta = w.norm();
  if (theta < 1e-6) {
    return Eigen::MatrixXd::Identity(3, 3);
  } else {
    Eigen::Matrix<double, 3, 1> a = w / theta;
    Eigen::Matrix<double, 3, 3> J =
        sin(theta) / theta * Eigen::MatrixXd::Identity(3, 3) +
        (1 - sin(theta) / theta) * a * a.transpose() +
        ((1 - cos(theta)) / theta) * SkewMatrix(a);
    return J;
  }
}

Eigen::Matrix<double, 3, 3> ESKF::JrSO3(const Eigen::Matrix<double, 3, 1>& w) {
  return JlSO3(-w);
}

Eigen::Matrix<double, 3, 3> ESKF::SkewMatrix(Eigen::Vector3d v) {
  Eigen::Matrix<double, 3, 3> m;
  m << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  return m;
}

}  // namespace fc
}  // namespace loc
}  // namespace mp
}  // namespace hozon
