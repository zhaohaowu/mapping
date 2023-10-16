/******************************************************************************
 * Copyright (C) 2022 HOZON-AUTO Ltd. All rights reserved.
 * Licensed Hozon
 * Author: pengwei
 ******************************************************************************/
#include "odometry2D.h"

#include <algorithm>
#include <iomanip>
#include <tuple>

namespace hozon {
namespace mp {
namespace dr {

bool Odometry2D::update() {
  static Eigen::Vector3d last_local_vel = {0.0, 0.0, 0.0};
  if (!initialized_) {
    if (CanInitialize()) {
      return Initialize();
    } else {
      return false;
    }
  }
  int update_cnt = 0;
  while (true) {
    std::vector<WheelDataHozon> oldest_wheels = get_oldest_two_wheel();
    if (oldest_wheels.size() < 2) {
      // HLOG_DEBUG << "==== init ==== no new wheel data";
      break;
    }
    std::vector<ImuDataHozon> imu_datas;
    get_imu_before_and_pop(oldest_wheels[1].timestamp + 5e-3, imu_datas);
    if (imu_datas.empty()) {
      // HLOG_INFO << "==== init ==== no time_matched imu data";
      break;
    }
    update_cnt++;
    // 里程计更新位置 (xyz)
    Eigen::Vector3d local_vel(0, 0, 0);
    double delta_dis = 0;
    std::tie(local_vel, delta_dis) =
        UpdatePosByWheel(oldest_wheels[0], oldest_wheels[1]);

    double wheel_time_dt =
        oldest_wheels[1].timestamp - oldest_wheels[0].timestamp;
    double wheel_time = oldest_wheels[1].timestamp;
    //  std::cout << "wheel time:" << wheel_time << std::endl;

    Eigen::Vector3d pos_pre = pos_;
    Eigen::Quaterniond qat_pre = qat_;
    unsigned int imu_cnt = 0;
    auto itr = imu_datas.begin() + 1;
    for (; itr != imu_datas.end(); itr++) {
      // imu数据更新姿态
      auto itr_pre = std::prev(itr);

      if (is_car_standstill_) {
        car_standstill_counter_ += 1.0;
        imu_acc_buffer_.push_back(itr->acc_measurement);
        car_standstill_omg_sum_ += itr->gyr_measurement;
        // 更新静态零偏
        if (car_standstill_counter_ >= 50) {  // FLAGS_car_standstill_counter
          EstimatedRollPitch();
          imu_acc_buffer_.clear();
          Eigen::Vector3d gyro_tmp =
              car_standstill_omg_sum_ / car_standstill_counter_;
          gyro_bias_ = gyro_bias_ * 0.8 + gyro_tmp * 0.2;
          constexpr double max_gyro_bias = 0.005;
          gyro_bias_[2] =
              std::min(std::max(gyro_bias_[2], -max_gyro_bias), max_gyro_bias);
          car_standstill_counter_ = 0.0;
          car_standstill_omg_sum_.setZero();
        }
      } else {
        car_standstill_counter_ = 0.0;
        car_standstill_omg_sum_.setZero();
        imu_acc_buffer_.clear();
      }

      double dt = itr->timestamp - itr_pre->timestamp;
      imu_cnt++;
      UpdateOrientationByIMU(*itr_pre, *itr);

      // HLOG_INFO << "==== init ==== qat ===update !! " << qat_.x() << " "
      //           << qat_.y() << "  " << qat_.z() << " " << qat_.w();

      // state prediction
      Eigen::Vector3d dx = Eigen::Vector3d::Zero();
      Eigen::Vector3d acc_measurement = {itr->acc_measurement[0],
                                         itr->acc_measurement[1], 0};
      dx = qat_.toRotationMatrix() * (acc_measurement - acc_bias_) + gravity_;
      vel_ += (dx * dt);

      // 计算加速度
      // acc_by_gyro_ = 0.5 * (itr_pre->acc_measurement + itr->acc_measurement);
      // acc_by_gyro_ = acc_by_gyro_ + qat_3D_.conjugate().matrix() * gravity_;

      if (wheel_vel_buffer_.size() > 1) {
        double cur_acc = (wheel_vel_buffer_.back().second -
                          (wheel_vel_buffer_.end() - 2)->second) /
                         (wheel_vel_buffer_.back().first -
                          (wheel_vel_buffer_.end() - 2)->first);
        if (itr->acc_measurement(0) >= 0) {
          acc_by_wheel_ =
              std::abs(cur_acc);  // 0.6 * cur_acc + 0.4 * acc_by_wheel_;
        } else {
          acc_by_wheel_ = -std::abs(cur_acc);
        }
      }
      acc_by_gyro_ << acc_by_wheel_, 0, 0;

      // state covariance prediction
      Eigen::MatrixXd prc_mat =
          Eigen::MatrixXd::Zero(error_state_.rows(), error_state_.rows());
      const double sgm_vel_prc = 0.3;
      const double sgm_acc_prc = 0.01;
      const double sgm_att_prc = 0.2 * DEG_2_RAD;
      const double sgm_gyro_bias_prc = 0.01 * DEG_2_RAD;
      prc_mat(0, 0) = dt * pow(sgm_vel_prc, 2.0);
      prc_mat(1, 1) = dt * pow(sgm_vel_prc, 2.0);
      prc_mat(3, 3) = dt * pow(sgm_acc_prc, 2.0);
      prc_mat(4, 4) = dt * pow(sgm_acc_prc, 2.0);
      prc_mat(8, 8) = dt * pow(sgm_att_prc, 2.0);
      prc_mat(11, 11) = dt * pow(sgm_gyro_bias_prc, 2.0);

      // v_x,v_y,v_z,ba_x,ba_y,ba_z,roll,pitch,yaw,bw_x,bw_y,bw_z
      Eigen::MatrixXd rot_mat = qat_.toRotationMatrix();
      Eigen::MatrixXd sys_mat =
          Eigen::MatrixXd::Zero(error_state_.rows(), error_state_.rows());
      sys_mat.block<2, 2>(0, 0) = Eigen::Matrix2d::Identity();
      // sys_mat.block<3,3>(0,3) = - rot_mat;
      sys_mat.block<2, 1>(0, 3) = -rot_mat.block<2, 1>(0, 0) * dt;
      sys_mat.block<2, 1>(0, 4) = -rot_mat.block<2, 1>(0, 1) * dt;
      sys_mat.block<2, 1>(0, 8) =
          -(rot_mat * get_cross_mat(acc_measurement - acc_bias_))
               .block<2, 1>(0, 2) *
          dt;
      sys_mat(8, 11) = -dt;

      error_state_cov_ =
          sys_mat * error_state_cov_ * sys_mat.transpose() + prc_mat;
    }
    Eigen::Quaterniond qat_veh_pdt = extrinsic_Q_ * qat_;
    Eigen::Vector3d eulerAngle_pdt = qat_veh_pdt.matrix().eulerAngles(2, 1, 0);

    // update pos
    Eigen::Vector3d delta_t_veh(delta_dis, 0, 0);
    Eigen::Vector3d delta_t_imu =
        extrinsic_Q_.conjugate().toRotationMatrix() *
            (delta_t_veh - extrinsic_T_) +
        (qat_pre.conjugate() * qat_ * extrinsic_Q_.conjugate())
                .toRotationMatrix() *
            extrinsic_T_;
    pos_ += qat_pre * delta_t_imu;

    Eigen::Vector3d local_vel_imu = delta_t_imu * (1.0 / wheel_time_dt);

    // state update
    // local_car_speed_correct(local_vel_imu);

    static unsigned int counter = 0;
    if (++counter % 100 == 0) {
      // local_yaw_cov_update();
    }

    // vel_ = qat_.toRotationMatrix() * local_vel;
    // 删除历史数据
    pop_front_wheel();

    OdometryData cur_odom_data;
    // transe to the wheel coordinate
    Eigen::Quaterniond qat_veh = qat_ * extrinsic_Q_.conjugate();
    Eigen::Vector3d pos_veh = pos_ - qat_veh * extrinsic_T_;
    Eigen::Vector3d vel_veh = extrinsic_Q_.conjugate() * vel_;

    cur_odom_data.timestamp = wheel_time;

    // cur_odom_data.odometry.position << pos_[0], pos_[1], pos_[2];

    // cur_odom_data.odometry.quaternion.w = qat_.w();
    // cur_odom_data.odometry.quaternion.x = qat_.x();
    // cur_odom_data.odometry.quaternion.y = qat_.y();
    // cur_odom_data.odometry.quaternion.z = qat_.z();

    cur_odom_data.odometry =
        WheelOdometry(pos_veh[0], pos_veh[1], pos_veh[2], qat_veh.w(),
                      qat_veh.x(), qat_veh.y(), qat_veh.z());

    cur_odom_data.loc_vel = local_vel;
    cur_odom_data.loc_omg = w_by_gyro_;
    cur_odom_data.loc_acc = acc_by_gyro_;
    cur_odom_data.gear = oldest_wheels[1].gear;

    // 把ins 数据加入(只加最新的)
    auto imu_itr = imu_datas.end() - 1;
    cur_odom_data.altitude = imu_itr->altitude;
    cur_odom_data.longitude = imu_itr->longitude;
    cur_odom_data.attitude = imu_itr->attitude;
    cur_odom_data.gpsStatus = imu_itr->gpsStatus;

    AddOdomData(cur_odom_data, delta_dis);
    last_local_vel = local_vel;

    // HLOG_INFO << "==== init ===="
    //           << " local_vel; " << local_vel(0) << " acc:" << acc_by_gyro_(0);
  }
  return update_cnt > 0 ? true : false;
}

bool Odometry2D::Initialize() {
  const double max_gyro_bias = 0.2 * DEG_2_RAD;
  const double vel_sgm = 0.3;
  const double acc_sgm = 0.03;
  const double att_sgm = 0.5 * DEG_2_RAD;
  // const double gyro_bias_sgm = 0.5 * DEG_2_RAD;
  const double gyro_bias_sgm = 0.005;
  const double yaw_sgm = 0.5 * DEG_2_RAD;

  error_state_ = Eigen::VectorXd::Zero(STATE_NUMBER);
  error_state_cov_ = Eigen::MatrixXd::Zero(STATE_NUMBER, STATE_NUMBER);

  error_state_cov_(0, 0) = pow(vel_sgm, 2.0);
  error_state_cov_(1, 1) = pow(vel_sgm, 2.0);
  error_state_cov_(3, 3) = pow(acc_sgm, 2.0);
  error_state_cov_(4, 4) = pow(acc_sgm, 2.0);
  error_state_cov_(8, 8) = pow(yaw_sgm, 2.0);
  error_state_cov_(11, 11) = pow(gyro_bias_sgm, 2.0);

  Eigen::Vector3d accl_sum = {0, 0, 0};
  Eigen::Vector3d gyro_sum = {0, 0, 0};
  Eigen::Vector3d gyro_mean = {0, 0, 0};
  Eigen::Vector3d accl_mean = {0, 0, 0};

  pos_ = Eigen::Vector3d::Zero() + extrinsic_T_;
  qat_ = extrinsic_Q_;
  vel_.setZero();
  acc_bias_.setZero();
  gyro_bias_.setZero();
  qat_3D_ = Eigen::Quaterniond(1.0, 0., 0., 0.);  // w, x, y, z
  imu_acc_buffer_.reserve(50);
  initialized_ = true;

  clear_imu_wheel_datas();

  // HLOG_INFO << "wheel data size:" << get_wheel_data_size()
  //           << " imu data size:" << get_imu_data_size();
  OdometryData cur_odom_data;
  cur_odom_data.timestamp = get_front_wheel().timestamp;
  cur_odom_data.odometry = WheelOdometry(0, 0, 0, 1, 0, 0, 0);
  // cur_odom_data.odometry.position << 0,0,0;

  // cur_odom_data.odometry.quaternion.w = 1;
  // cur_odom_data.odometry.quaternion.x = 0;
  // cur_odom_data.odometry.quaternion.y = 0;
  // cur_odom_data.odometry.quaternion.z = 0;

  cur_odom_data.loc_vel = {0, 0, 0};
  cur_odom_data.loc_omg = {0, 0, 0};
  cur_odom_data.loc_acc = {0, 0, 0};
  cur_odom_data.gear = 0;
  AddOdomData(cur_odom_data, 0.0);
  // HLOG_INFO << "DR initialize done";
  std::cout << "DR initialize done" << std::endl;
  return true;
}

std::tuple<Eigen::Vector3d, double> Odometry2D::UpdatePosByWheel(
    const WheelDataHozon& last, const WheelDataHozon& cur) {
  is_car_standstill_ = false;

  double begin_wl_1 = last.front_left_wheel;
  double begin_wr_1 = last.front_right_wheel;
  double end_wl_1 = last.rear_left_wheel;
  double end_wr_1 = last.rear_right_wheel;

  double begin_wl_2 = cur.front_left_wheel;
  double begin_wr_2 = cur.front_right_wheel;
  double end_wl_2 = cur.rear_left_wheel;
  double end_wr_2 = cur.rear_right_wheel;

  double end_wl_1_tmp = end_wl_1;
  double end_wl_2_tmp = end_wl_2;

  double end_wr_1_tmp = end_wr_1;
  double end_wr_2_tmp = end_wr_2;

  double left_diff = end_wl_2_tmp - end_wl_1_tmp;
  if (left_diff < 0) left_diff += MAX_WHEEL_COUNT;
  double right_diff = end_wr_2_tmp - end_wr_1_tmp;
  if (right_diff < 0) right_diff += MAX_WHEEL_COUNT;

  double left_dist = left_diff * wheel_param_.kl_;
  if (cur.rear_left_dir == 1) {
    left_dist *= -1.0;
    //    std::cout << "left back direction" << std::endl;
  }
  double right_dist = right_diff * wheel_param_.kr_;
  if (cur.rear_right_dir == 1) {
    right_dist *= -1.0;
    // std::cout << "right back direction" << std::endl;
  }
  double delta_yaw = (right_dist - left_dist) / wheel_param_.b_;
  double delta_dist = (right_dist + left_dist) * 0.5;

  // double wheel_speed = delta_dist / (cur.timestamp - last.timestamp);
  // 只求取了 X 方向得速度信息
  double wheel_speed = (cur.rear_left_speed + cur.rear_right_speed) / 2.0 / 3.6;
  is_car_standstill_ = fabs(wheel_speed) < 1.0e-6;
  if (wheel_vel_buffer_.size() > 10) {
    wheel_vel_buffer_.pop_front();
  }
  wheel_vel_buffer_.push_back(
      std::pair<double, double>(cur.timestamp, wheel_speed));
  Eigen::Vector3d vel(wheel_speed, 0, 0);
  return std::make_tuple(vel, delta_dist);
}

void Odometry2D::UpdateOrientationByIMU(const ImuDataHozon& last_imu,
                                        const ImuDataHozon& cur_imu) {
  double dt = cur_imu.timestamp - last_imu.timestamp;
  // update qat_
  Eigen::Vector3d last_gyr_measurement = {0, 0, last_imu.gyr_measurement[2]};
  Eigen::Vector3d cur_gyr_measurement = {0, 0, cur_imu.gyr_measurement[2]};
  Eigen::Vector3d gyro_bias_yaw = {0, 0, gyro_bias_[2]};
  Eigen::Vector3d gyro_unbias =
      0.5 * (last_gyr_measurement + cur_gyr_measurement) - gyro_bias_yaw;
  Eigen::Vector3d det_ang = gyro_unbias * dt;

  if (gyro_bias_.norm() > 1e-2) {
    w_by_gyro_ = 0.5 * (last_gyr_measurement + cur_gyr_measurement);
  } else {
    w_by_gyro_ = gyro_unbias;
  }

  if (det_ang.norm() > 1e-12) {
    qat_ = qat_ * Eigen::Quaterniond(
                      Eigen::AngleAxisd(det_ang.norm(), det_ang.normalized()));
    qat_ = qat_.normalized();
  }

  // update qat_3D_
  gyro_unbias =
      0.5 * (last_imu.gyr_measurement + cur_imu.gyr_measurement) - gyro_bias_;
  det_ang = gyro_unbias * dt;
  if (det_ang.norm() > 1e-12) {
    qat_3D_ = qat_3D_ * Eigen::Quaterniond(Eigen::AngleAxisd(
                            det_ang.norm(), det_ang.normalized()));
    qat_3D_ = qat_3D_.normalized();
  }
}

void Odometry2D::local_yaw_cov_update() {
  // const double sgm_yaw = 3.0 * DEG_2_RAD;
  // Eigen::MatrixXd r_mat = Eigen::MatrixXd::Zero(1, 1);
  // r_mat(0, 0) = pow(sgm_yaw, 2.0);

  // Eigen::MatrixXd trans_mat = qat_.toRotationMatrix();
  // Eigen::VectorXd res_error = Eigen::VectorXd::Zero(1);
  // Eigen::MatrixXd h_mat = Eigen::MatrixXd::Zero(1, error_state_.rows());
  // h_mat(0, 8) = 1.0;

  // // yaw_cov_corrector_.adaptive_correct(error_state_, error_state_cov_,
  // res_error,
  // //                                     h_mat, r_mat);
  // local_state_update();
}

void Odometry2D::local_state_update() {
  const double max_gyro_bias = 0.2 * DEG_2_RAD;
  Eigen::VectorXd ru = error_state_.segment(6, 3);
  Eigen::Quaterniond quat_error(1.0, ru(0), ru(1), ru(2));
  quat_error.normalize();
  vel_ += error_state_.segment(0, 3);
  acc_bias_ += error_state_.segment(3, 3);
  qat_ = qat_ * quat_error;
  gyro_bias_ += error_state_.segment(9, 3);
  Eigen::Vector3d delta_gyro_bias = error_state_.segment(9, 3);

  for (unsigned int i = 0; i != 3; ++i) {
    gyro_bias_(i) =
        std::min(std::max(gyro_bias_(i), -max_gyro_bias), max_gyro_bias);
  }
  error_state_.block<3, 1>(0, 0) = Eigen::VectorXd::Zero(3, 1);
  error_state_.block<3, 1>(3, 0) = Eigen::VectorXd::Zero(3, 1);
  error_state_.block<3, 1>(6, 0) = Eigen::VectorXd::Zero(3, 1);
  error_state_.block<3, 1>(9, 0) = Eigen::VectorXd::Zero(3, 1);
}

bool Odometry2D::EstimatedRollPitch() {
  // Compute mean and std of the imu buffer.
  if (static_cast<int>(imu_acc_buffer_.size()) != 50) {
    // std::cout << "imu acc data size error: " << imu_acc_buffer_.size() <<
    // std::endl;
    return false;
  }

  Eigen::Vector3d sum_acc(0., 0., 0.);
  for (const auto acc : imu_acc_buffer_) {
    sum_acc += acc;
  }

  const Eigen::Vector3d mean_acc =
      sum_acc / static_cast<double>(imu_acc_buffer_.size());

  Eigen::Vector3d sum_err2(0., 0., 0.);
  for (const auto acc : imu_acc_buffer_) {
    sum_err2 += (acc - mean_acc).cwiseAbs2();
  }
  const Eigen::Vector3d std_acc =
      (sum_err2 / static_cast<double>(imu_acc_buffer_.size())).cwiseSqrt();

  if (std_acc.maxCoeff() > 3.0) {
    // std::cout << "Estimated roll pitch too big acc std: " <<
    // std_acc.transpose() << std::endl;
    return false;
  }

  if ((mean_acc.norm() - CON_g0) > 0.5) {
    // std::cout << "veh not static, mean_acc_norm: " << mean_acc.norm()
    //           << ", mean_acc: " << mean_acc.transpose() << std::endl;
    return false;
  }

  // Compute rotation. refer to
  // https://github.com/rpng/open_vins/blob/master/ov_core/src/init/InertialInitializer.cpp
  // Three axises of the ENU frame in the IMU frame.
  // z-axis.
  const Eigen::Vector3d& z_axis = mean_acc.normalized();

  // x-axis.
  Eigen::Vector3d x_axis =
      Eigen::Vector3d::UnitX() -
      z_axis * z_axis.transpose() * Eigen::Vector3d::UnitX();
  x_axis.normalize();

  // y-axis.
  Eigen::Vector3d y_axis = z_axis.cross(x_axis);
  y_axis.normalize();

  Eigen::Matrix3d R_i_w;
  R_i_w.block<3, 1>(0, 0) = x_axis;
  R_i_w.block<3, 1>(0, 1) = y_axis;
  R_i_w.block<3, 1>(0, 2) = z_axis;

  Eigen::Quaterniond qat_tmp = Eigen::Quaterniond(R_i_w.transpose());
  double roll, pitch, yaw;
  Qat2EulerAngle(qat_tmp, roll, pitch, yaw);

  double roll_pre, pitch_pre, yaw_pre;
  Qat2EulerAngle(qat_, roll_pre, pitch_pre, yaw_pre);

  Eigen::AngleAxisd roll_aa(roll * M_PI / 180.0, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch_aa(pitch * M_PI / 180.0, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw_aa(yaw_pre * M_PI / 180.0, Eigen::Vector3d::UnitZ());
  qat_3D_ = yaw_aa * pitch_aa * roll_aa;  // 完成转换
  return true;
}

}  // namespace dr
}  // namespace mp
}  // namespace hozon
