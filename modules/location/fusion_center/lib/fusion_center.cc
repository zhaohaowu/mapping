/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： fusion_center.cc
 *   author     ： zhaohaowu
 *   date       ： 2024.09
 ******************************************************************************/

#include "modules/location/fusion_center/lib/fusion_center.h"
#include <math.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <memory>
#include <vector>

#include <boost/filesystem.hpp>

#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Geometry/Quaternion.h"
#include "Eigen/src/Geometry/Transform.h"
#include "Sophus/so3.hpp"
#include "depend/perception-base/base/utils/log.h"
#include "depend/proto/dead_reckoning/dr.pb.h"
#include "depend/proto/localization/node_info.pb.h"
#include "modules/location/fusion_center/lib/data_buffer.h"
#include "modules/util/include/util/geo.h"
#include "modules/util/include/util/tic_toc.h"
#include "yaml-cpp/node/node.h"
#include "yaml-cpp/node/parse.h"

namespace hozon {
namespace mp {
namespace loc {
namespace fc {

FusionCenter::~FusionCenter() {
  fc_thread_run_.store(false);
  {
    std::unique_lock<std::mutex> lock(imu_cv_mutex_);
    imu_cv_.notify_one();
  }
  if (fc_thread_->joinable()) {
    fc_thread_->join();
  }
}

bool FusionCenter::Init(const std::string& config_file) {
  YAML::Node node = YAML::LoadFile(config_file);
  option_.imu_noise_v_x = node["imu_noise_v_x"].as<double>();
  option_.imu_noise_v_y = node["imu_noise_v_y"].as<double>();
  option_.imu_noise_v_z = node["imu_noise_v_z"].as<double>();
  option_.imu_noise_ori_x = node["imu_noise_ori_x"].as<double>();
  option_.imu_noise_ori_y = node["imu_noise_ori_y"].as<double>();
  option_.imu_noise_ori_z = node["imu_noise_ori_z"].as<double>();

  option_.use_mm = node["use_mm"].as<bool>();
  option_.mm_noise_x = node["mm_noise_x"].as<double>();
  option_.mm_noise_y = node["mm_noise_y"].as<double>();
  option_.mm_noise_z = node["mm_noise_z"].as<double>();
  option_.mm_noise_ori_x = node["mm_noise_ori_x"].as<double>();
  option_.mm_noise_ori_y = node["mm_noise_ori_y"].as<double>();
  option_.mm_noise_ori_z = node["mm_noise_ori_z"].as<double>();

  option_.use_ins = node["use_ins"].as<bool>();
  option_.ins_noise_x = node["ins_noise_x"].as<double>();
  option_.ins_noise_y = node["ins_noise_y"].as<double>();
  option_.ins_noise_z = node["ins_noise_z"].as<double>();
  option_.ins_noise_ori_x = node["ins_noise_ori_x"].as<double>();
  option_.ins_noise_ori_y = node["ins_noise_ori_y"].as<double>();
  option_.ins_noise_ori_z = node["ins_noise_ori_z"].as<double>();

  option_.use_chassis = node["use_chassis"].as<bool>();
  option_.chassis_noise_v_x = node["chassis_noise_v_x"].as<double>();
  option_.chassis_noise_v_y = node["chassis_noise_v_y"].as<double>();
  option_.chassis_noise_v_z = node["chassis_noise_v_z"].as<double>();

  imu_buffer_ = std::make_shared<MessageBuffer<Predict::ConstPtr>>(300);
  chassis_buffer_ = std::make_shared<MessageBuffer<Measure::ConstPtr>>(300);
  ins_buffer_ = std::make_shared<MessageBuffer<Measure::ConstPtr>>(30);
  dr_buffer_ = std::make_shared<MessageBuffer<Dr::ConstPtr>>(300);
  mm_buffer_ = std::make_shared<MessageBuffer<Measure::ConstPtr>>(30);

  offset_.init = true;

  fc_thread_run_.store(true);
  fc_thread_ = std::make_shared<std::thread>(&FusionCenter::Run, this);
  return true;
}

void FusionCenter::Run() {
  // 设置fc线程名字
  pthread_setname_np(pthread_self(), "fc_thread");
  while (fc_thread_run_) {
    // imu回调函数触发
    {
      std::unique_lock<std::mutex> lock(imu_cv_mutex_);
      imu_cv_.wait(lock);
    }
    // HLOG_FATAL << "-------------fc start----------------";
    util::TicToc tic;

    // step1: 先验条件检查:
    // 1. 清空imu_vector和measure_vector
    // 2. ref_point初始化检查，获取refpoint_
    // 3. imu_buffer判空检查，获取latest_imu
    // 4. ins_buffer判空检查，获取latest_ins
    // 5. chassis_buffer判空检查，获取latest_chassis
    if (!PreConditionCheck()) {
      HLOG_WARN << "step1: pre condition check failed";
      continue;
    }

    // step2: Fc第一帧状态初始化
    // 同步chassis和ins数据对第一帧fc_state初始化
    if (!StateInit()) {
      HLOG_WARN << "step2: fc first frame state init failed";
      continue;
    }

    // step3: 回退至最早的Fc
    // 找到所有更新的观测中最老的观测时间之前的fc_state为first_fc
    // 获取first_fc到latest_imu时间之间的imu数据为imu_vector
    // 获取first_fc到latest_imu时间之间的mm数据、ins数据和chassis数据为measure_vector
    if (!GetFirstFc()) {
      HLOG_WARN << "step3: get first fc failed";
      continue;
    }

    // step4: 回退法eskf
    BackEskf();
    // step5: 最新状态赋予状态等信息
    UpdateLatestFc();
    // HLOG_FATAL << "fc_run_time " << tic.Toc();
  }
}

void FusionCenter::Clear() {
  imu_vector_.clear();
  measure_vector_.clear();
  fc_map_mutex_.lock();
  while (fc_map_.size() > 300) {
    fc_map_.erase(fc_map_.begin());
  }
  fc_map_mutex_.unlock();
}

bool FusionCenter::PreConditionCheck() {
  Clear();
  if (imu_buffer_->is_empty()) {
    HLOG_WARN << "imu_buffer_->is_empty()";
    return false;
  }
  latest_imu_ = *(imu_buffer_->back());
  if (ins_buffer_->is_empty()) {
    HLOG_WARN << "ins_buffer_->is_empty()";
    return false;
  }
  latest_ins_ = *(ins_buffer_->back());
  if (chassis_buffer_->is_empty()) {
    HLOG_WARN << "chassis_buffer_->is_empty()";
    return false;
  }
  latest_chassis_ = *(chassis_buffer_->back());
  if (!ref_point.init()) {
    HLOG_WARN << "fc_ref_point_ is not inited";
    return false;
  }
  fc_ref_point_ = ref_point.GetRefPoint();
  return true;
}

bool FusionCenter::StateInit() {
  static bool first_frame = true;
  if (!first_frame) {
    return true;
  }
  Measure::ConstPtr cur_chassis = GetChassisByTimestamp(latest_imu_.timestamp);
  Measure::ConstPtr cur_ins = GetInsByTimestamp(latest_imu_.timestamp);
  if (cur_chassis == nullptr || cur_ins == nullptr) {
    HLOG_WARN << "why can not get chassis or ins by imu timestamp";
    return false;
  }
  first_frame = false;
  // 初始化第一帧Fc状态
  FcState fc_state;
  fc_state.timestamp = cur_chassis->timestamp;
  fc_state.p = util::Geo::Gcj02ToEnu(cur_ins->gcj_position, fc_ref_point_);
  fc_state.gcj_position = cur_ins->gcj_position;
  fc_state.v = cur_ins->enu_q * cur_chassis->linear_velocity_vrf;
  fc_state.q = cur_ins->enu_q;
  fc_state.q.normalize();
  fc_state.dx = Eigen::Matrix<double, 9, 1>::Zero();
  fc_state.P = Eigen::Matrix<double, 9, 9>::Identity();
  fc_state.P.block<3, 3>(0, 0) =
      1.0e-1 * Eigen::Matrix<double, 3, 3>::Identity();
  fc_state.P.block<3, 3>(3, 3) =
      4.0e-2 * Eigen::Matrix<double, 3, 3>::Identity();
  fc_state.P.block<3, 3>(6, 6) =
      1.0e-2 * Eigen::Matrix<double, 3, 3>::Identity();
  fc_state.Q = Eigen::Matrix<double, 9, 9>::Identity();
  fc_state.Q.diagonal() << 0, 0, 0,
      option_.imu_noise_v_x * option_.imu_noise_v_x,
      option_.imu_noise_v_y * option_.imu_noise_v_y,
      option_.imu_noise_v_z * option_.imu_noise_v_z,
      option_.imu_noise_ori_x * option_.imu_noise_ori_x,
      option_.imu_noise_ori_y * option_.imu_noise_ori_y,
      option_.imu_noise_ori_z * option_.imu_noise_ori_z;
  fc_state.R_mm = Eigen::Matrix<double, 6, 6>::Identity();
  fc_state.R_mm.diagonal() << option_.mm_noise_x * option_.mm_noise_x,
      option_.mm_noise_y * option_.mm_noise_y,
      option_.mm_noise_z * option_.mm_noise_z,
      option_.mm_noise_ori_x * option_.mm_noise_ori_x,
      option_.mm_noise_ori_y * option_.mm_noise_ori_y,
      option_.mm_noise_ori_z * option_.mm_noise_ori_z;
  fc_state.R_ins = Eigen::Matrix<double, 6, 6>::Identity();
  fc_state.R_ins.diagonal() << option_.ins_noise_x * option_.ins_noise_x,
      option_.ins_noise_y * option_.ins_noise_y,
      option_.ins_noise_z * option_.ins_noise_z,
      option_.ins_noise_ori_x * option_.ins_noise_ori_x,
      option_.ins_noise_ori_y * option_.ins_noise_ori_y,
      option_.ins_noise_ori_z * option_.ins_noise_ori_z;
  fc_state.R_chassis = Eigen::Matrix<double, 3, 3>::Identity();
  fc_state.R_chassis.diagonal()
      << option_.chassis_noise_v_x * option_.chassis_noise_v_x,
      option_.chassis_noise_v_y * option_.chassis_noise_v_y,
      option_.chassis_noise_v_z * option_.chassis_noise_v_z;
  fc_map_mutex_.lock();
  fc_map_[fc_state.timestamp] = fc_state;
  fc_map_mutex_.unlock();
  return true;
}

bool FusionCenter::GetFirstFc() {
  double min_measure_timestamp = FLT_MAX;
  // 检验mm观测是否更新
  bool mm_update = false;
  static double last_mm_timestamp = 0;
  // 如果 mm_buffer_ 为空或者最后一个元素的时间戳没有变化，则不更新
  if (mm_buffer_->is_empty()) {
    mm_update = false;
  } else {
    latest_mm_ = *(mm_buffer_->back());
    if (latest_mm_.timestamp <= last_mm_timestamp) {
      mm_update = false;
    } else {
      // 如果有更新，则更新 last_mm_timestamp 并计算最小时间戳
      mm_update = true;
      min_measure_timestamp =
          std::min(min_measure_timestamp, latest_mm_.timestamp);
      last_mm_timestamp = latest_mm_.timestamp;
    }
  }

  // 检验ins观测是否更新
  bool ins_update = false;
  static double last_ins_timestamp = 0;
  // 如果 ins_buffer_ 最后一个元素的时间戳没有变化，则不更新
  if (latest_ins_.timestamp <= last_ins_timestamp) {
    ins_update = false;
  } else {
    // 如果有更新，则更新 last_ins_timestamp 并计算最小时间戳
    ins_update = true;
    min_measure_timestamp =
        std::min(min_measure_timestamp, latest_ins_.timestamp);
    last_ins_timestamp = latest_ins_.timestamp;
  }

  // 检验chassis观测是否更新
  bool chassis_update = false;
  static double last_chassis_timestamp = 0;
  // 如果 chassis_buffer_ 最后一个元素的时间戳没有变化，则不更新
  latest_chassis_.timestamp = chassis_buffer_->back()->timestamp;
  if (latest_chassis_.timestamp <= last_chassis_timestamp) {
    chassis_update = false;
  } else {
    // 如果有更新，则更新 last_chassis_timestamp 并计算最小时间戳
    chassis_update = true;
    last_chassis_timestamp = latest_chassis_.timestamp;
  }

  // 拿小于等于最新imu的最新chassis
  Measure::ConstPtr latest_chassis = nullptr;
  if (!chassis_buffer_->get_message_before_or_cur(latest_imu_.timestamp,
                                                  &latest_chassis)) {
    HLOG_WARN << "why can not get chassis (<= latest_imu time) ";
    HLOG_WARN << "chassis buffer is empty" << chassis_buffer_->is_empty();
    return false;
  }
  min_measure_timestamp =
      std::min(min_measure_timestamp, latest_chassis->timestamp);

  fc_map_mutex_.lock();
  if (fc_map_.empty()) {
    fc_map_mutex_.unlock();
    HLOG_WARN << "why fc_map is empty";
    return false;
  }
  auto it = fc_map_.lower_bound(min_measure_timestamp);
  if (it != fc_map_.begin()) {
    --it;
    first_fc_ = it->second;
  } else {
    fc_map_mutex_.unlock();
    HLOG_WARN << "why fc without less than mesasure";
    return false;
  }
  fc_map_mutex_.unlock();

  /*将所有大于first_fc且小于最新imu的imu放到vector中*/
  imu_buffer_->get_all_messages_after_end(first_fc_.timestamp,
                                          latest_imu_.timestamp, &imu_vector_);

  /*将所有大于first_fc且小于最新imu的观测放到vector中*/
  if (option_.use_chassis && chassis_update) {
    chassis_buffer_->get_all_messages_after_end(
        first_fc_.timestamp, latest_imu_.timestamp, &measure_vector_);
  }
  if (option_.use_ins && ins_update) {
    ins_buffer_->get_all_messages_after_end(
        first_fc_.timestamp, latest_imu_.timestamp, &measure_vector_);
  }

  if (option_.use_mm && mm_update) {
    mm_buffer_->get_all_messages_after_end(
        first_fc_.timestamp, latest_imu_.timestamp, &measure_vector_);
  }

  /*对观测和预测数据根据时间戳排序*/
  std::sort(measure_vector_.begin(), measure_vector_.end(),
            [](const Measure::ConstPtr& a, const Measure::ConstPtr& b) {
              return a->timestamp < b->timestamp;
            });
  return true;
}

void FusionCenter::BackEskf() {
  //         imu   mm   ins chassis imu  chassis  imu      chassis imu
  //
  //  队列为： ||____|____|____|______||______|____||___-----___|____||
  // 每一轮更新左侧时间戳
  FcState fc_state = first_fc_;
  fc_state.p = util::Geo::Gcj02ToEnu(fc_state.gcj_position, fc_ref_point_);
  for (int i = 0; i < static_cast<int>(imu_vector_.size()) - 1; ++i) {
    /*首先判断两帧imu之间是否有观测*/
    auto imu_left = imu_vector_[i];
    auto imu_right = imu_vector_[i + 1];
    std::vector<Measure::ConstPtr> mea_vector;
    for (const auto& mea : measure_vector_) {
      if (imu_left->timestamp < mea->timestamp &&
          imu_right->timestamp >= mea->timestamp) {
        mea_vector.push_back(mea);
      }
    }
    // HLOG_FATAL << "imu_left->timestamp:" << imu_left->timestamp;
    // HLOG_FATAL << "imu_right->timestamp:" << imu_right->timestamp;
    /*两帧imu之间无观测*/
    if (mea_vector.empty()) {
      double dt = imu_right->timestamp - imu_left->timestamp;
      Predict::Ptr mid_imu = std::make_shared<Predict>(*imu_right);
      mid_imu->angular_velocity_vrf =
          (imu_left->angular_velocity_vrf + imu_right->angular_velocity_vrf) /
          2;
      mid_imu->linear_acceleration_vrf = (imu_left->linear_acceleration_vrf +
                                          imu_right->linear_acceleration_vrf) /
                                         2;
      StatePredict(dt, mid_imu, &fc_state);
      fc_state.gcj_position = util::Geo::EnuToGcj02(fc_state.p, fc_ref_point_);
      fc_map_mutex_.lock();
      fc_map_[fc_state.timestamp] = fc_state;
      fc_map_mutex_.unlock();
      // HLOG_FATAL << "dt1: " << dt
      //            << " imu_left->timestamp: " << imu_left->timestamp
      //            << " imu_right->timestamp: " << imu_right->timestamp;
      // HLOG_FATAL << "fc_state: time: " << fc_state->timestamp
      //            << " p: " << fc_state->p.x() << " " << fc_state->p.y() << "
      //            "
      //            << fc_state->p.z()
      //            << " v: " << (fc_state->q.inverse() * fc_state->v).x() << "
      //            "
      //            << (fc_state->q.inverse() * fc_state->v).y() << " "
      //            << (fc_state->q.inverse() * fc_state->v).z();
      continue;
    }
    /*两帧imu之间有观测*/
    for (const auto& mea : mea_vector) {
      Predict::ConstPtr imu_left2 =
          GetImuByTimestamp(imu_vector_, fc_state.timestamp);
      Predict::ConstPtr imu_right2 =
          GetImuByTimestamp(imu_vector_, mea->timestamp);
      if (imu_left2 == nullptr) {
        HLOG_WARN << "why imu_left2 == nullptr";
        continue;
      }
      if (imu_right2 == nullptr) {
        HLOG_WARN << "why imu_right2 == nullptr";
        continue;
      }
      Predict::Ptr mid_imu = std::make_shared<Predict>(*imu_right2);
      mid_imu->angular_velocity_vrf =
          (imu_left2->angular_velocity_vrf + imu_right2->angular_velocity_vrf) /
          2;
      mid_imu->linear_acceleration_vrf = (imu_left2->linear_acceleration_vrf +
                                          imu_right2->linear_acceleration_vrf) /
                                         2;
      double dt = imu_right2->timestamp - imu_left2->timestamp;
      StatePredict(dt, mid_imu, &fc_state);
      StateMeasure(mea, &fc_state);
      // HLOG_FATAL << "dt2: " << dt << ","
      //            << " imu_left2->timestamp: " << imu_left2->timestamp
      //            << " imu_right2->timestamp: " << imu_right2->timestamp;
      // HLOG_FATAL << "mea->measure_type:" << mea->measure_type;
      // HLOG_FATAL << "mea->measure_type:" << mea->measure_type;
    }
    /*两帧imu之间融合到右侧imu只预测*/

    Predict::ConstPtr imu_left3 =
        GetImuByTimestamp(imu_vector_, fc_state.timestamp);
    if (imu_left3 == nullptr) {
      HLOG_WARN << "why imu_left3 == nullptr";
      continue;
    }
    const Predict::ConstPtr& imu_right3 = imu_right;
    Predict::Ptr mid_imu = std::make_shared<Predict>(*imu_right3);
    mid_imu->angular_velocity_vrf =
        (imu_left3->angular_velocity_vrf + imu_right3->angular_velocity_vrf) /
        2;
    mid_imu->linear_acceleration_vrf = (imu_left3->linear_acceleration_vrf +
                                        imu_right3->linear_acceleration_vrf) /
                                       2;
    double dt = imu_right3->timestamp - imu_left3->timestamp;
    StatePredict(dt, mid_imu, &fc_state);
    fc_state.gcj_position = util::Geo::EnuToGcj02(fc_state.p, fc_ref_point_);
    fc_map_mutex_.lock();
    fc_map_[fc_state.timestamp] = fc_state;
    fc_map_mutex_.unlock();
    // HLOG_FATAL << "dt3: " << dt
    //            << " imu_left3->timestamp: " << imu_left3->timestamp
    //            << " imu_right3->timestamp: " << imu_right3->timestamp;
    // HLOG_FATAL << "fc_state: time: " << fc_state->timestamp
    //            << " p: " << fc_state->p.x() << " " << fc_state->p.y() << " "
    //            << fc_state->p.z()
    //            << " v: " << (fc_state->q.inverse() * fc_state->v).x() << " "
    //            << (fc_state->q.inverse() * fc_state->v).y() << " "
    //            << (fc_state->q.inverse() * fc_state->v).z();
  }
}

void FusionCenter::UpdateLatestFc() {
  FcState fc_state = fc_map_[latest_imu_.timestamp];
  fc_state.state = 2;
  if (latest_mm_.warn_info == 130) {
    fc_state.state = 130;
  }
  if (latest_imu_.timestamp - latest_mm_.timestamp > 3) {
    fc_state.state = 5;
  }
  fc_state.rtk_state = latest_ins_.rtk_state;
  fc_state.second = latest_ins_.second;
  fc_state.week = latest_ins_.week;
  fc_state.ins_sd_position =
      std::max(latest_ins_.sd_position.x(), latest_ins_.sd_position.y());
  fc_state.ins_heading = latest_ins_.ins_heading;
  fc_state.ins_height = latest_ins_.gcj_position.z();
  auto dr = GetDrByTimestamp(fc_state.timestamp);
  if (dr != nullptr) {
    fc_state.p_dr = dr->dr_position;
    fc_state.q_dr = dr->dr_q;
    auto euler_angle = RotionMatrix2EulerAngle321(fc_state.q_dr.matrix());
    fc_state.yaw_dr = euler_angle[2];
    fc_state.pitch_dr = euler_angle[1];
  }
  // chassis横向速度为0
  // fc_state.linear_velocity_vrf = latest_chassis_.linear_velocity_vrf;
  fc_state.linear_velocity_vrf = fc_state.q.inverse() * fc_state.v;
  fc_state.angular_velocity_vrf = latest_imu_.angular_velocity_vrf;
  fc_state.linear_acceleration_vrf = latest_imu_.linear_acceleration_vrf;

  // 可视化ins偏差估计
  fc_state.p_ins_estimate = latest_ins_.gcj_position;
  fc_state.q_ins_estimate = latest_ins_.enu_q;

  fc_map_[latest_imu_.timestamp] = fc_state;
}

void FusionCenter::StatePredict(double dt, const Predict::ConstPtr& cur_imu,
                                FcState* fc_state) {
  // 名义状态更新
  fc_state->timestamp = cur_imu->timestamp;
  auto last_q = fc_state->q;
  fc_state->q = Eigen::Quaterniond(
      fc_state->q.matrix() *
      Sophus::SO3d::exp(cur_imu->angular_velocity_vrf / 180 * M_PI * dt)
          .matrix());
  fc_state->q.normalize();
  auto mid_q = Eigen::Quaterniond(
      (last_q.w() + fc_state->q.w()) / 2, (last_q.x() + fc_state->q.x()) / 2,
      (last_q.y() + fc_state->q.y()) / 2, (last_q.z() + fc_state->q.z()) / 2);
  auto last_v = fc_state->v;
  fc_state->v =
      fc_state->v + (mid_q * cur_imu->linear_acceleration_vrf * 9.8) * dt;
  auto mid_v = (fc_state->v + last_v) / 2;
  fc_state->p =
      fc_state->p + mid_v * dt +
      0.5 * (fc_state->q * cur_imu->linear_acceleration_vrf * 9.8) * dt * dt;
  // 误差状态预测
  Eigen::Matrix<double, 9, 9> F = Eigen::Matrix<double, 9, 9>::Identity();
  F.template block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
  F.template block<3, 3>(3, 6) =
      -fc_state->q.matrix() *
      Sophus::SO3d::hat(cur_imu->linear_acceleration_vrf * 9.8) * dt;
  F.template block<3, 3>(6, 6) =
      Sophus::SO3d::exp(-cur_imu->angular_velocity_vrf / 180 * M_PI * dt)
          .matrix();
  fc_state->dx = F * fc_state->dx;
  fc_state->P = F * fc_state->P.eval() * F.transpose() + fc_state->Q;
}

Measure::Ptr FusionCenter::GetNewIns(const Measure::ConstPtr& cur_ins) {
  Measure::Ptr new_ins = std::make_shared<Measure>(*cur_ins);

  if (mm_buffer_->is_empty()) {
    new_ins->timestamp = 0;
    return new_ins;
  }
  // 记录当前ins的位置和时间
  Eigen::Vector3d ins_position =
      util::Geo::Gcj02ToEnu(cur_ins->gcj_position, fc_ref_point_);
  double ins_time = cur_ins->timestamp;
  // 记录当前mm的位置和时间
  Eigen::Vector3d mm_position =
      util::Geo::Gcj02ToEnu(latest_mm_.gcj_position, fc_ref_point_);
  double mm_time = latest_mm_.timestamp;

  // 车系下ins和mm的位置
  Eigen::Vector3d ins_position_vrf = (cur_ins->enu_q).inverse() * ins_position;
  Eigen::Vector3d mm_position_vrf = (latest_mm_.enu_q).inverse() * mm_position;
  // HLOG_FATAL << "mm_position:" << mm_time << "," << mm_position_vrf.x() <<
  // ","
  //            << mm_position_vrf.y() << "," << mm_position_vrf.z();
  // HLOG_FATAL << "ins_position:" << ins_time << "," << ins_position_vrf.x()
  //            << "," << ins_position_vrf.y() << "," << ins_position_vrf.z();

  // 偏差估计条件判断
  if (cur_ins->rtk_state == 4 && cur_ins->sd_position.x() <= 0.05 &&
      cur_ins->sd_position.y() <= 0.05 &&
      (ins_position.norm() - mm_position.norm() < 100) &&
      ((ins_time - mm_time) < 2.0) && ((ins_time - mm_time) > 0.0)) {
    // HLOG_FATAL << "GetNewIns in";
    Eigen::Affine3d T_w_mm =
        Eigen::Translation3d(mm_position) * Eigen::Affine3d(latest_mm_.enu_q);
    Eigen::Affine3d T_w_ins =
        Eigen::Translation3d(ins_position) * Eigen::Affine3d(cur_ins->enu_q);
    // 获取mm和ins在车系下y和heading偏差
    Eigen::Matrix4d T_ins_mm = (T_w_ins.inverse() * T_w_mm).matrix();
    Eigen::Vector3d euler_angle_ins_mm =
        RotionMatrix2EulerAngle321(T_ins_mm.block<3, 3>(0, 0));
    Eigen::Matrix3d new_R =
        (Eigen::AngleAxisd(euler_angle_ins_mm[2], Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()))
            .matrix();
    double new_y = T_ins_mm(1, 3);
    Eigen::Matrix4d T_diff = Eigen::Matrix4d::Identity();
    T_diff(1, 3) = new_y;              // 使用mm的y
    T_diff.block<3, 3>(0, 0) = new_R;  // 使用mm的heading
    // 将mm和ins偏差结果对ins进行更新，获取加偏后的ins
    auto T_w_ins_new = T_w_ins * T_diff;
    new_ins->enu_position = T_w_ins_new.block<3, 1>(0, 3);
    new_ins->enu_q = Eigen::Quaterniond(T_w_ins_new.block<3, 3>(0, 0));
  } else {
    new_ins->timestamp = 0;
  }
  return new_ins;
}

Measure::Ptr FusionCenter::GetNewInsFilter(
    const Measure::ConstPtr& cur_ins, const Measure::ConstPtr& cur_mm,
    const Eigen::Vector3d& ref_point_mm) {
  // HLOG_INFO << "GetNewInsFilter in";
  Measure::Ptr new_ins = std::make_shared<Measure>(*cur_ins);

  // 记录当前ins的位置和时间
  Eigen::Vector3d ins_position =
      util::Geo::Gcj02ToEnu(cur_ins->gcj_position, ref_point_mm);
  double ins_time = cur_ins->timestamp;
  // 记录当前mm的位置和时间
  Eigen::Vector3d mm_position =
      util::Geo::Gcj02ToEnu(cur_mm->gcj_position, ref_point_mm);
  double mm_time = cur_mm->timestamp;

  // 车系下ins和mm的位置
  Eigen::Vector3d ins_position_vrf = (cur_ins->enu_q).inverse() * ins_position;
  Eigen::Vector3d mm_position_vrf = (cur_mm->enu_q).inverse() * mm_position;
  // HLOG_INFO << "mm_position:" << mm_time << "," << mm_position_vrf.x() << ","
  //           << mm_position_vrf.y() << "," << mm_position_vrf.z();
  // HLOG_INFO << "ins_position:" << ins_time << "," << ins_position_vrf.x() <<
  // ","
  //           << ins_position_vrf.y() << "," << ins_position_vrf.z();

  /*mm无效一定时间内且ins的rtk=4进行ins偏差估计*/
  static bool first_offset = true;
  static Eigen::Matrix4d T_offset = Eigen::Matrix4d::Identity();
  // 做当前时刻的偏差估计
  if (cur_ins->rtk_state == 4 && cur_ins->sd_position.x() <= 0.05 &&
      cur_ins->sd_position.y() <= 0.05 &&
      (cur_ins->timestamp - cur_mm->timestamp) < 3) {
    // HLOG_INFO << "get in rtk=4";
    // 用ins和mm做偏差估计
    Eigen::Affine3d T_w_mm =
        Eigen::Translation3d(mm_position) * Eigen::Affine3d(cur_mm->enu_q);
    Eigen::Affine3d T_w_ins =
        Eigen::Translation3d(ins_position) * Eigen::Affine3d(cur_ins->enu_q);
    // 获取mm和ins在车系下y和heading偏差
    Eigen::Matrix4d T_ins_mm = (T_w_ins.inverse() * T_w_mm).matrix();
    Eigen::Vector3d euler_angle_ins_mm =
        RotionMatrix2EulerAngle321(T_ins_mm.block<3, 3>(0, 0));
    Eigen::Matrix3d new_R =
        (Eigen::AngleAxisd(euler_angle_ins_mm[2], Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()))
            .matrix();
    double new_y = T_ins_mm(1, 3);
    Eigen::Matrix4d T_diff = Eigen::Matrix4d::Identity();
    T_diff(1, 3) = new_y;              // 补偿量横向y
    T_diff.block<3, 3>(0, 0) = new_R;  // 补偿量heading

    // HLOG_INFO << "T_diff_y" << T_diff(1, 3);
    /*对偏差进行低通滤波*/
    static Eigen::Vector3d last_ins_position = Eigen::Vector3d::Zero();
    static double last_ins_time = 0;

    // 偏差量过大直接返回ins值
    if (std::abs(T_diff(1, 3)) > 1.75) {
      // HLOG_INFO << "T_diff is large";
      new_ins->enu_position = ins_position;
      new_ins->enu_q = cur_ins->enu_q;
      // 偏差估计结果转到车体系debug
      Eigen::Vector3d new_ins_vrf =
          (new_ins->enu_q).inverse() * new_ins->enu_position;
      // HLOG_INFO << "new_ins->enu_position:" << new_ins->timestamp << ","
      //           << new_ins_vrf.x() << "," << new_ins_vrf.y() << ","
      //           << new_ins_vrf.z();
      return new_ins;
    }

    // 判断是否是第一次做偏差估计
    // HLOG_FATAL << "offset_.init:" << offset_.init;
    if (!offset_.init) {
      // 判断上一时刻偏差估计和当前时刻偏差估计的距离,如果大于一定距离则不进行滤波
      if ((ins_position.norm() - offset_.pos.norm() > 200) ||
          (ins_time - offset_.time) > 3.0) {
        // HLOG_INFO << "ins_time - offset_.time) > 2.0";
        offset_.T_offset = T_diff;
        offset_.pos = ins_position;
        offset_.time = ins_time;
      } else {
        // HLOG_INFO << "get in filter";
        offset_.T_offset = 0.2 * T_diff + 0.8 * offset_.T_offset;
        offset_.pos = ins_position;
        offset_.time = ins_time;
      }
    } else {
      // HLOG_INFO << "offset_.init = false";
      offset_.init = false;
      offset_.T_offset = T_diff;
      offset_.pos = ins_position;
      offset_.time = ins_time;
    }
    // HLOG_INFO << "offset_.T_offset_y" << offset_.T_offset(1, 3);

    // 将mm和ins偏差结果对ins进行更新，获取偏差估计后的ins
    auto T_w_ins_new = T_w_ins * offset_.T_offset;
    new_ins->enu_position = T_w_ins_new.block<3, 1>(0, 3);
    new_ins->enu_q = Eigen::Quaterniond(T_w_ins_new.block<3, 3>(0, 0));

    // 偏差估计结果转到车体系debug
    Eigen::Vector3d new_ins_vrf =
        (new_ins->enu_q).inverse() * new_ins->enu_position;
    // HLOG_INFO << "new_ins->enu_position:" << new_ins->timestamp << ","
    //           << new_ins_vrf.x() << "," << new_ins_vrf.y() << ","
    //           << new_ins_vrf.z();
    return new_ins;
  } else {
    // 测试有mm但rtk！＝４的低通滤波效果
    // HLOG_INFO << "rtk!=4 && 2s没有mm";
    new_ins->enu_position = ins_position;
    new_ins->enu_q = cur_ins->enu_q;

    // 偏差估计结果转到车体系debug
    Eigen::Vector3d new_ins_vrf =
        (new_ins->enu_q).inverse() * new_ins->enu_position;
    // HLOG_INFO << "new_ins->enu_position:" << new_ins->timestamp << ","
    //           << new_ins_vrf.x() << "," << new_ins_vrf.y() << ","
    //           << new_ins_vrf.z();

    return new_ins;
  }
}

Measure::Ptr FusionCenter::GetNewInsReal(const Measure::ConstPtr& cur_ins,
                                         const Measure::ConstPtr& cur_mm,
                                         const Eigen::Vector3d& ref_point_mm) {
  // HLOG_INFO << "GetNewInsReal in";
  Measure::Ptr new_ins = std::make_shared<Measure>(*cur_ins);

  // 记录当前ins的位置和时间
  Eigen::Vector3d ins_position =
      util::Geo::Gcj02ToEnu(cur_ins->gcj_position, ref_point_mm);
  double ins_time = cur_ins->timestamp;
  // 记录当前mm的位置和时间
  Eigen::Vector3d mm_position =
      util::Geo::Gcj02ToEnu(cur_mm->gcj_position, ref_point_mm);
  double mm_time = cur_mm->timestamp;

  // 车系下ins和mm的位置
  Eigen::Vector3d ins_position_vrf = (cur_ins->enu_q).inverse() * ins_position;
  Eigen::Vector3d mm_position_vrf = (cur_mm->enu_q).inverse() * mm_position;
  // HLOG_INFO << "mm_position:" << mm_time << "," << mm_position_vrf.x() << ","
  //           << mm_position_vrf.y() << "," << mm_position_vrf.z();
  // HLOG_INFO << "ins_position:" << ins_time << "," << ins_position_vrf.x() << ","
  //           << ins_position_vrf.y() << "," << ins_position_vrf.z();

  // 不管rtk状态　mm有效就进行偏差估计
  if (ins_time - mm_time < 3.0) {
    // HLOG_INFO << "ins_time - mm_time < 3.0";
    Eigen::Affine3d T_w_mm =
        Eigen::Translation3d(mm_position) * Eigen::Affine3d(cur_mm->enu_q);
    Eigen::Affine3d T_w_ins =
        Eigen::Translation3d(ins_position) * Eigen::Affine3d(cur_ins->enu_q);
    // 获取mm和ins在车系下y和heading偏差
    Eigen::Matrix4d T_ins_mm = (T_w_ins.inverse() * T_w_mm).matrix();
    Eigen::Vector3d euler_angle_ins_mm =
        RotionMatrix2EulerAngle321(T_ins_mm.block<3, 3>(0, 0));
    Eigen::Matrix3d new_R =
        (Eigen::AngleAxisd(euler_angle_ins_mm[2], Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()))
            .matrix();
    double new_y = T_ins_mm(1, 3);
    Eigen::Matrix4d T_diff = Eigen::Matrix4d::Identity();
    T_diff(1, 3) = new_y;              // 补偿量横向y
    T_diff.block<3, 3>(0, 0) = new_R;  // 补偿量heading

    // 将mm和ins偏差结果对ins进行更新，获取偏差估计后的ins
    auto T_w_ins_new = T_w_ins * T_diff;
    new_ins->enu_position = T_w_ins_new.block<3, 1>(0, 3);
    new_ins->enu_q = Eigen::Quaterniond(T_w_ins_new.block<3, 3>(0, 0));

    return new_ins;
  } else {
    new_ins->enu_position = ins_position;
    new_ins->enu_q = cur_ins->enu_q;

    return new_ins;
  }
}

Eigen::Vector3d FusionCenter::RotionMatrix2EulerAngle321(
    const Eigen::Matrix3d& rotation_matrix) {
  double roll = NAN;
  double pitch = NAN;
  double yaw = NAN;
  if (rotation_matrix(0, 2) < 1) {
    if (rotation_matrix(0, 2) > -1) {
      // Calculate roll, pitch, and yaw angles
      roll = atan2(rotation_matrix(2, 1), rotation_matrix(2, 2));
      pitch = asin(-rotation_matrix(2, 0));
      yaw = atan2(rotation_matrix(1, 0), rotation_matrix(0, 0));
    } else {  // rotationMatrix(0, 2) == -1
      // Gimbal lock case: pitch = -pi/2, roll - yaw can be solved as roll +
      // yaw
      roll = -atan2(-rotation_matrix(1, 2), rotation_matrix(1, 1));
      pitch = -M_PI / 2;
      yaw = 0;
    }
  } else {  // rotationMatrix(0, 2) == 1
    // Gimbal lock case: pitch = pi/2, roll + yaw can be solved as roll - yaw
    roll = atan2(-rotation_matrix(1, 2), rotation_matrix(1, 1));
    pitch = M_PI / 2;
    yaw = 0;
  }
  return {roll, pitch, yaw};
}

void FusionCenter::StateMeasure(const Measure::ConstPtr& measure,
                                FcState* fc_state) {
  fc_state->timestamp = measure->timestamp;
  if (measure->measure_type == MM || measure->measure_type == INS) {
    Eigen::Matrix<double, 6, 9> H = Eigen::Matrix<double, 6, 9>::Zero();
    H.block<3, 3>(0, 0) = Eigen::Matrix<double, 3, 3>::Identity();  // p部分
    H.block<3, 3>(3, 6) = Eigen::Matrix<double, 3, 3>::Identity();  // q部分
    // 误差状态更新
    fc_state->R_mm.block<3, 3>(0, 0) =
        fc_state->q * fc_state->R_mm.block<3, 3>(0, 0) * fc_state->q.inverse();
    fc_state->R_mm.block<3, 3>(3, 3) =
        fc_state->q * fc_state->R_mm.block<3, 3>(3, 3) * fc_state->q.inverse();
    fc_state->R_ins.block<3, 3>(0, 0) =
        fc_state->q * fc_state->R_ins.block<3, 3>(0, 0) * fc_state->q.inverse();
    fc_state->R_ins.block<3, 3>(3, 3) =
        fc_state->q * fc_state->R_ins.block<3, 3>(3, 3) * fc_state->q.inverse();
    Eigen::Matrix<double, 9, 6> K;
    if (measure->measure_type == MM) {
      K = fc_state->P * H.transpose() *
          (H * fc_state->P.eval() * H.transpose() + fc_state->R_mm).inverse();
      Eigen::Matrix<double, 6, 1> diff = Eigen::Matrix<double, 6, 1>::Zero();
      diff.block<3, 1>(0, 0) =
          util::Geo::Gcj02ToEnu(measure->gcj_position, fc_ref_point_) -
          fc_state->p;
      auto q_diff = fc_state->q.inverse() * measure->enu_q;
      q_diff.normalize();
      diff.block<3, 1>(3, 0) = Sophus::SO3d(q_diff).log();
      fc_state->dx = K * diff;
      fc_state->P =
          (Eigen::Matrix<double, 9, 9>::Identity() - K * H) * fc_state->P;
    } else if (measure->measure_type == INS) {
      K = fc_state->P * H.transpose() *
          (H * fc_state->P.eval() * H.transpose() + fc_state->R_ins).inverse();
      Eigen::Matrix<double, 6, 1> diff = Eigen::Matrix<double, 6, 1>::Zero();
      diff.block<3, 1>(0, 0) =
          util::Geo::Gcj02ToEnu(measure->gcj_position, fc_ref_point_) -
          fc_state->p;
      auto q_diff = fc_state->q.inverse() * measure->enu_q;
      q_diff.normalize();
      diff.block<3, 1>(3, 0) = Sophus::SO3d(q_diff).log();
      fc_state->dx = K * diff;
      fc_state->P =
          (Eigen::Matrix<double, 9, 9>::Identity() - K * H) * fc_state->P;
    }

    // HLOG_FATAL << "x_diff " << diff.block<3, 1>(0, 0).x();
    // HLOG_FATAL << "y_diff " << diff.block<3, 1>(0, 0).y();
    // HLOG_FATAL << "z_diff " << diff.block<3, 1>(0, 0).z();
    // 状态更新
    // HLOG_FATAL << "fc_state timestamp " << fc_state->timestamp;
    // HLOG_FATAL << "before x " << fc_state->p.x();
    // HLOG_FATAL << "before y " << fc_state->p.y();
    // HLOG_FATAL << "before z " << fc_state->p.z();
    fc_state->p = fc_state->p + fc_state->dx.block<3, 1>(0, 0);
    fc_state->gcj_position = util::Geo::EnuToGcj02(fc_state->p, fc_ref_point_);
    // HLOG_FATAL << "fc_state timestamp " << fc_state->timestamp;
    // HLOG_FATAL << "after x " << fc_state->p.x();
    // HLOG_FATAL << "after y " << fc_state->p.y();
    // HLOG_FATAL << "after z " << fc_state->p.z();
    // HLOG_FATAL << "------------------";
    // HLOG_FATAL << "q_diff roll "
    //            << RotionMatrix2EulerAngle321(q_diff.matrix())[0] / M_PI *
    //            180;
    // HLOG_FATAL << "q_diff pitch "
    //            << RotionMatrix2EulerAngle321(q_diff.matrix())[1] / M_PI *
    //            180;
    // HLOG_FATAL << "q_diff yaw "
    //            << RotionMatrix2EulerAngle321(q_diff.matrix())[2] / M_PI *
    //            180;
    // HLOG_FATAL << "before roll "
    //            << RotionMatrix2EulerAngle321(fc_state->q.matrix())[0] / M_PI
    //            *
    //                   180;
    // HLOG_FATAL << "before pitch "
    //            << RotionMatrix2EulerAngle321(fc_state->q.matrix())[1] / M_PI
    //            *
    //                   180;
    // HLOG_FATAL << "before yaw "
    //            << RotionMatrix2EulerAngle321(fc_state->q.matrix())[2] / M_PI
    //            *
    //                   180;
    fc_state->q = fc_state->q *
                  Sophus::SO3d::exp(fc_state->dx.block<3, 1>(6, 0)).matrix();
    fc_state->q.normalize();
    // HLOG_FATAL << "after roll "
    //            << RotionMatrix2EulerAngle321(fc_state->q.matrix())[0] / M_PI
    //            *
    //                   180;
    // HLOG_FATAL << "after pitch "
    //            << RotionMatrix2EulerAngle321(fc_state->q.matrix())[1] / M_PI
    //            *
    //                   180;
    // HLOG_FATAL << "after yaw "
    //            << RotionMatrix2EulerAngle321(fc_state->q.matrix())[2] / M_PI
    //            *
    //                   180;
    // HLOG_FATAL << "before vx " << fc_state->v.x();
    // HLOG_FATAL << "before vy " << fc_state->v.y();
    // HLOG_FATAL << "before vz " << fc_state->v.z();
    fc_state->v = fc_state->v + fc_state->dx.block<3, 1>(3, 0);
    // HLOG_FATAL << "after vx " << fc_state->v.x();
    // HLOG_FATAL << "after vy " << fc_state->v.y();
    // HLOG_FATAL << "after vz " << fc_state->v.z();

    // 误差状态重置
    fc_state->dx = Eigen::Matrix<double, 9, 1>::Zero();
    Eigen::Matrix<double, 9, 9> J = Eigen::Matrix<double, 9, 9>::Identity();
    J.block<3, 3>(6, 6) =
        Eigen::Matrix<double, 3, 3>::Identity() -
        0.5 * Sophus::SO3d::hat(fc_state->dx.block<3, 1>(6, 0));
    fc_state->P = J * fc_state->P * J.transpose();
  } else if (measure->measure_type == CHASSIS) {
    Eigen::Matrix<double, 3, 9> H = Eigen::Matrix<double, 3, 9>::Zero();
    H.block<3, 3>(0, 3) = Eigen::Matrix<double, 3, 3>::Identity();  // v部分
    // 误差状态更新
    Eigen::Matrix<double, 9, 3> K =
        fc_state->P * H.transpose() *
        (H * fc_state->P.eval() * H.transpose() +
         fc_state->q * fc_state->R_chassis * fc_state->q.inverse())
            .inverse();

    Eigen::Matrix<double, 3, 1> diff = Eigen::Matrix<double, 3, 1>::Zero();
    diff = fc_state->q * measure->linear_velocity_vrf - fc_state->v;
    fc_state->dx = K * diff;
    fc_state->P =
        (Eigen::Matrix<double, 9, 9>::Identity() - K * H) * fc_state->P;

    // 状态更新
    fc_state->v = fc_state->v + fc_state->dx.block<3, 1>(3, 0);

    // 误差状态重置
    fc_state->dx = Eigen::Matrix<double, 9, 1>::Zero();
    Eigen::Matrix<double, 9, 9> J = Eigen::Matrix<double, 9, 9>::Identity();
    J.block<3, 3>(6, 6) =
        Eigen::Matrix<double, 3, 3>::Identity() -
        0.5 * Sophus::SO3d::hat(fc_state->dx.block<3, 1>(6, 0));
    fc_state->P = J * fc_state->P * J.transpose();
  }
}

Measure::ConstPtr FusionCenter::GetChassisByTimestamp(double timestamp) {
  Measure::ConstPtr before = nullptr;
  Measure::ConstPtr after = nullptr;
  // |   _____      返回空指针
  //    |_____      返回空指针
  //     __|__      返回before和after
  //     _____|     返回队列end为after
  //     _____   |  返回空指针
  chassis_buffer_->get_messages_around(timestamp, before, after);
  // 情况1 2 5
  if (before == nullptr && after == nullptr) {
    HLOG_WARN << "GetChassisByTimestamp is nullptr: " << timestamp;
    return nullptr;
  }
  // 情况4
  if (before == nullptr) {
    Measure::Ptr chassis_ptr = std::make_shared<Measure>();
    chassis_ptr->timestamp = timestamp;
    chassis_ptr->linear_velocity_vrf = after->linear_velocity_vrf;
    return chassis_ptr;
  }
  // 概率极小，队列存在传入的时间戳
  if (before->timestamp == timestamp && after->timestamp == timestamp) {
    Measure::Ptr chassis_ptr = std::make_shared<Measure>();
    chassis_ptr->timestamp = timestamp;
    chassis_ptr->linear_velocity_vrf = after->linear_velocity_vrf;
    return chassis_ptr;
  }
  // 不会发生，过滤异常数据
  if (after->timestamp <= before->timestamp) {
    HLOG_WARN
        << "GetChassisPoseForTime: after->timestamp <= before->timestamp: "
        << after->timestamp << " < " << before->timestamp;
    return nullptr;
  }
  // 情况3
  double front_scale =
      (after->timestamp - timestamp) / (after->timestamp - before->timestamp);
  double back_scale = 1 - front_scale;
  Measure::Ptr chassis_ptr = std::make_shared<Measure>();
  chassis_ptr->timestamp = timestamp;
  chassis_ptr->linear_velocity_vrf = before->linear_velocity_vrf * front_scale +
                                     after->linear_velocity_vrf * back_scale;
  return chassis_ptr;
}

Measure::ConstPtr FusionCenter::GetInsByTimestamp(double timestamp) {
  Measure::ConstPtr before = nullptr;
  Measure::ConstPtr after = nullptr;
  // |   _____      返回空指针
  //    |_____      返回空指针
  //     __|__      返回before和after
  //     _____|     返回队列end为after
  //     _____   |  返回空指针
  ins_buffer_->get_messages_around(timestamp, before, after);
  // 情况1 2 5
  if (before == nullptr && after == nullptr) {
    HLOG_WARN << "GetInsByTimestamp is nullptr: " << timestamp;
    return nullptr;
  }
  // 情况4
  if (before == nullptr) {
    Measure::Ptr ins_ptr = std::make_shared<Measure>();
    ins_ptr->timestamp = timestamp;
    ins_ptr->gcj_position = after->gcj_position;
    ins_ptr->enu_q = after->enu_q;
    return ins_ptr;
  }
  // 队列存在传入的时间戳
  if (before->timestamp == timestamp && after->timestamp == timestamp) {
    return before;
  }
  // 不会发生，过滤异常数据
  if (after->timestamp <= before->timestamp) {
    HLOG_WARN << "GetInsPoseForTime: after->timestamp <= before->timestamp: "
              << after->timestamp << " < " << before->timestamp;
    return nullptr;
  }
  // 情况3
  double front_scale =
      (after->timestamp - timestamp) / (after->timestamp - before->timestamp);
  double back_scale = 1 - front_scale;
  Measure::Ptr ins_ptr = std::make_shared<Measure>();
  ins_ptr->timestamp = timestamp;
  ins_ptr->gcj_position =
      before->gcj_position * front_scale + after->gcj_position * back_scale;
  ins_ptr->enu_q = before->enu_q.slerp(back_scale, after->enu_q);
  return ins_ptr;
}

Predict::ConstPtr FusionCenter::GetImuByTimestamp(
    const std::vector<Predict::ConstPtr>& imu_vector, double timestamp) {
  if (imu_vector.empty()) {
    return nullptr;
  }
  // |   _____      返回空指针
  //    |_____      返回队列front
  //     __|__      插值
  //     _____|     返回队列end
  //     _____   |  返回空指针
  // 情况1 imu队列的时间在传入时间前面很远
  if (imu_vector.front()->timestamp > timestamp + 0.02) {
    HLOG_WARN << "imu is too new!";
    return nullptr;
  }
  // 情况5 imu队列的时间在传入时间后面很远
  if (imu_vector.back()->timestamp < timestamp - 0.02) {
    HLOG_WARN << "imu is too old!";
    return nullptr;
  }
  std::vector<Predict::ConstPtr> left_imu_vector;
  std::vector<Predict::ConstPtr> right_imu_vector;
  for (const auto& imu : imu_vector) {
    if (imu->timestamp < timestamp) {
      left_imu_vector.emplace_back(imu);
    } else {
      right_imu_vector.emplace_back(imu);
    }
  }
  // 情况2 imu队列的时间在传入时间前面不远处，取第一帧imu
  if (left_imu_vector.empty()) {
    Predict::Ptr imu_ptr = std::make_shared<Predict>(*right_imu_vector.front());
    imu_ptr->timestamp = timestamp;
    return imu_ptr;
  }
  // 情况4 imu队列的时间在传入时间前面不远处，取最后一帧imu
  if (right_imu_vector.empty()) {
    Predict::Ptr imu_ptr = std::make_shared<Predict>(*left_imu_vector.back());
    imu_ptr->timestamp = timestamp;
    return imu_ptr;
  }
  // 情况3 imu队列的时间包含传入时间，取相邻的两帧imu插值
  auto front = left_imu_vector.back();
  auto back = right_imu_vector.front();
  double front_scale = (back->timestamp - timestamp) /
                       (back->timestamp - front->timestamp + 1e-6);
  double back_scale = 1 - front_scale;
  Predict::Ptr imu_ptr = std::make_shared<Predict>();
  imu_ptr->timestamp = timestamp;
  imu_ptr->angular_velocity_vrf = front->angular_velocity_vrf * front_scale +
                                  back->angular_velocity_vrf * back_scale;
  imu_ptr->linear_acceleration_vrf =
      front->linear_acceleration_vrf * front_scale +
      back->linear_acceleration_vrf * back_scale;
  return imu_ptr;
}

Dr::ConstPtr FusionCenter::GetDrByTimestamp(double timestamp) {
  Dr::ConstPtr before = nullptr;
  Dr::ConstPtr after = nullptr;
  // |   _____      返回空指针
  //    |_____      返回空指针
  //     __|__      返回before和after
  //     _____|     返回队列end为after
  //     _____   |  返回空指针
  dr_buffer_->get_messages_around(timestamp, before, after);
  // 情况1 2 5
  if (before == nullptr && after == nullptr) {
    HLOG_ERROR << "GetDrByTimestamp is nullptr: " << timestamp;
    return nullptr;
  }
  // 情况4
  if (before == nullptr) {
    Dr::Ptr dr_ptr = std::make_shared<Dr>();
    dr_ptr->timestamp = timestamp;
    dr_ptr->dr_position = after->dr_position;
    dr_ptr->dr_q = after->dr_q;
    return dr_ptr;
  }
  // 概率极小，队列存在传入的时间戳
  if (before->timestamp == timestamp && after->timestamp == timestamp) {
    return before;
  }
  // 不会发生，过滤异常数据
  if (after->timestamp <= before->timestamp) {
    HLOG_ERROR << "GetDrPoseForTime: after->timestamp <= before->timestamp: "
               << after->timestamp << " < " << before->timestamp;
    return nullptr;
  }
  // 情况3
  double front_scale = (after->timestamp - timestamp) /
                       (after->timestamp - before->timestamp + 1e-6);
  double back_scale = 1 - front_scale;
  Dr::Ptr dr_ptr = std::make_shared<Dr>();
  dr_ptr->timestamp = timestamp;
  dr_ptr->dr_position =
      before->dr_position * front_scale + after->dr_position * back_scale;
  dr_ptr->dr_q = before->dr_q.slerp(back_scale, after->dr_q);
  return dr_ptr;
}

void FusionCenter::OnIns(const HafNodeInfo& ins) {
  static int count = 0;
  count++;
  if (count >= 10) {
    count = 0;
  } else {
    return;
  }
  if (!ins.valid_estimate()) {
    HLOG_WARN << "ins is not valid!";
    return;
  }
  static auto last_ins = ins;
  if (ins.header().seq() <= last_ins.header().seq()) {
    HLOG_ERROR << "error, insfusion seq: " << ins.header().seq();
    return;
  }
  if (ins.header().data_stamp() <= last_ins.header().data_stamp()) {
    HLOG_ERROR << "error, insfusion data_stamp: " << ins.header().data_stamp();
    return;
  }

  if (std::isnan(ins.pos_gcj02().x()) || std::isnan(ins.pos_gcj02().x()) ||
      std::isnan(ins.pos_gcj02().x()) || std::isnan(ins.quaternion().w()) ||
      std::isnan(ins.quaternion().x()) || std::isnan(ins.quaternion().y()) ||
      std::isnan(ins.quaternion().z())) {
    HLOG_ERROR << "insfusion data has nan";
  }

  if (Eigen::Quaterniond(ins.quaternion().w(), ins.quaternion().x(),
                         ins.quaternion().y(), ins.quaternion().z())
          .norm() < 1e-6) {
    HLOG_ERROR << "insfusion quaternion norm < 1e-6 ";
    return;
  }

  // refpoint初始赋值
  Eigen::Vector3d gcj_position{ins.pos_gcj02().x(), ins.pos_gcj02().y(),
                               ins.pos_gcj02().z()};
  ref_point.UpdateRefpoint(gcj_position);
  Measure::Ptr ins_data = std::make_shared<Measure>();
  ins_data->timestamp = ins.header().data_stamp();
  ins_data->measure_type = INS;
  ins_data->gcj_position.x() = ins.pos_gcj02().x();
  ins_data->gcj_position.y() = ins.pos_gcj02().y();
  ins_data->gcj_position.z() = ins.pos_gcj02().z();
  ins_data->enu_q.w() = ins.quaternion().w();
  ins_data->enu_q.x() = ins.quaternion().x();
  ins_data->enu_q.y() = ins.quaternion().y();
  ins_data->enu_q.z() = ins.quaternion().z();
  ins_data->rtk_state = static_cast<int>(ins.gps_status());
  ins_data->second = ins.gps_sec();
  ins_data->week = ins.gps_week();
  ins_data->sd_position.x() = ins.sd_position().x();
  ins_data->sd_position.y() = ins.sd_position().y();
  ins_data->sd_position.z() = ins.sd_position().z();
  ins_data->ins_heading = ins.heading();

  // ins偏差估计
  /*补偿量和补偿都在ins里做*/
  if (!mm_buffer_->is_empty()) {
    Measure::ConstPtr mm_data = (mm_buffer_->back());
    Eigen::Vector3d ref_point_ins = ref_point.GetRefPoint();

    Measure::Ptr new_ins_filter =
        GetNewInsFilter(ins_data, mm_data, ref_point_ins);

    ins_data->gcj_position =
        util::Geo::EnuToGcj02(new_ins_filter->enu_position, ref_point_ins);
    ins_data->enu_q = new_ins_filter->enu_q;

    if (ins_data->rtk_state != 4) {
      Measure::Ptr new_ins_real =
          GetNewInsReal(ins_data, mm_data, ref_point_ins);
      ins_data->gcj_position =
          util::Geo::EnuToGcj02(new_ins_real->enu_position, ref_point_ins);
      ins_data->enu_q = new_ins_real->enu_q;
    }
  }

  ins_buffer_->push_new_message(ins.header().data_stamp(), ins_data);
  last_ins = ins;
}

void FusionCenter::OnDr(const DeadReckoning& dr) {
  static auto last_dr = dr;
  if (dr.header().seq() <= last_dr.header().seq()) {
    HLOG_ERROR << "error, dr seq: " << dr.header().seq();
    return;
  }
  if (dr.header().data_stamp() <= last_dr.header().data_stamp()) {
    HLOG_ERROR << "error, dr data_stamp: " << dr.header().data_stamp();
    return;
  }
  if (std::isnan(dr.pose().pose_local().position().x()) ||
      std::isnan(dr.pose().pose_local().position().y()) ||
      std::isnan(dr.pose().pose_local().position().z()) ||
      std::isnan(dr.pose().pose_local().quaternion().w()) ||
      std::isnan(dr.pose().pose_local().quaternion().x()) ||
      std::isnan(dr.pose().pose_local().quaternion().y()) ||
      std::isnan(dr.pose().pose_local().quaternion().z())) {
    HLOG_ERROR << "dr data has nan";
    return;
  }
  if (Eigen::Quaterniond(dr.pose().pose_local().quaternion().w(),
                         dr.pose().pose_local().quaternion().x(),
                         dr.pose().pose_local().quaternion().y(),
                         dr.pose().pose_local().quaternion().z())
          .norm() < 1e-6) {
    HLOG_ERROR << "dr quaternion norm < 1e-6 ";
    return;
  }
  Dr::Ptr dr_data = std::make_shared<Dr>();
  dr_data->timestamp = dr.header().data_stamp();
  dr_data->dr_position.x() = dr.pose().pose_local().position().x();
  dr_data->dr_position.y() = dr.pose().pose_local().position().y();
  dr_data->dr_position.z() = dr.pose().pose_local().position().z();
  dr_data->dr_q.w() = dr.pose().pose_local().quaternion().w();
  dr_data->dr_q.x() = dr.pose().pose_local().quaternion().x();
  dr_data->dr_q.y() = dr.pose().pose_local().quaternion().y();
  dr_data->dr_q.z() = dr.pose().pose_local().quaternion().z();
  dr_buffer_->push_new_message(dr_data->timestamp, dr_data);
  last_dr = dr;
}

void FusionCenter::OnChassis(const Chassis& chassis) {
  static auto last_chassis = chassis;
  if (chassis.header().seq() <= last_chassis.header().seq()) {
    HLOG_WARN << "error, chassis seq: " << chassis.header().seq();
    return;
  }
  if (chassis.header().sensor_stamp().chassis_stamp() <=
      last_chassis.header().sensor_stamp().chassis_stamp()) {
    HLOG_WARN << "error, chassis sensor_stamp().chassis_stamp(): "
              << chassis.header().sensor_stamp().chassis_stamp();
    return;
  }
  if (std::isnan(chassis.wheel_speed().wheel_spd_rl()) ||
      std::isnan(chassis.wheel_speed().wheel_spd_rr())) {
    HLOG_WARN << "chassis data has nan";
    return;
  }

  Measure::Ptr chassis_data = std::make_shared<Measure>();
  chassis_data->timestamp = chassis.header().sensor_stamp().chassis_stamp();
  chassis_data->measure_type = CHASSIS;
  if (chassis.wheel_speed().wheel_direction_rl() ==
      hozon::soc::WheelSpeed_WheelSpeedType::
          WheelSpeed_WheelSpeedType_BACKWARD) {
    chassis_data->linear_velocity_vrf.x() =
        -(chassis.wheel_speed().wheel_spd_rl() +
          chassis.wheel_speed().wheel_spd_rr()) /
        2;
  } else {
    chassis_data->linear_velocity_vrf.x() =
        (chassis.wheel_speed().wheel_spd_rl() +
         chassis.wheel_speed().wheel_spd_rr()) /
        2;
  }

  chassis_data->linear_velocity_vrf.y() = 0;
  chassis_data->linear_velocity_vrf.z() = 0;
  chassis_buffer_->push_new_message(chassis_data->timestamp, chassis_data);

  last_chassis = chassis;
}

void FusionCenter::OnImu(const ImuIns& imu) {
  static auto last_imu = imu;
  if (imu.header().seq() <= last_imu.header().seq()) {
    HLOG_WARN << "error, imu seq: " << imu.header().seq();
    return;
  }
  if (imu.header().sensor_stamp().imuins_stamp() <=
      last_imu.header().sensor_stamp().imuins_stamp()) {
    HLOG_WARN << "error, imu sensor_stamp: "
              << imu.header().sensor_stamp().imuins_stamp();
    return;
  }
  if (std::isnan(imu.ins_info().augular_velocity().x()) ||
      std::isnan(imu.ins_info().augular_velocity().y()) ||
      std::isnan(imu.ins_info().augular_velocity().z()) ||
      std::isnan(imu.ins_info().linear_acceleration().x()) ||
      std::isnan(imu.ins_info().linear_acceleration().y()) ||
      std::isnan(imu.ins_info().linear_acceleration().z())) {
    HLOG_WARN << "imu data has nan";
    return;
  }

  Predict::Ptr imu_data = std::make_shared<Predict>();
  imu_data->timestamp = imu.header().sensor_stamp().imuins_stamp();
  imu_data->angular_velocity_vrf.x() = imu.ins_info().augular_velocity().x();
  imu_data->angular_velocity_vrf.y() = imu.ins_info().augular_velocity().y();
  imu_data->angular_velocity_vrf.z() = imu.ins_info().augular_velocity().z();
  imu_data->linear_acceleration_vrf.x() =
      imu.ins_info().linear_acceleration().x();
  imu_data->linear_acceleration_vrf.y() =
      imu.ins_info().linear_acceleration().y();
  imu_data->linear_acceleration_vrf.z() =
      imu.ins_info().linear_acceleration().z();
  imu_buffer_->push_new_message(imu.header().sensor_stamp().imuins_stamp(),
                                imu_data);
  last_imu = imu;
  {
    std::unique_lock<std::mutex> lock(imu_cv_mutex_);
    imu_cv_.notify_one();
  }
}

void FusionCenter::OnMm(const HafNodeInfo& mm) {
  if (!mm.valid_estimate()) {
    HLOG_WARN << "mm is not valid!";
    return;
  }
  static auto last_mm = mm;
  // if (mm.header().seq() <= last_mm.header().seq()) {
  //   HLOG_ERROR << "error, mm seq: " << mm.header().seq();
  //   return;
  // }
  if (mm.header().data_stamp() < last_mm.header().data_stamp()) {
    HLOG_ERROR << "error, mm data_stamp: " << mm.header().data_stamp();
    return;
  }

  if (std::isnan(mm.pos_gcj02().x()) || std::isnan(mm.pos_gcj02().x()) ||
      std::isnan(mm.pos_gcj02().x()) || std::isnan(mm.quaternion().w()) ||
      std::isnan(mm.quaternion().x()) || std::isnan(mm.quaternion().y()) ||
      std::isnan(mm.quaternion().z())) {
    HLOG_ERROR << "mm data has nan";
  }

  if (Eigen::Quaterniond(mm.quaternion().w(), mm.quaternion().x(),
                         mm.quaternion().y(), mm.quaternion().z())
          .norm() < 1e-6) {
    HLOG_ERROR << "mm quaternion norm < 1e-6 ";
    return;
  }

  Measure::Ptr mm_data = std::make_shared<Measure>();
  mm_data->timestamp = mm.header().data_stamp();
  mm_data->measure_type = MM;
  mm_data->gcj_position.x() = mm.pos_gcj02().x();
  mm_data->gcj_position.y() = mm.pos_gcj02().y();
  mm_data->gcj_position.z() = mm.pos_gcj02().z();
  mm_data->enu_q.w() = mm.quaternion().w();
  mm_data->enu_q.x() = mm.quaternion().x();
  mm_data->enu_q.y() = mm.quaternion().y();
  mm_data->enu_q.z() = mm.quaternion().z();
  mm_data->warn_info = mm.warn_info();

  // /*通过mm获得ins偏差估计的补偿量*/
  // Eigen::Vector3d ref_point_mm = ref_point.GetRefPoint();
  // if (ins_buffer_->is_empty()) {
  //   HLOG_WARN << "ins_buffer_->is_empty()";
  //   return;
  // }
  // /*临时使用解决mm时间戳不更新找不到ins的问题，后续mm里会解决*/
  // if (mm.header().data_stamp() == last_mm.header().data_stamp()) {
  //   HLOG_INFO << "mm时间戳不更新了";
  //   return;
  // }
  // Measure::ConstPtr latest_ins = GetInsByTimestamp(mm_data->timestamp);

  // // Measure::Ptr new_ins_really =
  // //     GetNewInsReal(latest_ins, mm_data, ref_point_mm);

  // // 测试有mm但rtk！＝４的低通滤波效果
  // Measure::Ptr new_ins_really =
  //     GetNewInsFilter(latest_ins, mm_data, ref_point_mm);

  // HLOG_INFO << "new_ins_really->enu_position:"
  //           << new_ins_really->enu_position.x() << ","
  //           << new_ins_really->enu_position.y() << ","
  //           << new_ins_really->enu_position.z();

  mm_buffer_->push_new_message(mm.header().data_stamp(), mm_data);

  last_mm = mm;
}

std::shared_ptr<Localization> FusionCenter::GetFcOutput() {
  fc_map_mutex_.lock();
  if (fc_map_.empty()) {
    HLOG_WARN << "fc is not inited";
    fc_map_mutex_.unlock();
    return nullptr;
  }
  auto fc_state = fc_map_.rbegin();
  fc_map_mutex_.unlock();

  // dr没初始化，不发fc
  if (fc_state->second.q_dr.norm() < 1e-6) {
    HLOG_WARN << "dr is not inited";
    return nullptr;
  }

  std::shared_ptr<Localization> fc_output = std::make_shared<Localization>();
  static int seq = 0;
  fc_output->mutable_header()->set_seq(seq++);
  fc_output->mutable_header()->set_data_stamp(fc_state->first);
  auto tp = std::chrono::time_point_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now());
  fc_output->mutable_header()->set_publish_stamp(
      static_cast<double>(tp.time_since_epoch().count()) * 1.0e-9);
  fc_output->mutable_pose()->mutable_gcj02()->set_x(
      fc_state->second.gcj_position.x());
  fc_output->mutable_pose()->mutable_gcj02()->set_y(
      fc_state->second.gcj_position.y());
  fc_output->mutable_pose()->mutable_gcj02()->set_z(
      fc_state->second.gcj_position.z());
  fc_output->mutable_pose()->mutable_linear_velocity()->set_x(
      fc_state->second.v.x());
  fc_output->mutable_pose()->mutable_linear_velocity()->set_y(
      fc_state->second.v.y());
  fc_output->mutable_pose()->mutable_linear_velocity()->set_z(
      fc_state->second.v.z());
  fc_output->mutable_pose()->mutable_quaternion()->set_w(
      static_cast<float>(fc_state->second.q.w()));
  fc_output->mutable_pose()->mutable_quaternion()->set_x(
      static_cast<float>(fc_state->second.q.x()));
  fc_output->mutable_pose()->mutable_quaternion()->set_y(
      static_cast<float>(fc_state->second.q.y()));
  fc_output->mutable_pose()->mutable_quaternion()->set_z(
      static_cast<float>(fc_state->second.q.z()));
  double heading = RotionMatrix2EulerAngle321(fc_state->second.q.matrix())[2];
  heading = 90 - heading / M_PI * 180;
  if (heading > 360) {
    heading -= 360;
  } else if (heading < 0) {
    heading += 360;
  }
  fc_output->mutable_pose()->set_heading(static_cast<float>(heading));
  fc_output->set_location_state(fc_state->second.state);
  fc_output->set_rtk_status(fc_state->second.rtk_state);
  fc_output->set_gps_sec(fc_state->second.second);
  fc_output->set_gps_week(fc_state->second.week);
  fc_output->mutable_pose()->mutable_linear_acceleration_raw_vrf()->set_x(
      fc_state->second.ins_sd_position);  // ins标准差
  fc_output->mutable_pose()->mutable_linear_acceleration_raw_vrf()->set_y(
      fc_state->second.ins_height);  // ins高度
  fc_output->mutable_pose()->mutable_linear_acceleration_raw_vrf()->set_z(
      fc_state->second.ins_heading);  // ins的heading
  for (int i = 0; i < 9; i++) {
    fc_output->add_covariance(
        static_cast<float>(fc_state->second.P(i, i) * 1e4));
  }
  // 线速度
  fc_output->mutable_pose()->mutable_linear_velocity_vrf()->set_x(
      fc_state->second.linear_velocity_vrf.x());  // 规控需要
  fc_output->mutable_pose()->mutable_linear_velocity_vrf()->set_y(
      fc_state->second.linear_velocity_vrf.y());  // 规控需要
  fc_output->mutable_pose()->mutable_linear_velocity_vrf()->set_z(
      fc_state->second.linear_velocity_vrf.z());
  // 角速度
  fc_output->mutable_pose()->mutable_angular_velocity_vrf()->set_x(
      fc_state->second.angular_velocity_vrf.x());  // 规控需要
  fc_output->mutable_pose()->mutable_angular_velocity_vrf()->set_y(
      fc_state->second.angular_velocity_vrf.y());  // 规控需要
  fc_output->mutable_pose()->mutable_angular_velocity_vrf()->set_z(
      fc_state->second.angular_velocity_vrf.z());  // 规控需要
  // 线加速度
  fc_output->mutable_pose()->mutable_linear_acceleration_vrf()->set_x(
      fc_state->second.linear_acceleration_vrf.x());  // 规控需要
  fc_output->mutable_pose()->mutable_linear_acceleration_vrf()->set_y(
      fc_state->second.linear_acceleration_vrf.y());  // 规控需要
  fc_output->mutable_pose()->mutable_linear_acceleration_vrf()->set_z(
      fc_state->second.linear_acceleration_vrf.z());  // 规控需要
  // // dr位置
  // fc_output->mutable_pose_local()->mutable_position()->set_x(
  //     fc_state->second.p_dr.x());  // 规控需要
  // fc_output->mutable_pose_local()->mutable_position()->set_y(
  //     fc_state->second.p_dr.y());  // 规控需要
  // fc_output->mutable_pose_local()->mutable_position()->set_z(
  //     fc_state->second.p_dr.z());  // 规控需要
  // // dr姿态
  // fc_output->mutable_pose_local()->mutable_quaternion()->set_w(
  //     static_cast<float>(fc_state->second.q_dr.w()));
  // fc_output->mutable_pose_local()->mutable_quaternion()->set_x(
  //     static_cast<float>(fc_state->second.q_dr.x()));
  // fc_output->mutable_pose_local()->mutable_quaternion()->set_y(
  //     static_cast<float>(fc_state->second.q_dr.y()));
  // fc_output->mutable_pose_local()->mutable_quaternion()->set_z(
  //     static_cast<float>(fc_state->second.q_dr.z()));
  // fc_output->mutable_pose_local()->set_local_heading(
  //     static_cast<float>(fc_state->second.yaw_dr));  // 规控需要
  // fc_output->mutable_pose()->mutable_euler_angle()->set_x(
  //     fc_state->second.pitch_dr);  // 规控需要

  // 用于ins偏差估计可视化
  fc_output->mutable_pose_dr()->mutable_position()->set_x(
      fc_state->second.p_ins_estimate.x());
  fc_output->mutable_pose_dr()->mutable_position()->set_y(
      fc_state->second.p_ins_estimate.y());
  fc_output->mutable_pose_dr()->mutable_position()->set_z(
      fc_state->second.p_ins_estimate.z());

  fc_output->mutable_pose_dr()->mutable_quaternion()->set_w(
      static_cast<float>(fc_state->second.q_ins_estimate.w()));
  fc_output->mutable_pose_dr()->mutable_quaternion()->set_x(
      static_cast<float>(fc_state->second.q_ins_estimate.x()));
  fc_output->mutable_pose_dr()->mutable_quaternion()->set_y(
      static_cast<float>(fc_state->second.q_ins_estimate.y()));
  fc_output->mutable_pose_dr()->mutable_quaternion()->set_z(
      static_cast<float>(fc_state->second.q_ins_estimate.z()));

  return fc_output;
}

}  // namespace fc
}  // namespace loc
}  // namespace mp
}  // namespace hozon
