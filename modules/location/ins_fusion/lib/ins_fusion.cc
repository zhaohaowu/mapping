/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： ins_fusion.cpp
 *   author     ： nihongjie
 *   date       ： 2023.09
 ******************************************************************************/
#include "modules/location/ins_fusion/lib/ins_fusion.h"
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <Sophus/se3.hpp>
#include <boost/filesystem.hpp>

#include "modules/util/include/util/geo.h"
#include "modules/util/include/util/mapping_log.h"
#include "util/rviz_agent/rviz_agent.h"
#include "modules/util/include/util/orin_trigger_manager.h"


namespace hozon {
namespace mp {
namespace loc {

namespace hmu = hozon::mp::util;

const char kNewestInsOdom[] = "/ins/fusion";

InsFusion::~InsFusion() {}

InsInitStatus InsFusion::Init(const std::string& configfile) {
  boost::filesystem::path path(configfile);
  if (!boost::filesystem::exists(path)) {
    HLOG_ERROR << "Not found:" << configfile;
    return InsInitStatus::CONFIG_NOT_FOUND;
  }
  LoadConfigParams(configfile);
  latest_origin_ins_.Clear();
  latest_inspva_data_.Clear();
  smoother_ = std::make_unique<Smoother>(
      config_.smooth_window_size, config_.smooth_gcj02_enu_east_diff_thr,
      config_.smooth_gcj02_enu_north_diff_thr,
      config_.smooth_gcj02_enu_norm_diff_thr, config_.smooth_momentum);
  if (config_.use_rviz_bridge) {
    int ret = util::RvizAgent::Instance().Register<adsfi_proto::viz::Odometry>(
        kNewestInsOdom);
    if (ret < 0) {
      std::cout << "not found:" << kNewestInsOdom << std::endl;
    }
  }
  return InsInitStatus::OK;
}

Eigen::Matrix<double, 3, 1> Rot2Euler312(
    const Eigen::Matrix<double, 3, 3>& mat) {
  Eigen::Matrix<double, 3, 1> v;
  v(0) = asinf(mat(2, 1));
  v(1) = atan2f(-mat(2, 0), mat(2, 2));
  v(2) = atan2f(-mat(0, 1), mat(1, 1));
  return v;
}

void InsFusion::LoadConfigParams(const std::string& configfile) {
  YAML::Node config_parser = YAML::LoadFile(configfile);
  config_.smooth = config_parser["smooth"].as<bool>();
  config_.use_rviz_bridge = config_parser["use_rviz_bridge"].as<bool>();
  config_.use_inspva = config_parser["use_inspva"].as<bool>();
  config_.use_deflection = config_parser["use_deflection"].as<bool>();
  config_.use_fixed_quat = config_parser["use_fixed_quat"].as<bool>();
  config_.smooth_window_size =
      config_parser["smooth_window_size"].as<uint32_t>();
  config_.smooth_gcj02_enu_east_diff_thr =
      config_parser["smooth_gcj02_enu_east_diff_thr"].as<double>();
  config_.smooth_gcj02_enu_north_diff_thr =
      config_parser["smooth_gcj02_enu_north_diff_thr"].as<double>();
  config_.smooth_gcj02_enu_norm_diff_thr =
      config_parser["smooth_gcj02_enu_norm_diff_thr"].as<double>();
  config_.smooth_momentum = config_parser["smooth_momentum"].as<double>();

  config_.gps_1_last_thr = config_parser["gps_1_last_thr"].as<double>();
  config_.gps_2_last_thr = config_parser["gps_2_last_thr"].as<double>();
  config_.gps_3_last_thr = config_parser["gps_3_last_thr"].as<double>();
  config_.gps_5_last_thr = config_parser["gps_5_last_thr"].as<double>();

  config_.gps_1_last_with_mm_thr =
      config_parser["gps_1_last_with_mm_thr"].as<double>();
  config_.gps_2_last_with_mm_thr =
      config_parser["gps_2_last_with_mm_thr"].as<double>();
  config_.gps_3_last_with_mm_thr =
      config_parser["gps_3_last_with_mm_thr"].as<double>();
  config_.gps_5_last_with_mm_thr =
      config_parser["gps_5_last_with_mm_thr"].as<double>();

  config_.gps_5_stdx_thr = config_parser["gps_5_stdx_thr"].as<double>();
  config_.gps_5_stdy_thr = config_parser["gps_5_stdy_thr"].as<double>();
  config_.gps_5_stdz_thr = config_parser["gps_5_stdz_thr"].as<double>();

  config_.ins84_deque_max_size =
      config_parser["ins84_deque_max_size"].as<uint32_t>();
  config_.fix_deflection_repeat =
      config_parser["fix_deflection_repeat"].as<bool>();
}

bool InsFusion::OnOriginIns(const hozon::soc::ImuIns& origin_ins, hozon::localization::HafNodeInfo* const node_info) {
  InsNode ins84_node;
  static int ins_count = 0;

  if (origin_ins.header().seq() <= latest_origin_ins_.header().seq() ||
      origin_ins.header().sensor_stamp().imuins_stamp() <=
          latest_origin_ins_.header().sensor_stamp().imuins_stamp()) {
    HLOG_INFO << "origin ins has the same seq or stamp";
    return false;
  }

  if (!Extract84InsNode(origin_ins, &ins84_node)) {
    return false;
  }
  latest_origin_ins_ = origin_ins;

  // debug
  if (latest_origin_ins_.ins_info().gps_status() == 0) {
    HLOG_ERROR << "ins_seq:" << latest_origin_ins_.header().seq()
               << ", ins_ticktime:"
               << latest_origin_ins_.header().sensor_stamp().imuins_stamp()
               << " ,ins_gps_state:"
               << latest_origin_ins_.ins_info().gps_status()
               << " ,ins_linear_velocity:"
               << latest_origin_ins_.ins_info().linear_velocity().x() << " ,"
               << latest_origin_ins_.ins_info().linear_velocity().y() << " ,"
               << latest_origin_ins_.ins_info().linear_velocity().z();
  }
  {
    std::unique_lock<std::mutex> lock(ins84_deque_mutex_);
    ins84_deque_.emplace_back(ins84_node);
    while (ins84_deque_.size() > config_.ins84_deque_max_size) {
      ins84_deque_.pop_front();
    }
  }
  if (!config_.use_inspva) {
    last_node_ = ins84_node;
  } else {
    return false;
  }
  if (config_.use_rviz_bridge) {
    PublishTopic();
  }
  Convert(origin_ins, node_info);

  #ifdef ISORIN
    // mapping trigger imu/ins帧率异常 帧间差 > 50ms
    CheckTriggerInsTime(latest_origin_ins_);
  #endif
  ++ins_count;
  if (ins_count >= 100) {
    ins_count = 0;
    HLOG_ERROR << "rev origin ins heartbeat";
  }
  return true;
}

void InsFusion::CheckTriggerInsTime(const hozon::soc::ImuIns& cur_ins) {
  double curr_imu_time = cur_ins.header().sensor_stamp().imuins_stamp();
  static double last_imu_time = curr_imu_time;
  if (curr_imu_time - last_imu_time > 0.05) {
    HLOG_WARN << "last_imu_time: " << last_imu_time
              << " curr_imu_time: " << curr_imu_time
              << " Start to trigger dc 1008";
    GLOBAL_DC_TRIGGER.TriggerCollect(1008);
  }
  last_imu_time = curr_imu_time;
}

bool InsFusion::OnInspva(const hozon::localization::HafNodeInfo& inspva,
                         hozon::localization::HafNodeInfo* const node_info) {
  static int inspva_count = 0;
  if (!config_.use_inspva) {
    return false;
  }
  if (inspva.header().seq() <= latest_inspva_data_.header().seq() ||
      inspva.header().data_stamp() <=
          latest_inspva_data_.header().data_stamp()) {
    HLOG_INFO << "plugin ins has the same seq or stamp";
    return false;
  }
  if (!ref_init_) {
    Eigen::Vector3d blh(inspva.pos_gcj02().x(), inspva.pos_gcj02().y(),
                        inspva.pos_gcj02().z());
    SetRefpoint(blh);
    ref_init_ = true;
  }

  AccumulateGpsStatus(inspva);

  curr_node_.Reset();
  if (!Extract02InsNode(inspva, &curr_node_)) {
    return false;
  }
  inspva_node_is_valid_ = true;
  // keep main structure of inspva for passthrough
  latest_inspva_data_ = inspva;
  latest_inspva_data_.mutable_header()->set_data_stamp(
      latest_inspva_data_.header().data_stamp());

  if (config_.fix_deflection_repeat &&
      FixDeflectionRepeat(last_node_, &curr_node_)) {
    HLOG_INFO << "fix deflection repeat succ. last.tick:" << last_node_.ticktime
              << ", curr.tick:" << curr_node_.ticktime;
  }

  if (config_.smooth) {
    SmoothProc(&curr_node_);
  }
  last_node_ = curr_node_;
  if (config_.use_rviz_bridge) {
    PublishTopic();
  }
  *node_info = latest_inspva_data_;
  node_info->set_type(hozon::localization::HafNodeInfo_NodeType_INS);
  node_info->mutable_header()->set_frame_id("ins_fusion");
  ++inspva_count;
  if (inspva_count >= 100) {
    inspva_count = 0;
    HLOG_ERROR << "rev plugin ins heartbeat";
  }

  // 取出INS四元数并转heading
  Eigen::Quaterniond q(latest_inspva_data_.quaternion().w(),
                       latest_inspva_data_.quaternion().x(),
                       latest_inspva_data_.quaternion().y(),
                       latest_inspva_data_.quaternion().z());
  double heading = QuaternionToHeading(q);
  node_info->set_heading(static_cast<float>(heading));

  return true;
}

double InsFusion::QuaternionToHeading(const Eigen::Quaterniond& q) {
  Eigen::Vector3d orientation = Sophus::SO3d(q).log();
  const Sophus::SO3d& rot = Sophus::SO3d::exp(orientation);
  Eigen::Vector3d euler = Rot2Euler312(rot.matrix());
  euler = euler - ((euler.array() > M_PI).cast<double>() * 2.0 * M_PI).matrix();
  double heading = 90.0 - euler.z() / M_PI * 180;
  if (heading < 0.0) {
    heading += 360.0;
  }
  return heading;
}

void InsFusion::Convert(const hozon::soc::ImuIns& origin_ins, hozon::localization::HafNodeInfo* const node_info) {
  node_info->set_type(hozon::localization::HafNodeInfo_NodeType_INS);
  node_info->mutable_header()->set_seq(origin_ins.header().seq());
  node_info->mutable_header()->set_frame_id("ins_fusion");
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>
      tp = std::chrono::time_point_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now());
  node_info->mutable_header()->set_publish_stamp(tp.time_since_epoch().count() * 1.0e-9);
  node_info->mutable_header()->set_gnss_stamp(origin_ins.header().gnss_stamp());
  node_info->mutable_header()->set_data_stamp(
      origin_ins.header().sensor_stamp().imuins_stamp());
  node_info->mutable_header()->mutable_status()->set_error_code(
      origin_ins.header().status().error_code());
  node_info->mutable_header()->mutable_status()->set_msg(
      origin_ins.header().status().msg());
  node_info->set_is_valid(true);

  node_info->mutable_pos_wgs()->set_x(origin_ins.ins_info().latitude());
  node_info->mutable_pos_wgs()->set_y(origin_ins.ins_info().longitude());
  node_info->mutable_pos_wgs()->set_z(origin_ins.ins_info().altitude());

  node_info->mutable_attitude()->set_x(origin_ins.ins_info().attitude().x());
  node_info->mutable_attitude()->set_y(origin_ins.ins_info().attitude().y());
  node_info->mutable_attitude()->set_z(origin_ins.ins_info().attitude().z());

  node_info->mutable_linear_velocity()->set_x(
      origin_ins.ins_info().linear_velocity().x());
  node_info->mutable_linear_velocity()->set_y(
      origin_ins.ins_info().linear_velocity().y());
  node_info->mutable_linear_velocity()->set_z(
      origin_ins.ins_info().linear_velocity().z());

  node_info->mutable_gyro_bias()->set_x(
      origin_ins.offset_info().gyo_bias().x());
  node_info->mutable_gyro_bias()->set_y(
      origin_ins.offset_info().gyo_bias().y());
  node_info->mutable_gyro_bias()->set_z(
      origin_ins.offset_info().gyo_bias().z());

  node_info->mutable_accel_bias()->set_x(
      origin_ins.offset_info().acc_bias().x());
  node_info->mutable_accel_bias()->set_y(
      origin_ins.offset_info().acc_bias().y());
  node_info->mutable_accel_bias()->set_z(
      origin_ins.offset_info().acc_bias().z());

  node_info->mutable_sd_position()->set_x(
      origin_ins.ins_info().sd_position().x());
  node_info->mutable_sd_position()->set_y(
      origin_ins.ins_info().sd_position().y());
  node_info->mutable_sd_position()->set_z(
      origin_ins.ins_info().sd_position().z());

  node_info->mutable_sd_attitude()->set_x(
      origin_ins.ins_info().sd_attitude().x());
  node_info->mutable_sd_attitude()->set_y(
      origin_ins.ins_info().sd_attitude().y());
  node_info->mutable_sd_attitude()->set_z(
      origin_ins.ins_info().sd_attitude().z());

  node_info->mutable_sd_velocity()->set_x(
      origin_ins.ins_info().sd_velocity().x());
  node_info->mutable_sd_velocity()->set_y(
      origin_ins.ins_info().sd_velocity().y());
  node_info->mutable_sd_velocity()->set_z(
      origin_ins.ins_info().sd_velocity().z());

  node_info->mutable_angular_velocity()->set_x(
      origin_ins.ins_info().augular_velocity().x());
  node_info->mutable_angular_velocity()->set_y(
      origin_ins.ins_info().augular_velocity().y());
  node_info->mutable_angular_velocity()->set_z(
      origin_ins.ins_info().augular_velocity().z());

  node_info->mutable_linear_acceleration()->set_x(
      origin_ins.ins_info().linear_acceleration().x());
  node_info->mutable_linear_acceleration()->set_y(
      origin_ins.ins_info().linear_acceleration().y());
  node_info->mutable_linear_acceleration()->set_z(
      origin_ins.ins_info().linear_acceleration().z());

  node_info->mutable_mounting_error()->set_x(
      origin_ins.ins_info().mounting_error().x());
  node_info->mutable_mounting_error()->set_y(
      origin_ins.ins_info().mounting_error().y());
  node_info->mutable_mounting_error()->set_z(
      origin_ins.ins_info().mounting_error().z());

  node_info->set_heading(origin_ins.ins_info().heading());
  node_info->set_sys_status(origin_ins.ins_info().sys_status());
  node_info->set_gps_status(origin_ins.ins_info().gps_status());
  node_info->set_sensor_used(origin_ins.ins_info().sensor_used());
  node_info->set_wheel_velocity(origin_ins.ins_info().wheel_velocity());
  node_info->set_odo_sf(origin_ins.ins_info().odo_sf());
  node_info->set_valid_estimate(true);
  for (int i = 0; i < 36; ++i) {
    node_info->add_covariance(0.);
  }

  Eigen::Quaterniond quat;
  if (config_.use_fixed_quat) {
    node_info->mutable_quaternion()->set_x(0.0);
    node_info->mutable_quaternion()->set_y(0.0);
    node_info->mutable_quaternion()->set_z(0.0);
    node_info->mutable_quaternion()->set_w(1.0);
  } else {
    // 弧度 roll pitch yaw
    Eigen::Matrix<double, 3, 1> attitude = Eigen::MatrixXd::Zero(3, 1);
    attitude(0) = origin_ins.ins_info().attitude().x() * M_PI / 180.0;
    attitude(1) = origin_ins.ins_info().attitude().y() * M_PI / 180.0;
    attitude(2) = origin_ins.ins_info().attitude().z() * M_PI / 180.0;

    Eigen::AngleAxisd roll(attitude[0], Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch(attitude[1], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw(attitude[2], Eigen::Vector3d::UnitZ());
    quat = yaw * roll * pitch;
    node_info->mutable_quaternion()->set_x(quat.x());
    node_info->mutable_quaternion()->set_y(quat.y());
    node_info->mutable_quaternion()->set_z(quat.z());
    node_info->mutable_quaternion()->set_w(quat.w());
  }
  // 取出INS四元数并转heading
  double heading = QuaternionToHeading(quat);
  node_info->set_heading(heading);

  if (config_.use_deflection) {
    Eigen::Vector3d wgs84(origin_ins.ins_info().latitude(),
                          origin_ins.ins_info().longitude(),
                          origin_ins.ins_info().altitude());
    auto gcj02 = hmu::Geo::Wgs84ToGcj02(wgs84);
    node_info->mutable_pos_gcj02()->set_x(gcj02[0]);
    node_info->mutable_pos_gcj02()->set_y(gcj02[1]);
    node_info->mutable_pos_gcj02()->set_z(gcj02[2]);
  }
  node_info->set_gps_week(origin_ins.gps_week());
  node_info->set_gps_sec(origin_ins.gps_sec());
}

void InsFusion::SetRefpoint(const Eigen::Vector3d& blh) { refpoint_ = blh; }

Eigen::Vector3d InsFusion::GetRefpoint() const { return refpoint_; }

void InsFusion::AccumulateGpsStatus(
    const hozon::localization::HafNodeInfo& inspva) {
  if (static_cast<int>(inspva.header().publish_stamp()) < 1000 ||
      static_cast<int>(latest_inspva_data_.header().publish_stamp()) < 1000) {
    return;
  }

  double curr_tick = inspva.header().publish_stamp();
  double last_tick = latest_inspva_data_.header().publish_stamp();
  const double tick_diff = fabs(curr_tick - last_tick);

  InsGpsStatus gps_status{inspva.gps_status()};
  switch (gps_status) {
    case InsGpsStatus::SPPO:
      gps_1_last_ += tick_diff;
      gps_1_last_with_mm_ += tick_diff;
      break;
    case InsGpsStatus::RTD_PO:
      gps_2_last_ += tick_diff;
      gps_2_last_with_mm_ += tick_diff;
      break;
    case InsGpsStatus::COMB_CAL:
      gps_3_last_ += tick_diff;
      gps_3_last_with_mm_ += tick_diff;
      break;
    case InsGpsStatus::RTK_FLOAT_PO:
      if (inspva.sd_position().x() > config_.gps_5_stdx_thr ||
          inspva.sd_position().y() > config_.gps_5_stdy_thr ||
          inspva.sd_position().z() > config_.gps_5_stdz_thr) {
        gps_5_last_ += tick_diff;
        gps_5_last_with_mm_ += tick_diff;
      }
      break;
    default:
      gps_1_last_ = 0.0;
      gps_2_last_ = 0.0;
      gps_3_last_ = 0.0;
      gps_5_last_ = 0.0;
      gps_1_last_with_mm_ = 0.0;
      gps_2_last_with_mm_ = 0.0;
      gps_3_last_with_mm_ = 0.0;
      gps_5_last_with_mm_ = 0.0;
      break;
  }
}

bool InsFusion::Extract02InsNode(const hozon::localization::HafNodeInfo& inspva,
                                 InsNode* const node) {
  if (!node || !ref_init_) {
    return false;
  }

  const auto& mq = inspva.quaternion();
  if (std::isnan(mq.w()) || std::isnan(mq.x()) || std::isnan(mq.y()) || std::isnan(mq.z())) {
    HLOG_WARN << "Inspva_quaternion is nan";
    return false;
  }

  Eigen::Quaterniond q(inspva.quaternion().w(), inspva.quaternion().x(),
                       inspva.quaternion().y(), inspva.quaternion().z());
  if (q.norm() < 1e-10) {
    HLOG_WARN << "Inspva_fusion_HafNodeInfo quaternion(w,x,y,z) "
              << q.w() << "," << q.x() << ","
              << q.y() << "," << q.z();
    return false;
  }

  if (std::fabs(q.norm()-1) > 1e-3) {
    HLOG_WARN << "Inspva_fusion_HafNodeInfo quaternion(w,x,y,z) "
              << q.w() << "," << q.x() << ","
              << q.y() << "," << q.z();
    return false;
  }

  node->seq = inspva.header().seq();
  node->ticktime = inspva.header().data_stamp();

  node->refpoint = GetRefpoint();
  node->blh << inspva.pos_gcj02().x(), inspva.pos_gcj02().y(),
      inspva.pos_gcj02().z();
  node->org_blh = node->blh;
  node->enu = hmu::Geo::BlhToEnu(node->blh, node->refpoint);
  node->orientation = Sophus::SO3d(q).log();
  node->velocity << inspva.linear_velocity().x(), inspva.linear_velocity().y(),
      inspva.linear_velocity().z();
  node->linear_accel << inspva.linear_acceleration().x(),
      inspva.linear_acceleration().y(), inspva.linear_acceleration().z();
  return true;
}

bool InsFusion::Extract84InsNode(const hozon::soc::ImuIns& ins,
                                 InsNode* const node) {
  if (config_.use_inspva && !ref_init_) {
    return false;
  }
  Eigen::Vector3d q(ins.ins_info().attitude().x(),
                    ins.ins_info().attitude().y(),
                    ins.ins_info().attitude().z());
  if (!node || q.norm() < 1e-10) {
    return false;
  }

  node->seq = ins.header().seq();
  node->ticktime = ins.header().sensor_stamp().imuins_stamp();
  node->refpoint = GetRefpoint();
  node->blh << ins.ins_info().latitude(), ins.ins_info().longitude(),
      ins.ins_info().altitude();
  node->enu = hmu::Geo::BlhToEnu(node->blh, node->refpoint);
  node->orientation << ins.ins_info().attitude().x(),
      ins.ins_info().attitude().y(), ins.ins_info().attitude().z();
  node->velocity << ins.ins_info().linear_velocity().x(),
      ins.ins_info().linear_velocity().y(),
      ins.ins_info().linear_velocity().z();
  node->linear_accel << ins.ins_info().linear_acceleration().x(),
      ins.ins_info().linear_acceleration().y(),
      ins.ins_info().linear_acceleration().z();
  return true;
}

bool InsFusion::CoordSameInPlanar(const Eigen::Vector3d& c1,
                                  const Eigen::Vector3d& c2) {
  if (fabs(c1(0) - c2(0)) < 1e-9 && fabs(c1(1) - c2(1)) < 1e-9) {
    return true;
  }
  return false;
}

Eigen::Vector3d InsFusion::PredictEN(const double& t,
                                     const InsNode& node) const {
  const auto& rot = Sophus::SO3d::exp(node.orientation);
  const auto& enu_vel = node.velocity;
  const auto& enu_acc = rot * (node.linear_accel * 9.8);
  const Eigen::Vector3d en(
      node.enu(0) + enu_vel(0) * t + 0.5 * enu_acc(0) * t * t,
      node.enu(1) + enu_vel(1) * t + 0.5 * enu_acc(1) * t * t, node.enu(2));
  return en;
}

bool InsFusion::FixDeflectionRepeat(const InsNode& prev_node,
                                    InsNode* const curr_node) {
  if (!curr_node) {
    return false;
  }

  const double t = fabs(curr_node->ticktime - prev_node.ticktime);
  if (prev_node.ticktime < 0 || curr_node->ticktime < 0 || t < 1e-6 ||
      prev_node.velocity.norm() < 0.1 ||
      !CoordSameInPlanar(prev_node.org_blh, curr_node->org_blh)) {
    return false;
  }

  curr_node->enu = PredictEN(t, prev_node);
  curr_node->blh = hmu::Geo::EnuToBlh(curr_node->enu, curr_node->refpoint);

  return true;
}

bool InsFusion::SimultaneousWgs84With02(const InsNode& gcj02,
                                        InsNode* const wgs84) {
  if (!wgs84) {
    return false;
  }
  if (gcj02.seq <= 0 || gcj02.ticktime < 0) {
    return false;
  }

  InsNode refer_node;
  {
    std::unique_lock<std::mutex> lock(ins84_deque_mutex_);
    if (ins84_deque_.empty() ||
        ins84_deque_.front().ticktime > gcj02.ticktime) {
      return false;
    }

    for (auto it = ins84_deque_.rbegin(); it != ins84_deque_.rend(); ++it) {
      if (fabs((*it).ticktime - gcj02.ticktime) < 1e-3) {
        *wgs84 = *it;
        return true;
      }
      if ((*it).ticktime < gcj02.ticktime) {
        refer_node = *it;
        break;
      }
    }
  }

  if (refer_node.ticktime < 0) {
    return false;
  }

  *wgs84 = refer_node;
  wgs84->ticktime = gcj02.ticktime;

  const double t = gcj02.ticktime - refer_node.ticktime;
  wgs84->enu = PredictEN(t, refer_node);
  wgs84->blh = hmu::Geo::EnuToBlh(wgs84->enu, wgs84->refpoint);

  return true;
}

bool InsFusion::SmoothProc(InsNode* const node) {
  if (!node || node->ticktime < 0 || !ref_init_) {
    return false;
  }

  InsNode wgs84_node;
  if (!SimultaneousWgs84With02(*node, &wgs84_node)) {
    return false;
  }
  if (wgs84_node.seq <= 0 || wgs84_node.ticktime <= 0) {
    return false;
  }

  // public version deflection
  const auto pbv_02_enu = hmu::Geo::BlhToEnu(
      hmu::Geo::Wgs84ToGcj02(wgs84_node.blh), wgs84_node.refpoint);
  const auto china_enu = node->enu;
  const auto err_enu = china_enu - pbv_02_enu;

  smoother_->SetSmoothInputData(err_enu, pbv_02_enu);
  const auto smooth_err_enu = smoother_->GetSmoothOutputData();
  node->enu = pbv_02_enu + smooth_err_enu;
  node->blh = hmu::Geo::EnuToBlh(node->enu, node->refpoint);

  return true;
}

bool InsFusion::PublishTopic() {
  if (!mp::util::RvizAgent::Instance().Ok()) {
    return false;
  }
  adsfi_proto::viz::Odometry odom;
  odom.mutable_header()->set_frameid("ins_fusion");
  uint64_t sec = uint64_t(last_node_.ticktime);
  uint64_t nsec = uint64_t((last_node_.ticktime - sec) * 1e9);
  odom.mutable_header()->mutable_timestamp()->set_sec(sec);
  odom.mutable_header()->mutable_timestamp()->set_nsec(nsec);
  odom.mutable_pose()->mutable_pose()->mutable_position()->set_x(
      last_node_.enu(0));
  odom.mutable_pose()->mutable_pose()->mutable_position()->set_y(
      last_node_.enu(1));
  odom.mutable_pose()->mutable_pose()->mutable_position()->set_z(0);

  odom.mutable_pose()->mutable_pose()->mutable_orientation()->set_x(
      last_node_.orientation.x());
  odom.mutable_pose()->mutable_pose()->mutable_orientation()->set_y(
      last_node_.orientation.y());
  odom.mutable_pose()->mutable_pose()->mutable_orientation()->set_z(
      last_node_.orientation.z());
  for (int i = 0; i < 36; ++i) {
    odom.mutable_pose()->add_covariance(0.);
  }
  mp::util::RvizAgent::Instance().Publish(kNewestInsOdom, odom);
  return true;
}

bool InsFusion::InsFusionState() { return config_.use_inspva; }
}  // namespace loc
}  // namespace mp
}  // namespace hozon
