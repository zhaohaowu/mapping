/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： ins_fusion.cpp
 *   author     ： nihongjie
 *   date       ： 2023.09
 ******************************************************************************/
#include "modules/location/ins_fusion/lib/ins_fusion.h"

#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <Sophus/se3.hpp>
#include <boost/filesystem.hpp>

#include "modules/util/include/util/geo.h"
#include "modules/util/include/util/temp_log.h"
#include "util/rviz_agent/rviz_agent.h"

namespace hozon {
namespace mp {
namespace loc {

namespace hlu = hozon::mp::util;

const char kNewestInsOdom[] = "/ins/fusion";

InsFusion::~InsFusion() {
  monitor_ins_proc_run_ = false;
  monitor_ins_proc_.get();
}

InsInitStatus InsFusion::Init(const std::string& configfile) {
  boost::filesystem::path path(configfile);
  if (!boost::filesystem::exists(path)) {
    return InsInitStatus::CONFIG_NOT_FOUND;
  }
  LoadConfigParams(configfile);
  latest_origin_ins_.Clear();
  latest_inspva_data_.Clear();
  smoother_ = std::make_unique<Smoother>(
      config_.smooth_window_size, config_.smooth_gcj02_enu_east_diff_thr,
      config_.smooth_gcj02_enu_north_diff_thr,
      config_.smooth_gcj02_enu_norm_diff_thr, config_.smooth_momentum);
  if (config_.monitor_ins) {
    monitor_ins_proc_ = std::async(&InsFusion::ProcessMonitorIns, this);
    monitor_ins_proc_run_ = true;
  }
  if (config_.use_rviz_bridge) {
    int ret = util::RvizAgent::Instance().Register<adsfi_proto::viz::Odometry>(
        kNewestInsOdom);
    if (ret < 0) {
      std::cout << "not found:" << kNewestInsOdom << std::endl;
    }
  }
  return InsInitStatus::OK;
}

void InsFusion::LoadConfigParams(const std::string& configfile) {
  YAML::Node config_parser = YAML::LoadFile(configfile);
  config_.smooth = config_parser["smooth"].as<bool>();
  config_.use_rviz_bridge = config_parser["use_rviz_bridge"].as<bool>();
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
  config_.monitor_ins = config_parser["monitor_ins"].as<bool>();
  config_.monitor_ins_deque_max_size =
      config_parser["monitor_ins_deque_max_size"].as<uint32_t>();
  config_.monitor_ins_sleep_time =
      config_parser["monitor_ins_sleep_time"].as<double>();
  config_.monitor_ins_loss_frame_min_time =
      config_parser["monitor_ins_loss_frame_min_time"].as<double>();
  config_.monitor_ins_loss_frame_max_time =
      config_parser["monitor_ins_loss_frame_max_time"].as<double>();
  config_.monitor_ins_useless_time =
      config_parser["monitor_ins_useless_time"].as<double>();
}

double InsFusion::ToSeconds(const uint32_t& sec, const uint32_t& nsec) {
  return static_cast<double>(sec) + static_cast<double>(nsec) * 1e-9;
}

void InsFusion::OnOriginIns(
    const adsfi_proto::hz_Adsfi::AlgInsInfo& origin_ins) {
  if (!origin_ins.is_valid()) {
    return;
  }
  std::unique_lock<std::mutex> lock(origin_ins_mutex_);
  if (origin_ins.header().seq() <= latest_origin_ins_.header().seq()) {
    return;
  }

  InsNode ins84_node;
  if (!Extract84InsNode(origin_ins, &ins84_node)) {
    return;
  }
  {
    std::unique_lock<std::mutex> lock(ins84_deque_mutex_);

    last_timestamp_ = std::chrono::steady_clock::now();
    ins84_deque_.emplace_back(ins84_node);
    while (ins84_deque_.size() > config_.ins84_deque_max_size) {
      ins84_deque_.pop_front();
    }
  }
  ins_state_enum_ = InsStateEnum::NORMAL;
  latest_origin_ins_ = origin_ins;
}

void InsFusion::OnInspva(const adsfi_proto::internal::HafNodeInfo& inspva) {
  if (!inspva.is_valid()) {
    return;
  }
  std::unique_lock<std::mutex> lock(inspva_mutex_);
  if (inspva.header().seq() <= latest_inspva_data_.header().seq()) {
    return;
  }
  if (!ref_init_) {
    Eigen::Vector3d blh(inspva.pos_gcj02().x(), inspva.pos_gcj02().y(),
                        inspva.pos_gcj02().z());
    SetRefpoint(blh);
    ref_init_ = true;
  }

  AccumulateGpsStatus(inspva);

  // keep main structure of inspva for passthrough
  latest_inspva_data_ = inspva;

  curr_node_.Reset();
  if (!Extract02InsNode(inspva, &curr_node_)) {
    return;
  }

  if (config_.fix_deflection_repeat &&
      FixDeflectionRepeat(last_node_, &curr_node_)) {
    HLOG_INFO << SETPRECISION(15)
              << "fix deflection repeat succ. last.tick:" << last_node_.ticktime
              << ", curr.tick:" << curr_node_.ticktime;
  }
  bool flag = false;
  if (config_.smooth && SmoothProc(&curr_node_)) {
    flag = true;
  }

  latest_inspva_data_.mutable_pos_gcj02()->set_x(curr_node_.blh(0));
  latest_inspva_data_.mutable_pos_gcj02()->set_y(curr_node_.blh(1));
  {
    std::unique_lock<std::mutex> lock2(inspva_deque_mutex_);
    auto inspva_data = latest_inspva_data_;
    if (flag) {
      auto inspva_data_ticktime = curr_node_.ticktime;
      inspva_data.mutable_header()->mutable_timestamp()->set_sec(
          static_cast<uint32_t>(inspva_data_ticktime));
      uint32_t nsec = static_cast<uint32_t>(
          (inspva_data_ticktime - static_cast<int>(inspva_data_ticktime)) *
          1e9);
      inspva_data.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    }
    inspva_deque_.emplace_back(inspva_data);
    while (inspva_deque_.size() > config_.monitor_ins_deque_max_size) {
      inspva_deque_.pop_front();
    }
    last_timestamp_ = std::chrono::steady_clock::now();
  }

  last_node_ = curr_node_;
  if (config_.use_rviz_bridge) {
    PublishTopic();
  }
}

bool InsFusion::GetResult(adsfi_proto::internal::HafNodeInfo* const node_info) {
  if (!ref_init_ || !node_info) {
    return false;
  }

  node_info->Clear();
  {
    std::unique_lock<std::mutex> lock(inspva_mutex_);
    *node_info = latest_inspva_data_;
  }
  node_info->set_type(adsfi_proto::internal::HafNodeInfo_NodeType_INS);
  node_info->mutable_header()->set_frameid("ins_fusion");
  if (ins_state_enum_ == InsStateEnum::MILD) {
    if (inspva_deque_.empty()) {
      return false;
    }
    auto inspva_data = inspva_deque_.back();
    auto inspva_data_ticktime =
        inspva_data.header().timestamp().sec() +
        inspva_data.header().timestamp().nsec() * 1.0e-9;
    node_info->mutable_pos_wgs()->set_x(inspva_data.mutable_pos_wgs()->x());
    node_info->mutable_pos_wgs()->set_y(inspva_data.mutable_pos_wgs()->y());
    node_info->mutable_pos_wgs()->set_z(inspva_data.mutable_pos_wgs()->z());
    node_info->mutable_gyro_bias()->set_x(inspva_data.mutable_gyro_bias()->x());
    node_info->mutable_gyro_bias()->set_y(inspva_data.mutable_gyro_bias()->y());
    node_info->mutable_gyro_bias()->set_z(inspva_data.mutable_gyro_bias()->z());
    node_info->mutable_accel_bias()->set_x(
        inspva_data.mutable_accel_bias()->x());
    node_info->mutable_accel_bias()->set_y(
        inspva_data.mutable_accel_bias()->y());
    node_info->mutable_accel_bias()->set_z(
        inspva_data.mutable_accel_bias()->z());
    return true;
  }

  adsfi_proto::hz_Adsfi::AlgInsInfo origin_ins;
  {
    std::unique_lock<std::mutex> lock(origin_ins_mutex_);
    origin_ins = latest_origin_ins_;
  }

  node_info->mutable_pos_wgs()->set_x(origin_ins.latitude());
  node_info->mutable_pos_wgs()->set_y(origin_ins.longitude());
  node_info->mutable_pos_wgs()->set_z(origin_ins.altitude());
  node_info->mutable_gyro_bias()->set_x(origin_ins.gyo_bias().x());
  node_info->mutable_gyro_bias()->set_y(origin_ins.gyo_bias().y());
  node_info->mutable_gyro_bias()->set_z(origin_ins.gyo_bias().z());
  node_info->mutable_accel_bias()->set_x(origin_ins.acc_bias().x());
  node_info->mutable_accel_bias()->set_y(origin_ins.acc_bias().y());
  node_info->mutable_accel_bias()->set_z(origin_ins.acc_bias().z());

  return true;
}

bool InsFusion::GetLocalizationEstimate(
    hozon::localization::LocalizationEstimate* const estimate) {
  if (!ref_init_ || !estimate) {
    return false;
  }
  estimate->Clear();
  return true;
}

void InsFusion::SetRefpoint(const Eigen::Vector3d& blh) { refpoint_ = blh; }

Eigen::Vector3d InsFusion::GetRefpoint() const { return refpoint_; }

void InsFusion::AccumulateGpsStatus(
    const adsfi_proto::internal::HafNodeInfo& inspva) {
  if (inspva.header().timestamp().sec() < 1000 ||
      latest_inspva_data_.header().timestamp().sec() < 1000) {
    return;
  }

  double curr_tick = ToSeconds(inspva.header().timestamp().sec(),
                               inspva.header().timestamp().nsec());
  double last_tick = ToSeconds(latest_inspva_data_.header().timestamp().sec(),
                               latest_inspva_data_.header().timestamp().nsec());
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

bool InsFusion::Extract02InsNode(
    const adsfi_proto::internal::HafNodeInfo& inspva, InsNode* const node) {
  if (!node || !ref_init_) {
    return false;
  }

  Eigen::Quaterniond q(inspva.quaternion().w(), inspva.quaternion().x(),
                       inspva.quaternion().y(), inspva.quaternion().z());
  if (q.norm() < 1e-10) {
    return false;
  }

  node->seq = inspva.header().seq();
  node->ticktime = ToSeconds(inspva.header().timestamp().sec(),
                             inspva.header().timestamp().nsec());

  node->refpoint = GetRefpoint();
  node->blh << inspva.pos_gcj02().x(), inspva.pos_gcj02().y(),
      inspva.pos_gcj02().z();
  node->org_blh = node->blh;
  node->enu = hlu::Geo::BlhToEnu(node->blh, node->refpoint);
  node->orientation = Sophus::SO3d(q).log();
  node->velocity << inspva.linear_velocity().x(), inspva.linear_velocity().y(),
      inspva.linear_velocity().z();
  node->linear_accel << inspva.linear_acceleration().x(),
      inspva.linear_acceleration().y(), inspva.linear_acceleration().z();
  return true;
}

bool InsFusion::Extract84InsNode(const adsfi_proto::hz_Adsfi::AlgInsInfo& ins,
                                 InsNode* const node) {
  if (!node || !ref_init_) {
    return false;
  }

  node->seq = ins.header().seq();
  node->ticktime = ToSeconds(ins.header().timestamp().sec(),
                             ins.header().timestamp().nsec());
  node->refpoint = GetRefpoint();
  node->blh << ins.latitude(), ins.longitude(), ins.altitude();
  node->enu = hlu::Geo::BlhToEnu(node->blh, node->refpoint);
  node->orientation << ins.attitude().x(), ins.attitude().y(),
      ins.attitude().z();
  node->velocity << ins.linear_velocity().x(), ins.linear_velocity().y(),
      ins.linear_velocity().z();
  node->linear_accel << ins.linear_acceleration().x(),
      ins.linear_acceleration().y(), ins.linear_acceleration().z();
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
  curr_node->blh = hlu::Geo::EnuToBlh(curr_node->enu, curr_node->refpoint);

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
  wgs84->blh = hlu::Geo::EnuToBlh(wgs84->enu, wgs84->refpoint);

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
  const auto pbv_02_enu = hlu::Geo::BlhToEnu(
      hlu::Geo::Wgs84ToGcj02(wgs84_node.blh), wgs84_node.refpoint);
  const auto china_enu = node->enu;
  const auto err_enu = china_enu - pbv_02_enu;

  smoother_->SetSmoothInputData(err_enu, pbv_02_enu);
  const auto smooth_err_enu = smoother_->GetSmoothOutputData();
  node->enu = pbv_02_enu + smooth_err_enu;
  node->blh = hlu::Geo::EnuToBlh(node->enu, node->refpoint);

  return true;
}

void InsFusion::ProcessMonitorIns() {
  pthread_setname_np(pthread_self(), "loc_monitor_ins");
  while (monitor_ins_proc_run_) {
    if (!config_.monitor_ins) {
      usleep(config_.monitor_ins_useless_time);
      continue;
    }

    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
                        end - last_timestamp_)
                        .count();
    // 微秒单位
    auto cost_time = duration * 0.000001;
    if (cost_time < config_.monitor_ins_loss_frame_min_time) {
      ins_state_enum_ = InsStateEnum::NORMAL;
      loss_ins_frame_id_ = 0;
      usleep(config_.monitor_ins_sleep_time);
      continue;
    }

    if (cost_time >= config_.monitor_ins_loss_frame_max_time) {
      ins_state_enum_ = InsStateEnum::SERIOUSLY;
      usleep(config_.monitor_ins_sleep_time);
      continue;
    }

    auto j =
        static_cast<int>(cost_time / config_.monitor_ins_loss_frame_max_time);
    if (j <= 0 || j == loss_ins_frame_id_) {
      usleep(config_.monitor_ins_sleep_time);
      continue;
    }

    ++loss_ins_frame_id_;
    ins_state_enum_ = InsStateEnum::MILD;
    if (ins_state_enum_ != InsStateEnum::MILD) {
      usleep(2 * config_.monitor_ins_sleep_time);
      continue;
    }

    auto start = std::chrono::steady_clock::now();
    std::unique_lock<std::mutex> lock(inspva_deque_mutex_);
    if (inspva_deque_.empty()) {
      continue;
    }

    auto inspva_data = inspva_deque_.back();
    auto current_inspva_data_ticktime =
        inspva_data.mutable_header()->timestamp().sec() +
        inspva_data.mutable_header()->timestamp().nsec() * 1.0e-9;
    auto inspva_data_ticktime = current_inspva_data_ticktime + cost_time / j;
    inspva_data.mutable_header()->mutable_timestamp()->set_sec(
        static_cast<uint32_t>(inspva_data_ticktime));
    uint32_t nsec = static_cast<uint32_t>(
        inspva_data_ticktime - static_cast<int>(inspva_data_ticktime) * 1e9);
    inspva_data.mutable_header()->mutable_timestamp()->set_nsec(nsec);
    Eigen::Vector3d velocity(inspva_data.linear_velocity().x(),
                             inspva_data.linear_velocity().y(),
                             inspva_data.linear_velocity().z());
    auto distance = velocity * cost_time / j;
    Eigen::Vector3d blh(inspva_data.mutable_pos_wgs()->x(),
                        inspva_data.mutable_pos_wgs()->y(),
                        inspva_data.mutable_pos_wgs()->z());
    auto refpoint = GetRefpoint();
    auto enu = hlu::Geo::BlhToEnu(blh, refpoint) + distance;
    blh = hlu::Geo::EnuToBlh(enu, refpoint);
    inspva_data.mutable_pos_wgs()->set_x(blh[0]);
    inspva_data.mutable_pos_wgs()->set_y(blh[1]);
    inspva_data.mutable_pos_wgs()->set_z(blh[2]);
    inspva_data.mutable_header()->set_seq(inspva_data.mutable_header()->seq() +
                                          1);

    blh << inspva_data.pos_gcj02().x(), inspva_data.pos_gcj02().y(),
        inspva_data.pos_gcj02().z();
    auto enu2 = hlu::Geo::BlhToEnu(blh, refpoint) + distance;
    blh = hlu::Geo::EnuToBlh(enu2, refpoint);
    inspva_data.mutable_pos_gcj02()->set_x(blh[0]);
    inspva_data.mutable_pos_gcj02()->set_y(blh[1]);
    inspva_data.mutable_pos_gcj02()->set_z(blh[2]);
    inspva_deque_.emplace_back(inspva_data);
    end = std::chrono::steady_clock::now();
    duration =
        std::chrono::duration_cast<std::chrono::microseconds>(end - start)
            .count();
    cost_time = duration * 0.000001;
    usleep(2 * config_.monitor_ins_sleep_time);
  }
}

bool InsFusion::PublishTopic() {
  if (!mp::util::RvizAgent::Instance().Ok()) {
    return false;
  }
  adsfi_proto::viz::Odometry odom;
  odom.mutable_header()->set_frameid("map");
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
}  // namespace loc
}  // namespace mp
}  // namespace hozon
