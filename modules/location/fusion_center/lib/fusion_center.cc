/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： fusion_center.cc
 *   author     ： lilanxing
 *   date       ： 2023.09
 ******************************************************************************/

#include "modules/location/fusion_center/lib/fusion_center.h"
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>

#include <boost/filesystem.hpp>

#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Geometry/Transform.h"
#include "modules/location/common/data_verify.h"
#include "modules/location/fusion_center/lib/defines.h"
#include "modules/location/fusion_center/lib/eulerangle.h"
#include "modules/map_fusion/include/map_fusion/map_service/global_hd_map.h"
#include "modules/util/include/util/geo.h"
#include "modules/util/include/util/mapping_log.h"
#include "modules/util/include/util/orin_trigger_manager.h"
#include "onboard/onboard_lite/map_fusion/map_fusion_config_lite.h"

namespace hozon {
namespace mp {
namespace loc {
namespace fc {

namespace hmu = hozon::mp::util;

FusionCenter::~FusionCenter() {
  fusion_run_ = false;
  if (th_fusion_->joinable()) {
    th_fusion_->join();
  }

  if (params_.use_debug_txt) {
    if (fc_ins_diff_file_.is_open()) {
      fc_ins_diff_file_.close();
    }

    if (mm_ins_diff_file_.is_open()) {
      mm_ins_diff_file_.close();
    }
  }
}

bool FusionCenter::Init(const std::string& configfile,
                        const std::string& filterconf,
                        const std::string& eskfconf,
                        const std::string& monitorconf) {
  boost::filesystem::path path(configfile);
  if (!boost::filesystem::exists(path)) {
    HLOG_ERROR << "location fc conf:" << configfile << " not exist";
    return false;
  }
  if (!LoadParams(configfile)) {
    HLOG_ERROR << "location fc load params from " << configfile << " error";
    return false;
  }

  if (!kalman_filter_.Init(filterconf)) {
    HLOG_WARN << "filter init failed: " << filterconf;
    return false;
  }

  eskf_ = std::make_shared<ESKF>();
  if (!eskf_->Init(eskfconf)) {
    HLOG_WARN << "filter init failed: " << eskfconf;
    return false;
  }

  monitor_ = std::make_shared<Monitor>();
  if (!monitor_->Init(monitorconf)) {
    HLOG_WARN << "monitor init failed: " << monitorconf;
    return false;
  }

  // debug print enu and euler diff
  if (params_.use_debug_txt) {
    fc_ins_diff_file_.open("./fc_ins_diff.txt", std::ios::trunc);
    mm_ins_diff_file_.open("./mm_ins_diff.txt", std::ios::trunc);
  }

  fusion_run_ = true;
  ref_init_ = false;
  th_fusion_ = std::make_shared<std::thread>(&FusionCenter::RunFusion, this);

  return true;
}

void FusionCenter::OnImu(const ImuIns& imuins) {
  if (!params_.recv_imu) {
    return;
  }
  // seq not used on orin
  if (imuins.header().seq() == prev_imuins_.header().seq()) {
    return;
  }

  std::unique_lock<std::mutex> lock(imuins_deque_mutex_);
  imuins_deque_.emplace_back(std::make_shared<ImuIns>(imuins));
  ShrinkQueue(&imuins_deque_, params_.imu_deque_max_size);
  prev_imuins_ = imuins;
  curr_imuins_ = imuins;
}

void FusionCenter::OnIns(const HafNodeInfo& ins) {
  if (!params_.recv_ins || !ins.valid_estimate()) {
    HLOG_WARN << "ins_seq is not valid!";
    return;
  }

  {
    std::unique_lock<std::mutex> lock(latest_ins_mutex_);
    if (ins.header().seq() == latest_ins_data_.header().seq()) {
      HLOG_WARN << "ins seq is same!" << ins.header().seq();
      return;
    }
    latest_ins_data_ = ins;
    // debug
    if (ins.gps_status() == 0) {
      HLOG_WARN << "ins_seq:" << ins.header().seq()
                << ", ins_ticktime:" << ins.header().data_stamp()
                << " ,ins_gps_state:" << ins.gps_status()
                << " ,ins_linear_velocity:" << ins.linear_velocity().x() << " ,"
                << ins.linear_velocity().y() << " ,"
                << ins.linear_velocity().z();
    }
  }

  if (!ref_init_) {
    const Eigen::Vector3d refpoint(ins.pos_gcj02().x(), ins.pos_gcj02().y(),
                                   ins.pos_gcj02().z());
    SetRefpoint(refpoint);
    ref_init_ = true;
  }

  Node node;
  node.type = NodeType::INS;
  if (!ExtractBasicInfo(ins, &node)) {
    HLOG_WARN << "ExtractBasicInfo ins fail";
    return;
  }
  // 标准差赋值
  node.cov(0, 0) = std::max(ins.sd_position().x(), ins.sd_position().y());
  // 随距离切refpoint
  if (node.enu.norm() > params_.refpoint_update_dist) {
    SetRefpoint(node.blh);
    monitor_->Clear();
    HLOG_INFO << node.ticktime << " refpoint change to " << node.blh(0) << ","
              << node.blh(1) << "," << node.blh(2);
  }

  {
    std::unique_lock<std::mutex> lock_ins_deque(ins_deque_mutex_);
    ins_deque_.emplace_back(std::make_shared<Node>(node));
    ShrinkQueue(&ins_deque_, params_.ins_deque_max_size);
  }

  if (init_dr_) {
    if (!init_dr_ins_) {
      init_dr_ins_ = true;
      init_ins_node_ = node;
    }
  }
  // 加入ins+偏差修正的观测队列,10hz频率加入
  if (params_.lateral_error_compensation && node.cov(0, 0) <= 0.08 &&
      node.rtk_status == 4 && ins_meas_cnt_ % 10 == 0) {
    auto ref_point = node.refpoint;
    auto node_enu = hmu::Geo::BlhToEnu(node.blh, ref_point);
    InsOffset offset;
    {
      std::unique_lock<std::mutex> lock_ins_offset(ins_offset_mutex_);
      offset = ins_offset_;
    }
    if (offset.init) {
      auto last_enu = hmu::Geo::BlhToEnu(offset.latest_ins_node.blh, ref_point);
      double last_estimate_dis = (node_enu - last_enu).norm();
      if (last_estimate_dis < 120 && offset.smooth_cnt >= 5) {
        auto compensate_pose = Node2Eigen(node) * offset.offset;
        node.blh = hmu::Geo::EnuToBlh(compensate_pose.translation(), ref_point);
        node.quaternion = Eigen::Quaterniond(compensate_pose.rotation());
        node.orientation = Sophus::SO3d(node.quaternion).log();
        node.enu = hmu::Geo::BlhToEnu(node.blh, node.refpoint);
      }
      node.type = NodeType::INS_MM;
      node.pe_cov_coef = node.cov(0, 0) / 0.02;
      node.pe_cov_coef = last_estimate_dis / 20;
      std::unique_lock<std::mutex> lock_ins_meas_deque(ins_meas_deque_mutex_);
      ins_meas_deque_.emplace_back(std::make_shared<Node>(node));
      ShrinkQueue(&ins_meas_deque_, 100);
      HLOG_INFO << "add ins meas";
    }
  }
  ins_meas_cnt_++;
}

void FusionCenter::OnDR(const hozon::dead_reckoning::DeadReckoning& dr) {
  if (!params_.recv_dr) {
    return;
  }
  // seq not used on orin
  if (dr.header().seq() <= prev_raw_dr_.header().seq()) {
    HLOG_ERROR << "error,seq:\n" << dr.header().seq();
    return;
  }
  if (dr.header().data_stamp() <= prev_raw_dr_.header().data_stamp()) {
    HLOG_ERROR << "error,data_stamp:\n" << dr.header().data_stamp();
    return;
  }
  prev_raw_dr_ = dr;

  Node node;
  node.type = NodeType::DR;
  if (!DrToBasicInfo(dr, &node)) {
    return;
  }
  std::unique_lock<std::mutex> lock(dr_deque_mutex_);
  dr_deque_.emplace_back(std::make_shared<Node>(node));
  ShrinkQueue(&dr_deque_, params_.dr_deque_max_size);
  if (!init_dr_ && dr_deque_.size() >= 3) {
    OnInitDR(*(dr_deque_.front()));
    init_dr_ = true;
  }
}

void FusionCenter::OnInitDR(const Node& initdr) { init_dr_node_ = initdr; }

void FusionCenter::OnPoseEstimate(const HafNodeInfo& pe) {
  if (!ref_init_ || !params_.recv_pe) {
    return;
  }

  static int count = 0;
  if (pe.header().data_stamp() == prev_raw_pe_.header().data_stamp()) {
    ++count;
    if (count >= 10) {
      count = 0;
      mm_is_valid_ = false;
    }
    HLOG_ERROR << "error OnPE,data_stamp:" << pe.header().data_stamp();
    return;
  }
  monitor_->OnPeFault(pe);

  if (!pe.valid_estimate()) {
    ++count;
    if (count >= 10) {
      count = 0;
      mm_is_valid_ = false;
    }
    return;
  }
  count = 0;
  mm_is_valid_ = true;

  Node node;
  node.type = NodeType::MM;
  if (!ExtractBasicInfo(pe, &node)) {
    return;
  }
  prev_raw_pe_ = pe;

  // 时间戳同步:使用将mm时间戳对齐至ins时间戳
  Node ins_node;
  {
    std::unique_lock<std::mutex> lock(ins_deque_mutex_);
    auto ins_it = ins_deque_.rbegin();
    for (; ins_it != ins_deque_.rend(); ++ins_it) {
      if ((*ins_it)->ticktime <= node.ticktime) {
        break;
      }
    }
    if (ins_it == ins_deque_.rend()) {
      HLOG_ERROR << "Not find ins node in INS deque,error!";
      return;
    } else if (ins_it == ins_deque_.rbegin()) {
      node.ticktime = (*ins_it)->ticktime;
      ins_node = *(*ins_it);
    } else {
      auto it_next = std::prev(ins_it);
      if (abs(node.ticktime - (*ins_it)->ticktime) <=
          abs(node.ticktime - (*it_next)->ticktime)) {
        node.ticktime = (*ins_it)->ticktime;
        ins_node = *(*ins_it);
      } else {
        node.ticktime = (*it_next)->ticktime;
        ins_node = *(*it_next);
      }
    }
  }

  node.state = FilterPoseEstimation(node);
  // 最新的有效的MM测量，用于判断是否加入INS测量
  if (node.state) {
    lastest_valid_pe_mutex_.lock();
    lastest_valid_pe_ = node;
    lastest_valid_pe_mutex_.unlock();
    filter_valid_pe_ = false;
  } else {
    filter_valid_pe_ = true;
  }

  {
    std::unique_lock<std::mutex> lock(pe_deque_mutex_);
    pe_deque_.emplace_back(std::make_shared<Node>(node));
    ShrinkQueue(&pe_deque_, params_.pe_deque_max_size);
  }

  // 计算ins与mm之间的偏差
  // mm不在路口内 并且 ins的标准差<0.05
  HLOG_INFO << "ins cov:" << ins_node.cov(0, 0) << "," << node.rtk_status;
  if (ins_node.rtk_status == 4 && ins_node.cov(0, 0) < 0.08 &&
      node.pe_cov_coef <= 1.0 && node.rtk_status == 0) {
    auto ref_point = Refpoint();
    node.enu = hmu::Geo::BlhToEnu(node.blh, ref_point);
    ins_node.enu = hmu::Geo::BlhToEnu(ins_node.blh, ref_point);
    auto ins2pe_offset = Node2Eigen(ins_node).inverse() * Node2Eigen((node));
    {
      std::unique_lock<std::mutex> lock_ins_offset(ins_offset_mutex_);
      if (ins_offset_.init) {
        // 计算上一次偏差地点和本次的差距
        auto last_enu =
            hmu::Geo::BlhToEnu(ins_offset_.latest_ins_node.blh, ref_point);
        // 距离太远 不平滑 直接重置
        if ((ins_node.enu - last_enu).norm() > 100.0) {
          ins_offset_.smooth_cnt = 0;
          ins_offset_.offset = ins2pe_offset;
          ins_offset_.latest_ins_node = ins_node;
          ins_offset_.latest_pe_node = node;
        } else {
          // 平滑
          ins_offset_.smooth_cnt++;
          Eigen::Quaterniond q_cur(ins2pe_offset.rotation());
          Eigen::Quaterniond q_last(ins_offset_.offset.rotation());
          Eigen::Quaterniond q = q_last.slerp(0.2, q_cur);
          Eigen::Vector3d t = 0.2 * ins_offset_.offset.translation() +
                              0.8 * ins2pe_offset.translation();
          ins_offset_.offset = Eigen::Isometry3d::Identity();
          ins_offset_.offset.linear() = q.toRotationMatrix();
          ins_offset_.offset.translation() = t;
          ins_offset_.latest_ins_node = ins_node;
          ins_offset_.latest_pe_node = node;
        }
      } else {
        ins_offset_.init = true;
        ins_offset_.smooth_cnt = 0;
        ins_offset_.offset = ins2pe_offset;
        ins_offset_.latest_ins_node = ins_node;
        ins_offset_.latest_pe_node = node;
      }
      HLOG_INFO << "ins offset:" << ins_offset_.smooth_cnt << ","
                << ins_offset_.offset.translation().x() << ","
                << ins_offset_.offset.translation().y();
    }
  }

  // debug print enu and euler diff
  if (params_.use_debug_txt) {
    std::unique_lock<std::mutex> lock(ins_deque_mutex_);
    Node ins_debug_node;
    for (const auto& ins_node : ins_deque_) {
      if (ins_node->ticktime - node.ticktime < 0.001) {
        ins_debug_node = *ins_node;
      }
    }
    if (ins_debug_node.ticktime > 0) {
      DebugDiffTxt(mm_ins_diff_file_, node, ins_debug_node);
    }
  }
}

void FusionCenter::SetEhpCounter(int32_t counter) { ehp_counter_ = counter; }

int32_t FusionCenter::GetEhpCounter() const { return ehp_counter_; }

bool FusionCenter::GetCurrentOutput(Localization* const location) {
  if (location == nullptr) {
    return false;
  }
  Context ctx;
  if (!init_dr_) {
    HLOG_ERROR << "OnInitDR failed";
    return false;
  }
  if (!GetCurrentContext(&ctx)) {
    HLOG_ERROR << "get context failed";
    return false;
  }

  Node2Localization(ctx, location);
  return true;
}

bool FusionCenter::GetCurrentContext(Context* const ctx) {
  if (ctx == nullptr) {
    return false;
  }

  ctx->imuins = curr_imuins_;
  {
    std::unique_lock<std::mutex> lock(latest_ins_mutex_);
    ctx->ins = latest_ins_data_;
  }
  {
    std::unique_lock<std::mutex> lock(ins_deque_mutex_);
    if (ins_deque_.empty()) {
      HLOG_ERROR << "ins_deque is empty";
    } else {
      ctx->ins_node = *(ins_deque_.back());
    }
  }
  {
    std::unique_lock<std::mutex> lock(dr_deque_mutex_);
    if (dr_deque_.empty()) {
      HLOG_ERROR << "dr_deque is empty";
      return false;
    }
    ctx->dr_node = *(dr_deque_.back());
  }

  if (!GetLocalPose(ctx)) {
    HLOG_ERROR << "get loca pose failed";
    if (params_.require_local_pose) {
      return false;
    }
  }

  if (!GetGlobalPose(ctx)) {
    HLOG_WARN << "get global pose failed";
  }

#ifdef ISORIN
  // mapping trigger ins 状态或定位结果跳变
  std::shared_ptr<Node> last_ins;
  ins_deque_mutex_.lock();
  if (!ins_deque_.empty()) {
    last_ins = ins_deque_.back();
  }
  ins_deque_mutex_.unlock();
  if (last_ins != nullptr) {
    CheckTriggerInsStatePose(last_ins);
  }
#endif
  return true;
}

bool FusionCenter::IsInsDrift(const std::shared_ptr<Node> ins_node) {
  if (!ref_init_) {
    HLOG_ERROR << "ref_init fail";
    return false;
  }
  const Eigen::Vector3d refpoint = Refpoint();
  int i_size = static_cast<int>(ins_trig_deque_.size());
  if (i_size < 20) {
    ins_trig_deque_.push_back(ins_node);
    return false;
  }
  if (i_size >= 20) {
    ins_trig_deque_.pop_front();
    ins_trig_deque_.push_back(ins_node);
  }
  // Node curr_i_node, prev_i_node;
  auto curr_i_node = ins_trig_deque_.at(i_size - 1);
  curr_i_node->enu = hmu::Geo::BlhToEnu(curr_i_node->blh, refpoint);
  auto prev_i_node = ins_trig_deque_.at(i_size - 2);
  prev_i_node->enu = hmu::Geo::BlhToEnu(prev_i_node->blh, refpoint);
  // rtk状态正常时判断，异常直接返回
  if (curr_i_node->rtk_status != 4 || curr_i_node->sys_status != 2) {
    return false;
  }
  const auto pose_diff =
      Node2SE3(*prev_i_node).inverse() * Node2SE3(*curr_i_node);
  if (fabs(pose_diff.translation().y()) > 0.5) {
    HLOG_INFO << "Vehicle y diff:" << fabs(pose_diff.translation().y())
              << " larger than thr:" << 0.5;
    return true;
  }
  return false;
}

bool FusionCenter::IsInsStateChange(const std::shared_ptr<Node> node) {
  static uint32_t last_sys;
  static uint32_t last_rtk;
  static bool first_flag = true;
  if (first_flag) {
    last_sys = node->sys_status;
    last_rtk = node->rtk_status;
    first_flag = false;
    return false;
  } else if ((last_rtk == 4 && node->rtk_status != 4 &&
              node->rtk_status != 5) ||
             (last_sys == 2 && node->sys_status != 2)) {
    HLOG_INFO << "node->rtk_status: " << node->rtk_status
              << " node->sys_status: " << node->sys_status;
    last_sys = node->sys_status;
    last_rtk = node->rtk_status;
    return true;
  }
  last_sys = node->sys_status;
  last_rtk = node->rtk_status;
  return false;
}

void FusionCenter::CheckTriggerInsStatePose(
    const std::shared_ptr<Node> i_node) {
  static double last_time = -1;
  static bool enable_02 = true;
  if (i_node->linear_vel_VRF(0) < 6.0 || i_node->sys_status != 2) {
    return;
  }
  if (IsInsStateChange(i_node)) {
    if (enable_02) {
      HLOG_WARN << "Start to trigger dc 1002 state";
      GLOBAL_DC_TRIGGER.TriggerCollect(1002);
      enable_02 = false;
      last_time = i_node->ticktime;
    }
  } else if (IsInsDrift(i_node)) {
    if (enable_02) {
      HLOG_WARN << "Start to trigger dc 1002 drift";
      GLOBAL_DC_TRIGGER.TriggerCollect(1002);
      enable_02 = false;
      last_time = i_node->ticktime;
    }
  }
  enable_02 = (i_node->ticktime - last_time) > 600;
}

bool FusionCenter::LoadParams(const std::string& configfile) {
  YAML::Node node = YAML::LoadFile(configfile);
  params_.recv_imu = node["recv_imu"].as<bool>();
  params_.recv_ins = node["recv_ins"].as<bool>();
  params_.recv_dr = node["recv_dr"].as<bool>();
  params_.recv_pe = node["recv_pe"].as<bool>();
  params_.imu_deque_max_size = node["imu_deque_max_size"].as<uint32_t>();
  params_.ins_deque_max_size = node["ins_deque_max_size"].as<uint32_t>();
  params_.dr_deque_max_size = node["dr_deque_max_size"].as<uint32_t>();
  params_.pe_deque_max_size = node["pe_deque_max_size"].as<uint32_t>();
  params_.passthrough_ins = node["passthrough_ins"].as<bool>();

  params_.smooth_outputs = node["smooth_outputs"].as<bool>();
  params_.use_smooth_yaw = node["use_smooth_yaw"].as<bool>();
  params_.search_state_cnt = node["search_state_cnt"].as<uint32_t>();
  params_.refpoint_update_dist = node["refpoint_update_dist"].as<double>();
  params_.no_mm_min_time = node["no_mm_min_time"].as<double>();
  params_.no_mm_max_time = node["no_mm_max_time"].as<double>();
  params_.no_mm_max_dis = node["no_mm_max_dis"].as<double>();
  params_.use_dr_measurement = node["use_dr_measurement"].as<bool>();
  params_.run_fusion_interval_ms =
      node["run_fusion_interval_ms"].as<uint32_t>();
  params_.window_size = node["window_size"].as<uint32_t>();
  params_.require_local_pose = node["require_local_pose"].as<bool>();
  params_.use_ins_predict_mm = node["use_ins_predict_mm"].as<bool>();
  params_.use_debug_txt = node["use_debug_txt"].as<bool>();
  params_.loc2_posx_conv = node["loc2_posx_conv"].as<double>();
  params_.loc2_posy_conv = node["loc2_posy_conv"].as<double>();
  params_.loc2_yaw_conv = node["loc2_yaw_conv"].as<double>();
  params_.exit_multiple = node["exit_multiple"].as<double>();
  params_.ins_init_status.clear();
  const auto& init_status_node = node["ins_init_status"];
  for (const auto& init_status : init_status_node) {
    const auto sys_status = init_status["sys_status"].as<uint32_t>();
    const auto gps_status = init_status["gps_status"].as<uint32_t>();
    params_.ins_init_status.emplace_back(sys_status, gps_status);
  }
  params_.lateral_error_compensation =
      node["lateral_error_compensation"].as<bool>();
  params_.max_dr_pe_horizontal_dist_error =
      node["max_dr_pe_horizontal_dist_error"].as<double>();
  params_.max_dr_pe_heading_error =
      node["max_dr_pe_heading_error"].as<double>();
  params_.max_fc_pe_horizontal_dist_error =
      node["max_fc_pe_horizontal_dist_error"].as<double>();
  return true;
}

template <typename T>
void FusionCenter::ShrinkQueue(T* const deque, uint32_t maxsize) {
  if (!deque) {
    return;
  }
  while (deque->size() > maxsize) {
    deque->pop_front();
  }
}

bool FusionCenter::DrToBasicInfo(
    const hozon::dead_reckoning::DeadReckoning& msg, Node* const node) {
  if (node == nullptr) {
    return false;
  }
  const auto& mq = msg.pose().pose_local().quaternion();
  if (std::isnan(mq.w()) || std::isnan(mq.x()) || std::isnan(mq.y()) ||
      std::isnan(mq.z())) {
    HLOG_WARN << "DeadReckoning is nan";
    return false;
  }

  Eigen::Quaterniond q(msg.pose().pose_local().quaternion().w(),
                       msg.pose().pose_local().quaternion().x(),
                       msg.pose().pose_local().quaternion().y(),
                       msg.pose().pose_local().quaternion().z());
  if (q.norm() < 1e-10) {
    HLOG_ERROR << "DeadReckoning quaternion(w,x,y,z) "
               << msg.pose().pose_local().quaternion().w() << ","
               << msg.pose().pose_local().quaternion().x() << ","
               << msg.pose().pose_local().quaternion().y() << ","
               << msg.pose().pose_local().quaternion().z();
    return false;
  }
  if (std::fabs(q.norm() - 1) > 1e-3) {
    HLOG_ERROR << "DeadReckoning quaternion(w,x,y,z) "
               << msg.pose().pose_local().quaternion().w() << ","
               << msg.pose().pose_local().quaternion().x() << ","
               << msg.pose().pose_local().quaternion().y() << ","
               << msg.pose().pose_local().quaternion().z()
               << ",norm:" << q.norm();
    return false;
  }
  node->seq = msg.header().seq();
  node->ticktime = msg.header().data_stamp();
  node->blh << msg.pose().pose_local().position().x(),
      msg.pose().pose_local().position().y(),
      msg.pose().pose_local().position().z();
  node->orientation = Sophus::SO3d(q).log();
  node->velocity << msg.velocity().twist_vrf().linear_vrf().x(),
      msg.velocity().twist_vrf().linear_vrf().y(),
      msg.velocity().twist_vrf().linear_vrf().z();
  node->angular_velocity << msg.velocity().twist_vrf().angular_vrf().x(),
      msg.velocity().twist_vrf().angular_vrf().y(),
      msg.velocity().twist_vrf().angular_vrf().z();
  node->linear_accel << msg.acceleration().linear_vrf().linear_raw_vrf().x(),
      msg.acceleration().linear_vrf().linear_raw_vrf().y(),
      msg.acceleration().linear_vrf().linear_raw_vrf().z();
  node->quaternion = q;
  // dr是逆时针（0-360），fc是顺时针（-180~180)
  // 局部坐标系是以x轴为0度，逆时针。
  auto heading = msg.pose().pose_local().heading();
  heading = heading > 180.0F ? heading - 360.0F : heading;
  while (heading < -180) {
    heading += 360;
  }
  while (heading > 180) {
    heading -= 360;
  }
  node->heading = heading;
  node->refpoint = Refpoint();

  node->enu = node->blh;

  for (int i = 0; i < 36; ++i) {
    node->cov(i) = 0;
  }
  return true;
}

bool FusionCenter::ExtractBasicInfo(const HafNodeInfo& msg, Node* const node) {
  if (node == nullptr) {
    HLOG_WARN << "ins node is nullptr";
    return false;
  }

  const auto& mq = msg.quaternion();
  if (std::isnan(mq.w()) || std::isnan(mq.x()) || std::isnan(mq.y()) ||
      std::isnan(mq.z())) {
    HLOG_WARN << "quaternion is nan";
    return false;
  }

  Eigen::Quaterniond q(msg.quaternion().w(), msg.quaternion().x(),
                       msg.quaternion().y(), msg.quaternion().z());

  if (q.norm() < 1e-10) {
    HLOG_WARN << "HafNodeInfo quaternion(w,x,y,z) " << msg.quaternion().w()
              << "," << msg.quaternion().x() << "," << msg.quaternion().y()
              << "," << msg.quaternion().z();
    return false;
  }

  if (std::fabs(q.norm() - 1) > 1e-3) {
    HLOG_WARN << "HafNodeInfo quaternion(w,x,y,z) " << msg.quaternion().w()
              << "," << msg.quaternion().x() << "," << msg.quaternion().y()
              << "," << msg.quaternion().z() << ",norm:" << q.norm();
    return false;
  }

  node->seq = msg.header().seq();
  node->ticktime = msg.header().data_stamp();
  node->blh << msg.pos_gcj02().x(), msg.pos_gcj02().y(), msg.pos_gcj02().z();
  node->orientation = Sophus::SO3d(q).log();
  node->velocity << msg.linear_velocity().x(), msg.linear_velocity().y(),
      msg.linear_velocity().z();
  node->angular_velocity << msg.angular_velocity().x(),
      msg.angular_velocity().y(), msg.angular_velocity().z();
  node->linear_accel << msg.linear_acceleration().x(),
      msg.linear_acceleration().y(), msg.linear_acceleration().z();
  node->b_a << msg.accel_bias().x(), msg.accel_bias().y(), msg.accel_bias().z();
  node->b_g << msg.gyro_bias().x(), msg.gyro_bias().y(), msg.gyro_bias().z();
  node->quaternion = q;
  node->sys_status = msg.sys_status();
  node->rtk_status = msg.gps_status();
  node->heading = msg.heading();
  node->refpoint = Refpoint();

  if (node->type == NodeType::MM) {
    node->pe_cov_coef = msg.sd_velocity().y();
  }

  if (msg.type() == hozon::localization::HafNodeInfo_NodeType_DR) {
    // dr fusion pos_gcj02 means local pose in dr coord
    node->enu = node->blh;
  } else {
    node->enu = hmu::Geo::BlhToEnu(node->blh, node->refpoint);
  }

  for (int i = 0; i < msg.covariance_size(); ++i) {
    node->cov(i) = msg.covariance()[i];
  }

  return true;
}

void FusionCenter::RunFusion() {
  pthread_setname_np(pthread_self(), "loc_fc_eskf");

  while (fusion_run_) {
    // refpoint
    if (!ref_init_) {
      HLOG_ERROR << "ref_init fail";
      usleep(params_.run_fusion_interval_ms * 1000);
      continue;
    }
    const Eigen::Vector3d refpoint = Refpoint();

    // Init
    if (!init_ins_ && !PoseInit(refpoint)) {
      usleep(params_.run_fusion_interval_ms * 1000);
      continue;
    }

    // ESKF
    bool meas_flag = GenerateNewESKFMeas(refpoint);
    bool pre_flag = GenerateNewESKFPre();

    if (pre_flag && meas_flag) {
      RunESKFFusion(refpoint);
    }
    PruneDeques();
    usleep(params_.run_fusion_interval_ms * 1000);
  }
}

void FusionCenter::PruneDeques() {
  if (fusion_deque_.empty()) {
    ins_deque_mutex_.lock();
    ins_deque_.clear();
    ins_deque_mutex_.unlock();

    dr_deque_mutex_.lock();
    dr_deque_.clear();
    dr_deque_mutex_.unlock();

    pe_deque_mutex_.lock();
    pe_deque_.clear();
    pe_deque_mutex_.unlock();

    imuins_deque_mutex_.lock();
    imuins_deque_.clear();
    imuins_deque_mutex_.unlock();

    return;
  }

  fusion_deque_mutex_.lock();
  while (fusion_deque_.size() > params_.window_size) {
    fusion_deque_.pop_front();
  }
  const double ticktime = fusion_deque_.front()->ticktime - 3.0;
  fusion_deque_mutex_.unlock();

  ins_deque_mutex_.lock();
  CutoffDeque(ticktime, &ins_deque_);
  ins_deque_mutex_.unlock();

  dr_deque_mutex_.lock();
  CutoffDeque(ticktime, &dr_deque_);
  dr_deque_mutex_.unlock();

  pe_deque_mutex_.lock();
  CutoffDeque(ticktime, &pe_deque_);
  pe_deque_mutex_.unlock();

  imuins_deque_mutex_.lock();
  while (!imuins_deque_.empty()) {
    const auto& imu_phy_data = imuins_deque_.front();
    double imu_tick = (*imu_phy_data).header().data_stamp();

    if (imu_tick > ticktime) {
      break;
    }
    imuins_deque_.pop_front();
  }
  imuins_deque_mutex_.unlock();
}

template <typename T>
void FusionCenter::CutoffDeque(double timestamp,
                               std::deque<std::shared_ptr<T>>* const d) {
  if (!d) {
    return;
  }
  while (!d->empty() && d->front()->ticktime <= timestamp) {
    d->pop_front();
  }
}

void FusionCenter::SetRefpoint(const Eigen::Vector3d& blh) {
  std::unique_lock<std::mutex> lock(refpoint_mutex_);
  refpoint_ = blh;
}

Eigen::Vector3d FusionCenter::Refpoint() {
  std::unique_lock<std::mutex> lock(refpoint_mutex_);
  return refpoint_;
}

double ConvertHeading(double heading) {
  double cal_heading = -heading * M_PI / 180.0 + M_PI / 2;
  if (cal_heading > M_PI) {
    cal_heading -= 2 * M_PI;
  } else if (cal_heading < -M_PI) {
    cal_heading += 2 * M_PI;
  }
  return cal_heading;
}

Eigen::Isometry3d FusionCenter::Node2Eigen(const Node& node) {
  Eigen::Quaterniond q(node.quaternion);
  Eigen::Vector3d t(node.enu);
  Eigen::Isometry3d res = Eigen::Isometry3d::Identity();
  res.rotate(q);
  res.pretranslate(t);
  return res;
}

void FusionCenter::Node2Localization(const Context& ctx,
                                     Localization* const location) {
  if (location == nullptr) {
    return;
  }

  location->Clear();

  const auto& ins = ctx.ins;
  const auto& imu = ctx.imuins.imu_info();
  const auto& global_node = ctx.global_node;
  const auto& local_node = ctx.local_node;
  // publish time align to dr coordinate
  const double ticktime = local_node.ticktime;

  auto* const header = location->mutable_header();
  header->set_seq(seq_++);
  header->set_frame_id("location");
  header->set_data_stamp(ticktime);

  if (abs(global_node.ticktime - local_node.ticktime) > 0.001) {
    HLOG_INFO << "location global node.ticktime:" << global_node.ticktime
              << ", local node.ticktime:" << local_node.ticktime;
  }
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>
      tp = std::chrono::time_point_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now());
  header->set_publish_stamp(tp.time_since_epoch().count() * 1.0e-9);
  header->set_gnss_stamp(ticktime);

  location->set_measurement_time(ticktime);

  location->set_gps_week(ins.gps_week());
  location->set_gps_sec(ins.gps_sec());
  location->set_received_ehp_counter(ehp_counter_);
  location->set_rtk_status(global_node.rtk_status);
  location->set_location_state(global_node.location_state);
  if (!global_node.state) {
    location->set_location_state(0);
  }
  // location->mutable_mounting_error()->set_x(
  //     static_cast<float>(ins.mounting_error().x()));
  // location->mutable_mounting_error()->set_y(
  //     static_cast<float>(ins.mounting_error().y()));
  // location->mutable_mounting_error()->set_z(
  //     static_cast<float>(ins.mounting_error().z()));

  auto* const pose = location->mutable_pose();

  const Sophus::SO3d& rot = Sophus::SO3d::exp(global_node.orientation);
  // const auto& ins_node = ctx.ins_node;
  // const Sophus::SO3d& rot = Sophus::SO3d::exp(ins_node.orientation);
  pose->mutable_quaternion()->set_w(
      static_cast<float>(rot.unit_quaternion().w()));
  pose->mutable_quaternion()->set_x(
      static_cast<float>(rot.unit_quaternion().x()));
  pose->mutable_quaternion()->set_y(
      static_cast<float>(rot.unit_quaternion().y()));
  pose->mutable_quaternion()->set_z(
      static_cast<float>(rot.unit_quaternion().z()));

  Eigen::Vector3d euler = Rot2Euler312(rot.matrix());
  euler = euler - ((euler.array() > M_PI).cast<double>() * 2.0 * M_PI).matrix();
  pose->mutable_euler_angle()->set_x(euler.x());
  pose->mutable_euler_angle()->set_y(euler.y());
  pose->mutable_euler_angle()->set_z(euler.z());
  pose->mutable_euler_angles()->set_x(euler.x());
  pose->mutable_euler_angles()->set_y(euler.y());
  pose->mutable_euler_angles()->set_z(euler.z());

  pose->mutable_rotation_vrf()->set_x(rot.log().x());
  pose->mutable_rotation_vrf()->set_y(rot.log().y());
  pose->mutable_rotation_vrf()->set_z(rot.log().z());

  double heading = 90.0 - euler.z() / M_PI * 180;
  if (heading < 0.0) {
    heading += 360.0;
  }
  pose->set_heading(static_cast<float>(heading));

  pose->mutable_linear_velocity_vrf()->set_x(local_node.velocity(0));
  pose->mutable_linear_velocity_vrf()->set_y(local_node.velocity(1));
  pose->mutable_linear_velocity_vrf()->set_z(local_node.velocity(2));

  pose->mutable_linear_acceleration_vrf()->set_x(ins.linear_acceleration().x());
  pose->mutable_linear_acceleration_vrf()->set_y(ins.linear_acceleration().y());
  pose->mutable_linear_acceleration_vrf()->set_z(ins.linear_acceleration().z());

  pose->mutable_angular_velocity_vrf()->set_x(ins.angular_velocity().x());
  pose->mutable_angular_velocity_vrf()->set_y(ins.angular_velocity().y());
  pose->mutable_angular_velocity_vrf()->set_z(ins.angular_velocity().z());

  pose->mutable_gcj02()->set_x(global_node.blh(0));
  pose->mutable_gcj02()->set_y(global_node.blh(1));
  pose->mutable_gcj02()->set_z(global_node.blh(2));

  // mashaoping map's default value is 51
  // const double zone = pose->gcj02().y() / 6.0 + 31;
  // const uint32_t curr_zone = std::floor(zone);
  // uint32_t near_zone = 0;
  // if ((zone - curr_zone) > 0.5) {
  //   near_zone = curr_zone + 1;
  // } else {
  //   near_zone = curr_zone - 1;
  // }

  // pose->set_utm_zone_01(curr_zone);
  // pose->set_utm_zone_02(near_zone);
  // pose->set_using_utm_zone(curr_zone);

  auto curr_zone = 51;
  auto near_zone = 51;
  pose->set_utm_zone_01(curr_zone);
  pose->set_utm_zone_02(near_zone);
  pose->set_using_utm_zone(curr_zone);

  Eigen::Vector3d curr_utm;
  Eigen::Vector3d near_utm;
  hmu::Geo::LatLonToUtmXy(curr_zone, pose->gcj02().y(), pose->gcj02().x(),
                          &curr_utm);
  hmu::Geo::LatLonToUtmXy(near_zone, pose->gcj02().y(), pose->gcj02().x(),
                          &near_utm);

  pose->mutable_pos_utm_01()->set_x(curr_utm(0));
  pose->mutable_pos_utm_01()->set_y(curr_utm(1));
  pose->mutable_pos_utm_01()->set_z(curr_utm(2));
  pose->mutable_pos_utm_02()->set_x(near_utm(0));
  pose->mutable_pos_utm_02()->set_y(near_utm(1));
  pose->mutable_pos_utm_02()->set_z(near_utm(2));

  // set laneid
  // temp close getting lane_id
  // const std::string laneid = GetHdCurrLaneId(curr_utm,
  // ConvertHeading(heading)); location->set_laneid(laneid);

  // pose->mutable_linear_acceleration_raw_vrf()->set_x(
  //     imu.imuvb_linear_acceleration().x());
  // pose->mutable_linear_acceleration_raw_vrf()->set_y(
  //     imu.imuvb_linear_acceleration().y());
  // pose->mutable_linear_acceleration_raw_vrf()->set_z(
  //     imu.imuvb_linear_acceleration().z());

  pose->mutable_linear_velocity()->set_x(global_node.velocity(0));
  pose->mutable_linear_velocity()->set_y(global_node.velocity(1));
  pose->mutable_linear_velocity()->set_z(global_node.velocity(2));

  pose->mutable_local_pose()->set_x(local_node.enu(0));
  pose->mutable_local_pose()->set_y(local_node.enu(1));
  pose->mutable_local_pose()->set_z(local_node.enu(2));
  auto local_heading = local_node.heading - init_dr_node_.heading;
  local_heading =
      local_heading > 180.0F ? local_heading - 360.0F : local_heading;
  while (local_heading < -180) {
    local_heading += 360;
  }
  while (local_heading > 180) {
    local_heading -= 360;
  }
  pose->set_local_heading(local_heading);

  // pose->mutable_angular_velocity_raw_vrf()->set_x(
  //     imu.imuvb_angular_velocity().x());
  // pose->mutable_angular_velocity_raw_vrf()->set_y(
  //     imu.imuvb_angular_velocity().y());
  // pose->mutable_angular_velocity_raw_vrf()->set_z(
  //     imu.imuvb_angular_velocity().z());

  // KF滤波参数
  pose->mutable_linear_acceleration_raw_vrf()->set_x(global_node.KF_kdiff(0) *
                                                     1e5);
  pose->mutable_linear_acceleration_raw_vrf()->set_y(global_node.KF_kdiff(1) *
                                                     1e5);
  pose->mutable_linear_acceleration_raw_vrf()->set_z(global_node.KF_kdiff(2) *
                                                     1e5);

  pose->mutable_angular_velocity_raw_vrf()->set_x(global_node.KF_cov(0, 0));
  pose->mutable_angular_velocity_raw_vrf()->set_y(global_node.KF_cov(1, 1));
  pose->mutable_angular_velocity_raw_vrf()->set_z(global_node.KF_cov(2, 2));

  pose->mutable_wgs()->set_x(global_node.KF_kdiff(5) * 1e5);
  pose->mutable_wgs()->set_y(global_node.KF_cov(5, 5) * 1e3);
  // pose->mutable_wgs()->set_z(ins.pos_wgs().z());
  if (filter_valid_pe_) {
    // filter mm valid
    pose->mutable_wgs()->set_z(1.0);
  } else {
    pose->mutable_wgs()->set_z(2.0);
  }

  const Sophus::SO3d& rot_local = Sophus::SO3d::exp(local_node.orientation);
  Eigen::Vector3d euler_local = Rot2Euler312(rot_local.matrix());
  euler_local =
      euler_local -
      ((euler_local.array() > M_PI).cast<double>() * 2.0 * M_PI).matrix();

  pose->mutable_euler_angles_local()->set_x(euler_local.x());
  pose->mutable_euler_angles_local()->set_y(euler_local.y());
  pose->mutable_euler_angles_local()->set_z(euler_local.z());

  auto* const pose_local = location->mutable_pose_local();
  pose_local->mutable_quaternion()->set_w(
      static_cast<float>(local_node.quaternion.w()));
  pose_local->mutable_quaternion()->set_x(
      static_cast<float>(local_node.quaternion.x()));
  pose_local->mutable_quaternion()->set_y(
      static_cast<float>(local_node.quaternion.y()));
  pose_local->mutable_quaternion()->set_z(
      static_cast<float>(local_node.quaternion.z()));
  pose_local->mutable_position()->set_x(local_node.enu(0));
  pose_local->mutable_position()->set_y(local_node.enu(1));
  pose_local->mutable_position()->set_z(local_node.enu(2));

  pose_local->mutable_linear_velocity()->set_x(local_node.velocity(0));
  pose_local->mutable_linear_velocity()->set_y(local_node.velocity(1));
  pose_local->mutable_linear_velocity()->set_z(local_node.velocity(2));

  pose_local->mutable_angular_velocity()->set_x(local_node.angular_velocity(0));
  pose_local->mutable_angular_velocity()->set_y(local_node.angular_velocity(1));
  pose_local->mutable_angular_velocity()->set_z(local_node.angular_velocity(2));

  pose_local->set_local_heading(local_heading);

  pose_local->mutable_linear_acceleration()->set_x(local_node.linear_accel(0));
  pose_local->mutable_linear_acceleration()->set_y(local_node.linear_accel(1));
  pose_local->mutable_linear_acceleration()->set_z(local_node.linear_accel(2));

  pose_local->mutable_angular_velocity()->set_x(local_node.angular_velocity(0));
  pose_local->mutable_angular_velocity()->set_y(local_node.angular_velocity(1));
  pose_local->mutable_angular_velocity()->set_z(local_node.angular_velocity(2));

  // 协方差改为使用FC的ESKF结果，目前由于非实时架构，中间会有0
  static int count = 0;
  Eigen::Matrix<float, 6, 1> diag;
  // diag << static_cast<float>(ins.sd_position().x()),
  //     static_cast<float>(ins.sd_position().y()),
  //     static_cast<float>(ins.sd_position().z()),
  //     static_cast<float>(ins.sd_attitude().x()),
  //     static_cast<float>(ins.sd_attitude().y()),
  //     static_cast<float>(ins.sd_attitude().z());
  // Eigen::Matrix<float, 6, 6, Eigen::RowMajor> sd = diag.asDiagonal();
  // Eigen::Matrix<float, 6, 6, Eigen::RowMajor> cov = sd * sd;
  diag << static_cast<float>(global_node.cov(0, 0) * 1e10),
      static_cast<float>(global_node.cov(1, 1) * 1e10),
      static_cast<float>(global_node.cov(2, 2) * 1e10),
      static_cast<float>(global_node.cov(6, 6) * 1e10),
      static_cast<float>(global_node.cov(7, 7) * 1e10),
      static_cast<float>(global_node.cov(8, 8) * 1e10);
  Eigen::Matrix<float, 6, 6, Eigen::RowMajor> cov = diag.asDiagonal();
  location->mutable_mounting_error()->set_x(
      static_cast<float>(global_node.cov(0, 0) * 1e10));
  location->mutable_mounting_error()->set_y(
      static_cast<float>(global_node.cov(1, 1) * 1e10));
  location->mutable_mounting_error()->set_z(
      static_cast<float>(global_node.cov(8, 8) * 1e10));
  ++count;
  if (count >= 100) {
    count = 0;
    HLOG_INFO << "dr_ticktime:" << ticktime
              << ",ESKF_Cov:" << global_node.cov(0, 0) << ", "
              << global_node.cov(1, 1) << ", " << global_node.cov(2, 2) << ", "
              << global_node.cov(6, 6) << ", " << global_node.cov(7, 7) << ", "
              << global_node.cov(8, 8) << ",gcj02:" << pose->gcj02().x() << ","
              << pose->gcj02().y();
  }
  location->clear_covariance();
  for (int i = 0; i < 36; ++i) {
    location->add_covariance(cov(i));
  }

  const auto local_to_global_pose = Node2SE3(init_ins_node_) *
                                    Node2SE3(init_dr_node_).inverse() *
                                    Node2SE3(ctx.dr_node);

  auto* const pose_dr = location->mutable_pose_dr();
  auto enu = local_to_global_pose.translation();
  auto global_dr_pos = hmu::Geo::EnuToBlh(enu, ctx.ins_node.refpoint);
  pose_dr->mutable_position()->set_x(global_dr_pos(0));
  pose_dr->mutable_position()->set_y(global_dr_pos(1));
  pose_dr->mutable_position()->set_z(global_dr_pos(2));

  auto q_dr = Eigen::Quaterniond(local_to_global_pose.rotationMatrix());
  pose_dr->mutable_quaternion()->set_w(q_dr.w());
  pose_dr->mutable_quaternion()->set_x(q_dr.x());
  pose_dr->mutable_quaternion()->set_y(q_dr.y());
  pose_dr->mutable_quaternion()->set_z(q_dr.z());

  // debug
  if (global_node.rtk_status == 0) {
    HLOG_WARN << "ins_seq:" << ins.header().seq()
              << ", ins_ticktime:" << ins.header().data_stamp()
              << " ,ins_gps_state:" << ins.gps_status()
              << " ,fc_rtk_state:" << global_node.rtk_status
              << " ,ins_linear_velocity:" << ins.linear_velocity().x() << " ,"
              << ins.linear_velocity().y() << " ," << ins.linear_velocity().z();
  }
}

bool FusionCenter::IsInterpolable(const std::shared_ptr<Node>& n1,
                                  const std::shared_ptr<Node>& n2,
                                  double dis_tol, double ang_tol,
                                  double time_tol) {
  if (n1 == nullptr || n2 == nullptr) {
    return false;
  }

  const double dis_delta = (n1->enu - n2->enu).norm();
  const double ang_delta = (Sophus::SO3d::exp(n1->orientation).inverse() *
                            Sophus::SO3d::exp(n2->orientation))
                               .log()
                               .norm();
  const double time_delta = fabs(n2->ticktime - n1->ticktime);

  return (dis_delta < dis_tol && ang_delta < ang_tol && time_delta < time_tol);
}

bool FusionCenter::Interpolate(double ticktime,
                               const std::deque<std::shared_ptr<Node>>& d,
                               Node* const node, double dis_tol, double ang_tol,
                               double time_tol) {
  if (node == nullptr) {
    return false;
  }
  int i = 0;
  const int dlen = static_cast<int>(d.size());

  if (dlen == 0) {
    HLOG_WARN << "Interpolate error 1,INS deque is empty";
    return false;
  }
  for (; i < dlen; ++i) {
    if (fabs(d[i]->ticktime - ticktime) < 1e-3) {
      *node = *(d[i]);
      return true;
    }
    if (d[i]->ticktime > ticktime) {
      break;
    }
  }

  if (i == 0 || i == dlen) {
    HLOG_WARN << "Interpolate error 2,i:" << i;
    return false;
  }

  if (!IsInterpolable(d[i - 1], d[i], dis_tol, ang_tol, time_tol)) {
    HLOG_WARN << "Interpolate error 3";
    return false;
  }

  Sophus::SE3d p1 = Node2SE3(d[i - 1]);
  Sophus::SE3d p2 = Node2SE3(d[i]);
  Sophus::SE3d delta = p1.inverse() * p2;

  double ratio =
      (ticktime - d[i - 1]->ticktime) / (d[i]->ticktime - d[i - 1]->ticktime);
  if (std::isnan(ratio) || std::isinf(ratio)) {
    HLOG_WARN << "Interpolate error 4, ratio error";
    return false;
  }

  auto pose = p1 * Sophus::SE3d(Sophus::SO3d::exp(ratio * delta.so3().log()),
                                ratio * delta.translation());
  node->enu = pose.translation();
  node->orientation = pose.so3().log();
  node->b_a = (1 - ratio) * d[i - 1]->b_a + ratio * d[i]->b_a;
  node->b_g = (1 - ratio) * d[i - 1]->b_g + ratio * d[i]->b_g;
  node->velocity = (1 - ratio) * d[i - 1]->velocity + ratio * d[i]->velocity;
  node->ticktime = ticktime;
  node->sys_status = d[i]->sys_status;
  node->rtk_status = d[i]->rtk_status;

  return true;
}

bool FusionCenter::PoseInit(const Eigen::Vector3d& refpoint) {
  // 1.使用ins初始化(if--->第一次初始化
  if (!prev_global_valid_) {
    std::unique_lock<std::mutex> lock(ins_deque_mutex_);
    for (auto iter = ins_deque_.rbegin(); iter != ins_deque_.rend(); ++iter) {
      if (AllowInit((*iter)->sys_status, (*iter)->rtk_status)) {
        (*iter)->enu = hmu::Geo::BlhToEnu((*iter)->blh, refpoint);
        InsertESKFFusionNode(**iter);
        init_ins_ = true;
        return true;
      }
    }
    return false;
  } else {
    // 2.后续初始化（非第一次，通过prev_global_valid_判断）
    fusion_deque_mutex_.lock();
    fusion_deque_.clear();
    fusion_deque_mutex_.unlock();
    // 2.1 首选使用上次最终的融合定位结果初始化，卡尔曼之后的输出
    std::unique_lock<std::mutex> lock(ins_deque_mutex_);
    std::unique_lock<std::mutex> lock2(prev_global_node_mutex_);
    double ins_earliest_time = ins_deque_.front()->ticktime;

    if (prev_global_node_.ticktime > ins_earliest_time) {
      prev_global_node_.enu =
          hmu::Geo::BlhToEnu(prev_global_node_.blh, refpoint);
      InsertESKFFusionNode(prev_global_node_);
      HLOG_ERROR << "FusionDeque use prev_global_node_ reinit";
      init_ins_ = true;
      return true;
    } else {
      // 2.2 备选使用INS测量进行初始化，防止上一次定位有效输出无法在INS队列插值
      for (auto iter = ins_deque_.rbegin(); iter != ins_deque_.rend(); ++iter) {
        if (AllowInit((*iter)->sys_status, (*iter)->rtk_status)) {
          (*iter)->enu = hmu::Geo::BlhToEnu((*iter)->blh, refpoint);
          InsertESKFFusionNode(**iter);
          HLOG_ERROR << "FusionDeque use ins meas reinit";
          init_ins_ = true;
          return true;
        }
      }
    }
  }
  return false;
}

bool FusionCenter::GenerateNewESKFPre() {
  // 收集预测节点
  fusion_deque_mutex_.lock();
  double cur_ticktime = fusion_deque_.back()->ticktime;
  fusion_deque_mutex_.unlock();
  bool pre_flag = false;
  pre_deque_.clear();
  {
    std::unique_lock<std::mutex> lock(ins_deque_mutex_);
    for (const auto& ins_node : ins_deque_) {
      if (ins_node->ticktime - cur_ticktime > 0) {
        pre_deque_.emplace_back(std::make_shared<Node>(*ins_node));
        pre_flag = true;
      }
    }
  }

  return pre_flag;
}

bool FusionCenter::GenerateNewESKFMeas(const Eigen::Vector3d& refpoint) {
  // 进行优势传感器的测量数据收集（加入各种判断条件，如MM不工作时用INS）
  fusion_deque_mutex_.lock();
  auto last_fc_node = fusion_deque_.back();
  double cur_fusion_ticktime = last_fc_node->ticktime;
  Eigen::Vector3d cur_eskf_enu =
      hmu::Geo::BlhToEnu(last_fc_node->blh, refpoint);
  fusion_deque_mutex_.unlock();
  bool meas_flag = false;
  meas_deque_.clear();

  // 1.1 加入MM测量
  {
    std::unique_lock<std::mutex> lock(pe_deque_mutex_);
    for (const auto& mm_node : pe_deque_) {
      auto ticktime_diff = mm_node->ticktime - cur_fusion_ticktime;
      if (ticktime_diff > 0 && mm_node->ticktime > last_meas_time_ &&
          mm_node->state) {
        meas_deque_.emplace_back(std::make_shared<Node>(*mm_node));
        meas_flag = true;
      }
    }
  }
  // 1.2 加入ins_mm测量
  if (mm_is_valid_ == false) {
    std::unique_lock<std::mutex> lock(ins_meas_deque_mutex_);
    for (const auto& ins_mm_node : ins_meas_deque_) {
      auto ticktime_diff = ins_mm_node->ticktime - cur_fusion_ticktime;
      if (ticktime_diff > 0 && ins_mm_node->ticktime > last_meas_time_) {
        meas_deque_.emplace_back(std::make_shared<Node>(*ins_mm_node));
        meas_flag = true;
        HLOG_INFO << "add ins_mm measre";
      }
    }
  }
  // 1.3 MM不工作时
  if (!meas_flag) {
    latest_ins_mutex_.lock();
    double ins_ticktime = latest_ins_data_.header().data_stamp();
    latest_ins_mutex_.unlock();

    double time_diff = params_.no_mm_max_time;
    pe_deque_mutex_.lock();
    int mm_size = static_cast<int>(pe_deque_.size());
    if (mm_size != 0) {
      lastest_valid_pe_mutex_.lock();
      double cur_mm_ticktime = lastest_valid_pe_.ticktime;
      lastest_valid_pe_mutex_.unlock();
      time_diff = ins_ticktime - cur_mm_ticktime;
    }
    pe_deque_mutex_.unlock();

    // 对NCP增加距离和时间判断，持续超过100m或者150s无MM都会加入INS测量
    double no_mm_max_time = params_.no_mm_max_time;
    double dis_diff = 0.0;

    prev_global_node_mutex_.lock();
    if (prev_global_valid_) {
      Eigen::Vector3d cur_output_enu =
          hmu::Geo::BlhToEnu(prev_global_node_.blh, refpoint);
      dis_diff = (cur_eskf_enu - cur_output_enu).norm();
    }
    prev_global_node_mutex_.unlock();
    // (1)INS测量加入
    if (mm_size == 0 || time_diff >= no_mm_max_time ||
        dis_diff >= params_.no_mm_max_dis) {
      std::unique_lock<std::mutex> lock(ins_deque_mutex_);
      for (const auto& ins_node : ins_deque_) {
        if (ins_node->ticktime > cur_fusion_ticktime &&
            ins_node->ticktime > last_meas_time_) {
          std::shared_ptr<Node> new_ins_node =
              std::make_shared<Node>(*ins_node);
          meas_deque_.emplace_back(std::make_shared<Node>(*new_ins_node));
          meas_flag = true;
        }
      }
      HLOG_DEBUG << "No MM measurement,insert INS meas. time_diff:"
                 << time_diff;
    }
  }
  // 2. DR测量加入（目前用INS的相对代替）
  if (params_.use_dr_measurement) {
    if (!InsertESKFMeasDR()) {
      HLOG_ERROR << "Insert DR Measurements Error!";
    }
    // 根据时间戳对测量队列排序
    std::deque<std::shared_ptr<Node>> sort_deque;
    while (meas_deque_.size() > 0) {
      std::shared_ptr<Node> min_node = meas_deque_.front();
      auto del_index = meas_deque_.begin();

      for (auto it = del_index; it != meas_deque_.end(); ++it) {
        if (min_node->ticktime > (*it)->ticktime) {
          min_node = *it;
          del_index = it;
        }
      }
      sort_deque.push_back(min_node);
      meas_deque_.erase(del_index);
    }
    meas_deque_ = sort_deque;
  }
  // 队列排序
  std::sort(meas_deque_.begin(), meas_deque_.end(),
            [](const std::shared_ptr<Node>& node1,
               const std::shared_ptr<Node>& node2) {
              return node1->ticktime < node2->ticktime;
            });
  return meas_flag;
}

bool FusionCenter::PredictMMMeas() {
  if (meas_deque_.size() > 0) {
    HLOG_ERROR << "No need to predict MM Meas!";
    return false;
  }

  // 1.取出mm队列最新节点，并取出对应时间的MM节点
  pe_deque_mutex_.lock();
  const auto mm_refer = pe_deque_.back();
  pe_deque_mutex_.unlock();

  double mm_ticktime = mm_refer->ticktime;
  std::shared_ptr<Node> ins_refer = nullptr;

  ins_deque_mutex_.lock();
  for (const auto& ins_node : ins_deque_) {
    if (abs(ins_node->ticktime - mm_ticktime) < 0.001) {
      ins_refer = ins_node;
    }
  }
  ins_deque_mutex_.unlock();
  if (ins_refer == nullptr) {
    HLOG_ERROR << "Dont find INS refer node!";
    return false;
  }

  fusion_deque_mutex_.lock();
  double fusion_ticktime = fusion_deque_.back()->ticktime;
  fusion_deque_mutex_.unlock();

  // 2.100ms一个，根据INS测量递推出MM测量
  static double last_predict_mmmeas = 0;
  double now_ticktime = mm_ticktime;
  ins_deque_mutex_.lock();
  for (const auto& ins_node : ins_deque_) {
    if (ins_node->ticktime - now_ticktime > 0.1 &&
        ins_node->ticktime > fusion_ticktime &&
        ins_node->ticktime - last_predict_mmmeas > 0.1) {
      auto node = std::make_shared<Node>(*ins_node);
      const auto& T_delta =
          Node2SE3(*ins_refer).inverse() * Node2SE3(*ins_node);
      const auto& pose = Node2SE3(*mm_refer) * T_delta;
      node->enu = pose.translation();
      node->orientation = pose.so3().log();
      node->type = NodeType::INS;
      meas_deque_.emplace_back(node);
      now_ticktime = ins_node->ticktime;
      last_predict_mmmeas = ins_node->ticktime;
      HLOG_ERROR << "INS predict MM successfully,last_predict_mmmeas:"
                 << last_predict_mmmeas
                 << ", meas_deque_.size():" << meas_deque_.size();
    }
  }
  ins_deque_mutex_.unlock();
  return (meas_deque_.size() > 0);
}

void FusionCenter::InsertESKFFusionTmpNode(const Node& node) {
  auto new_node = std::make_shared<Node>();
  if (new_node == nullptr) {
    HLOG_ERROR << "new_node is nullptr";
    return;
  }
  *new_node = node;
  const Sophus::SO3d& rot = Sophus::SO3d::exp(new_node->orientation);
  Eigen::Vector3d euler = Rot2Euler312(rot.matrix());
  euler = euler - ((euler.array() > M_PI).cast<double>() * 2.0 * M_PI).matrix();
  new_node->heading = euler.z() / M_PI * 180;
  fusion_deque_tmp_.emplace_back(new_node);
}

void FusionCenter::InsertESKFFusionNode(const Node& node) {
  auto new_node = std::make_shared<Node>();
  if (new_node == nullptr) {
    HLOG_ERROR << "new_node is nullptr";
    return;
  }
  *new_node = node;
  fusion_deque_mutex_.lock();
  const Sophus::SO3d& rot = Sophus::SO3d::exp(new_node->orientation);
  Eigen::Vector3d euler = Rot2Euler312(rot.matrix());
  euler = euler - ((euler.array() > M_PI).cast<double>() * 2.0 * M_PI).matrix();
  new_node->heading = euler.z() / M_PI * 180;
  fusion_deque_.emplace_back(new_node);
  fusion_deque_mutex_.unlock();
}

void FusionCenter::RunESKFFusion(const Eigen::Vector3d& refpoint) {
  // eskf开始
  fusion_deque_mutex_.lock();
  HLOG_DEBUG << "-------eskf前-------"
             << "pre_deque_.size():" << pre_deque_.size()
             << ", meas_deque_.size():" << meas_deque_.size()
             << ", meas_type:" << static_cast<int>(meas_deque_.back()->type)
             << ", fusion_deque.size():" << fusion_deque_.size();
  auto init_node = std::make_shared<Node>(*fusion_deque_.back());
  fusion_deque_mutex_.unlock();

  fusion_deque_tmp_.clear();
  init_node->enu = hmu::Geo::BlhToEnu(init_node->blh, refpoint);
  eskf_->StateInit(init_node);

  while (!pre_deque_.empty() && !meas_deque_.empty()) {
    Node meas_node;
    Node predict_node;
    if (meas_deque_.front() == nullptr || pre_deque_.front() == nullptr) {
      HLOG_ERROR << "meas_deque_.front() and pre_deque_.front() is nullptr-";
      return;
    }
    meas_node = *meas_deque_.front();
    meas_node.enu = hmu::Geo::BlhToEnu(meas_node.blh, refpoint);
    predict_node = *pre_deque_.front();

    if (predict_node.ticktime < meas_node.ticktime) {
      if (!eskf_->Predict(predict_node)) {
        init_ins_ = false;
        return;
      }
      pre_deque_.pop_front();
    } else {
      // 多源融合时，多个测量时间戳一样时，进行原地更新（无需预测）
      if (abs(last_meas_time_ - meas_node.ticktime) <= 0.001) {
        eskf_->Correct(meas_node);
        meas_deque_.pop_front();
      } else {
        if (!eskf_->Predict(predict_node)) {
          init_ins_ = false;
          return;
        }
        pre_deque_.pop_front();
        eskf_->Correct(meas_node);
        last_meas_time_ = meas_node.ticktime;
        meas_deque_.pop_front();
      }
    }

    State state = eskf_->GetState();
    InsertESKFFusionTmpNode(State2Node(state, refpoint));
  }
  can_output_ = true;

  fusion_deque_mutex_.lock();
  while (!fusion_deque_tmp_.empty()) {
    fusion_deque_.push_back(fusion_deque_tmp_.front());
    fusion_deque_tmp_.pop_front();
  }
  HLOG_DEBUG << "-------eskf后-------"
             << "pre_deque_.size():" << pre_deque_.size()
             << ", meas_deque_.size():" << meas_deque_.size()
             << ", meas_type:" << static_cast<int>(meas_deque_.back()->type)
             << ", fusion_deque.size():" << fusion_deque_.size();
  fusion_deque_mutex_.unlock();
}

Node FusionCenter::State2Node(const State& state,
                              const Eigen::Vector3d& refpoint) {
  Node node;
  node.refpoint = refpoint;
  node.ticktime = state.ticktime;
  node.type = state.meas_type;
  node.enu = state.p;
  node.velocity = state.v;
  node.b_g = state.b_g;
  node.b_a = state.b_a;
  node.quaternion = state.R.unit_quaternion();
  node.orientation = state.R.log();
  node.blh = hmu::Geo::EnuToBlh(node.enu, node.refpoint);
  // 位置
  node.cov(0, 0) = state.cov(0, 0);
  node.cov(1, 1) = state.cov(1, 1);
  node.cov(2, 2) = state.cov(2, 2);
  // 姿态
  node.cov(6, 6) = state.cov(6, 6);
  node.cov(7, 7) = state.cov(7, 7);
  node.cov(8, 8) = state.cov(8, 8);

  node.sys_status = state.sys_status;
  node.rtk_status = state.rtk_status;

  return node;
}

bool FusionCenter::AllowInsMeas(uint32_t sys_status, uint32_t rtk_status) {
  return ((sys_status == 1 || sys_status == 2 || sys_status == 3) &&
          (rtk_status == 1 || rtk_status == 2 || rtk_status == 3 ||
           rtk_status == 4 || rtk_status == 5));
}

bool FusionCenter::AllowInit(uint32_t sys_status, uint32_t rtk_status) {
  return ((sys_status == 1 || sys_status == 2 || sys_status == 3) &&
          (rtk_status == 1 || rtk_status == 2 || rtk_status == 3 ||
           rtk_status == 4 || rtk_status == 5));
}

bool FusionCenter::InsertESKFMeasDR() {
  // 1.取出fc最新节点，并取出对应时间的INS节点
  fusion_deque_mutex_.lock();
  const auto fc_refer = fusion_deque_.back();
  fusion_deque_mutex_.unlock();

  double fc_ticktime = fc_refer->ticktime;
  std::shared_ptr<Node> ins_refer = nullptr;

  ins_deque_mutex_.lock();
  for (const auto& ins_node : ins_deque_) {
    if (abs(ins_node->ticktime - fc_ticktime) < 0.001) {
      ins_refer = ins_node;
    }
  }
  ins_deque_mutex_.unlock();
  if (ins_refer == nullptr) {
    HLOG_ERROR << "Dont find INS refer node!";
    return false;
  }

  // 2.取出超过当前fc时间戳的INS节点，并计算转换出以fc为基准加上取出INS节点的相对测量
  ins_deque_mutex_.lock();
  for (const auto& ins_node : ins_deque_) {
    if (ins_node->ticktime - fc_ticktime > 0.001) {
      auto node = std::make_shared<Node>(*ins_node);
      const auto& T_delta =
          Node2SE3(*ins_refer).inverse() * Node2SE3(*ins_node);
      const auto& pose = Node2SE3(*fc_refer) * T_delta;
      node->enu = pose.translation();
      node->orientation = pose.so3().log();
      meas_deque_.emplace_back(node);
    }
  }
  ins_deque_mutex_.unlock();

  return true;
}

double Rad2TwoPi(double rad) {
  double deg = rad * 180.0 / M_PI;
  if (deg < 0) {
    deg += 360.0;
  }
  if (deg > 360.0) {
    deg -= 360;
  }
  return deg;
}

double TwoPi2Rad(double deg) {
  double rad = deg > 180.0 ? deg - 360.0 : deg;
  rad = rad / 180.0 * M_PI;
  return rad;
}

Sophus::SE3d FusionCenter::Node2SE3(const Node& node) {
  return Sophus::SE3d(Sophus::SO3d::exp(node.orientation), node.enu);
}

Sophus::SE3d FusionCenter::Node2SE3(const std::shared_ptr<Node>& node) {
  return Node2SE3(*node);
}

void FusionCenter::KalmanFiltering(Node* const node,
                                   const Eigen::Vector3d& ref_point) {
  if (!node) {
    return;
  }

  Eigen::VectorXd eskf_state(6, 1);
  eskf_state << node->enu(0), node->enu(1), node->enu(2), node->orientation(0),
      node->orientation(1), node->orientation(2);
  if (!kalman_filter_.IsInitialized()) {
    kalman_blh_ = node->blh;
    kalman_filter_.SetInitialState(eskf_state);
  }
  Eigen::Vector3d kalman_enu = hmu::Geo::BlhToEnu(kalman_blh_, ref_point);
  kalman_filter_.SetEnuState(kalman_enu);

  double delta_t = 0.0;
  if (prev_global_valid_) {
    prev_global_node_mutex_.lock();
    delta_t = node->ticktime - prev_global_node_.ticktime;
    prev_global_node_mutex_.unlock();
  }

  kalman_filter_.Predict(delta_t, node->linear_vel_VRF(0),
                         node->linear_vel_VRF(1), node->linear_vel_VRF(2),
                         node->angular_velocity(2) / 57.3);
  kalman_filter_.MeasurementUpdate(eskf_state);

  const auto curr_state = kalman_filter_.GetState();
  kalman_enu << curr_state(0), curr_state(1), curr_state(2);
  kalman_blh_ = hmu::Geo::EnuToBlh(kalman_enu, ref_point);
  node->enu << curr_state(0), curr_state(1), curr_state(2);

  node->orientation(0) = curr_state(3);
  node->orientation(1) = curr_state(4);
  node->orientation(2) = curr_state(5);

  node->KF_kdiff = kalman_filter_.GetKydiff();
  node->KF_cov = kalman_filter_.GetP();
}

bool FusionCenter::GetGlobalPose(Context* const ctx) {
  if (!ref_init_ || !ctx) {
    HLOG_ERROR << "Ref_init fail or ctx is nullptr";
    return false;
  }
  if (params_.passthrough_ins) {
    ctx->global_node = ctx->ins_node;
    ctx->global_node.location_state = 5;
    ctx->global_node.state = true;
    return true;
  }

  const auto refpoint = Refpoint();
  Node fusion_node;
  {
    std::unique_lock<std::mutex> lock(fusion_deque_mutex_);
    if (fusion_deque_.empty()) {
      HLOG_WARN << "fusion deque is empty";
      return false;
    }
    // 判断融合队列的值是通过Ins、MM观测更新的才有效输出
    auto it = fusion_deque_.rbegin();
    for (; it != fusion_deque_.rend(); ++it) {
      if ((*it)->type != NodeType::NONE) {
        (*it)->enu = hmu::Geo::BlhToEnu((*it)->blh, refpoint);
        fusion_node = *((*it));
        break;
      }
    }

    if (it == fusion_deque_.rend()) {
      HLOG_ERROR << "not find valid output in fusion deque";
      return false;
    }
  }

  // 定位状态码赋值
  uint32_t loc_state = 0;
  loc_state = GetGlobalLocationState();
  ctx->global_node.location_state = loc_state;
  if (monitor_->MonitorFault()) {
    loc_state = FaultCodeAssign(loc_state);
  }

  const double diff_dr_fc = ctx->dr_node.ticktime - fusion_node.ticktime;
  if (diff_dr_fc < 1e-3) {
    ctx->global_node = fusion_node;
    ctx->global_node.location_state = loc_state;
    ctx->global_node.velocity = ctx->ins_node.velocity;
  } else {
    // DR实时补帧至最新
    Node refer_node = fusion_node;
    Node ni;
    {
      std::lock_guard<std::mutex> lock(dr_deque_mutex_);
      if (!Interpolate(refer_node.ticktime, dr_deque_, &ni)) {
        HLOG_ERROR << "interpolate dr output failed,fusion node time:"
                   << refer_node.ticktime
                   << ",ins_time:" << ctx->dr_node.ticktime;
        return false;
      }
    }
    // todo此处后期改成dr_node
    ctx->global_node = ctx->ins_node;
    // ctx->global_node = ctx->dr_node;
    // ctx->global_node.rtk_status = ctx->ins_node.rtk_status;
    ctx->global_node.location_state = loc_state;
    ctx->global_node.cov = fusion_node.cov;
#ifdef ISORIN
    CheckTriggerLocState(ctx);
#endif
    const auto& T_delta = Node2SE3(ni).inverse() * Node2SE3(ctx->dr_node);
    const auto& pose = Node2SE3(refer_node) * T_delta;
    ctx->global_node.enu = pose.translation();
    ctx->global_node.blh = hmu::Geo::EnuToBlh(ctx->global_node.enu, refpoint);
    ctx->global_node.orientation = pose.so3().log();
    ctx->global_node.quaternion = pose.so3().unit_quaternion();
    // ctx->global_node.velocity = ctx->ins_node.velocity;
    ctx->global_node.velocity =
        ctx->global_node.quaternion * ctx->dr_node.velocity;
  }

  // kf 使用dr 替换ins
  ctx->global_node.linear_vel_VRF = ctx->dr_node.velocity;
  ctx->global_node.angular_velocity =
      (ctx->dr_node.angular_velocity) * 180 / M_PI;
  ctx->global_node.ticktime = ctx->dr_node.ticktime;

  // KF滤波
  if (params_.smooth_outputs) {
    KalmanFiltering(&(ctx->global_node), refpoint);
  }
  ctx->global_node.blh = hmu::Geo::EnuToBlh(ctx->global_node.enu, refpoint);
  ctx->global_node.rtk_status = ctx->ins_node.rtk_status;
  monitor_->OnFc(ctx->global_node);
  prev_global_node_mutex_.lock();
  prev_global_node_ = ctx->global_node;
  prev_global_node_mutex_.unlock();
  if (!prev_global_valid_) {
    prev_global_valid_ = true;
  }

  ctx->global_node.state = true;
  return true;
}

uint32_t FusionCenter::FaultCodeAssign(uint32_t state) {
  uint32_t loc_state = state;
  // 普通故障
  if (monitor_->fault_state.pecep_lane_error) {
    loc_state = 123;
  }
  if (monitor_->fault_state.map_lane_error) {
    loc_state = 124;
  }
  if (monitor_->fault_state.fc_single_jump_error) {
    loc_state = 128;
  }
  if (monitor_->fault_state.fc_map_lane_match_error) {
    loc_state = 130;
  }
  return loc_state;
}

void FusionCenter::CheckTriggerLocState(Context* const ctx) {
  static bool enable_05 = true, enable_10 = true, enable_03 = true;
  static uint32_t last_state = 2;
  static double last_time_05 = -1, last_time_10 = -1, last_time_03 = -1;
  auto curr_state = ctx->global_node.location_state;
  auto curr_time = ctx->global_node.ticktime;
  if (FLAGS_map_service_mode != 1 || last_state != 2) {
    return;
  }
  if (curr_state != last_state && curr_state == 123 && enable_05) {
    // mapping trigger 无车道线
    HLOG_WARN << "curr_loc_state: " << curr_state
              << " Start to trigger dc 1005";
    GLOBAL_DC_TRIGGER.TriggerCollect(1005);
    enable_05 = false;
    last_time_05 = curr_time;
  } else if (curr_state != last_state && curr_state == 128 && enable_10) {
    // mapping trigger 定位结果跳变
    HLOG_WARN << "curr_loc_state: " << curr_state
              << " Start to trigger dc 1010";
    GLOBAL_DC_TRIGGER.TriggerCollect(1010);
    enable_10 = false;
    last_time_10 = curr_time;
  } else if (curr_state != last_state && curr_state == 130 && enable_03) {
    HLOG_WARN << "curr_loc_state: " << curr_state
              << " Start to trigger dc 1003";
    GLOBAL_DC_TRIGGER.TriggerCollect(1003);
    enable_03 = false;
    last_time_03 = curr_time;
  }
  last_state = curr_state;
  enable_05 = (curr_time - last_time_05) > 600;
  enable_10 = (curr_time - last_time_10) > 600;
  enable_03 = (curr_time - last_time_03) > 600;
}

uint32_t FusionCenter::GetGlobalLocationState() {
  static uint32_t state = 5;
  uint32_t search_cnt = 0;

  std::unique_lock<std::mutex> lock(fusion_deque_mutex_);
  // 进入loc=2条件
  if (state != 2) {
    for (auto it = fusion_deque_.rbegin(); it != fusion_deque_.rend(); ++it) {
      if ((*it)->type == NodeType::MM || (*it)->type == NodeType::INS_MM) {
        state = 2;
      }
      if ((++search_cnt) > params_.search_state_cnt) {
        break;
      }
    }
  } else {
    // 退出loc=2条件
    for (auto it = fusion_deque_.rbegin(); it != fusion_deque_.rend(); ++it) {
      if ((*it)->type == NodeType::MM || (*it)->type == NodeType::INS_MM) {
        break;
      }
      if ((++search_cnt) > params_.search_state_cnt) {
        state = 5;
        break;
      }
    }
  }

  return state;
}

bool FusionCenter::GetLocalPose(Context* const ctx) {
  if (!ctx) {
    return false;
  }
  if (!init_dr_) {
    HLOG_ERROR << "OnInitDR failed";
    return false;
  }

  ctx->local_node = ctx->dr_node;
  const auto local_pose =
      Node2SE3(init_dr_node_).inverse() * Node2SE3(ctx->dr_node);
  ctx->local_node.enu = local_pose.translation();
  ctx->local_node.orientation = local_pose.so3().log();
  ctx->local_node.quaternion = local_pose.so3().unit_quaternion();
  return true;
}

std::string FusionCenter::GetHdCurrLaneId(const Eigen::Vector3d& utm,
                                          double heading) {
  if (!(GLOBAL_HD_MAP)) {
    return "";
  }

  hozon::common::PointENU point;
  point.set_x(utm(0));
  point.set_y(utm(1));
  point.set_z(utm(2));

  double s = 0;
  double l = 0;
  hozon::hdmap::LaneInfoConstPtr nearest_lane;
  const int got_lane = GLOBAL_HD_MAP->GetNearestLaneWithHeading(
      point, 50, heading, M_PI / 4.0, &nearest_lane, &s, &l);
  if (got_lane != 0 || !nearest_lane) {
    return "";
  }

  return nearest_lane->id().id();
}

void FusionCenter::DebugDiffTxt(std::ofstream& diff_file,
                                const Node& compare_node,
                                const Node& ins_node) {
  const auto com_SO3 = Sophus::SO3d::exp(compare_node.orientation);
  const auto ins_SO3 = Sophus::SO3d::exp(ins_node.orientation);
  Eigen::Matrix3d com_rot = com_SO3.matrix();
  Eigen::Matrix3d ins_rot = ins_SO3.matrix();
  Eigen::Vector3d com_euler = com_rot.eulerAngles(0, 1, 2);
  Eigen::Vector3d ins_euler = ins_rot.eulerAngles(0, 1, 2);

  Eigen::Vector3d euler_diff = com_euler - ins_euler;
  Eigen::Vector3d enu_diff = compare_node.enu - ins_node.enu;

  if (euler_diff.x() > 3.1) {
    euler_diff.x() -= M_PI;
  } else if (euler_diff.x() < -3.1) {
    euler_diff.x() += M_PI;
  }

  if (euler_diff.y() > 3.1) {
    euler_diff.y() -= M_PI;
  } else if (euler_diff.y() < -3.1) {
    euler_diff.y() += M_PI;
  }

  if (euler_diff.z() > 3.1) {
    euler_diff.z() -= M_PI;
  } else if (euler_diff.z() < -3.1) {
    euler_diff.z() += M_PI;
  }

  euler_diff.x() = euler_diff.x() * 180 / M_PI;
  euler_diff.y() = euler_diff.y() * 180 / M_PI;
  euler_diff.z() = euler_diff.z() * 180 / M_PI;

  diff_file << std::fixed << std::setprecision(10) << compare_node.ticktime
            << " " << enu_diff.x() << " " << enu_diff.y() << " " << enu_diff.z()
            << " " << euler_diff.x() << " " << euler_diff.y() << " "
            << euler_diff.z() << std::endl;
}

double FusionCenter::OrientationToHeading(const Eigen::Vector3d& orientation) {
  const Sophus::SO3d& rot = Sophus::SO3d::exp(orientation);
  Eigen::Vector3d euler = Rot2Euler312(rot.matrix());
  euler = euler - ((euler.array() > M_PI).cast<double>() * 2.0 * M_PI).matrix();

  return 90.0 - euler.z() / M_PI * 180;
}

bool FusionCenter::FilterPoseEstimation(const Node& node) {
  std::unique_lock<std::mutex> lock_pe(pe_deque_mutex_);
  if (pe_deque_.empty() || dr_deque_.empty()) {
    return true;
  }

  bool flag = false;
  auto pe = pe_deque_.back();
  auto pe_dist = 0.0;
  std::unique_lock<std::mutex> lock_fusion(fusion_deque_mutex_);
  double pe_back_time = pe_deque_.back()->ticktime;
  for (auto it = pe_deque_.rbegin(); it != pe_deque_.rend(); ++it) {
    auto pe_diff = pe_back_time - (*it)->ticktime;
    if (pe_diff < -1.0e-3 || pe_diff > 1) {
      continue;
    }
    for (const auto& fusion : fusion_deque_) {
      if (fabs(fusion->ticktime - (*it)->ticktime) > 0.01 ||
          fusion->sys_status != 2) {
        continue;
      }
      auto last_pe_fusion_node = Node2SE3(*fusion).inverse() * Node2SE3((*it));
      auto curr_pe_fusion_enu = last_pe_fusion_node.translation();
      flag = std::fabs(curr_pe_fusion_enu(0)) <=
             params_.max_fc_pe_horizontal_dist_error;
      break;
    }
    if (flag) {
      pe = (*it);
      auto last_cur_rel_pose = Node2SE3(*pe).inverse() * Node2SE3(node);
      auto last_pe_enu = last_cur_rel_pose.translation();
      pe_dist = last_pe_enu(0);
      break;
    }
  }

  std::shared_ptr<Node> last_dr = nullptr, curr_dr = nullptr;
  if (flag) {
    std::unique_lock<std::mutex> lock_dr(dr_deque_mutex_);
    for (const auto& dr_deque : dr_deque_) {
      if (fabs(dr_deque->ticktime - pe->ticktime) < 0.01) {
        last_dr = std::make_shared<Node>(*dr_deque);
      }
      if (fabs(dr_deque->ticktime - node.ticktime) < 0.01) {
        curr_dr = std::make_shared<Node>(*dr_deque);
      }
    }
  }

  if (!last_dr || !curr_dr || !flag) {
    return true;
  }
  flag = curr_dr->ticktime > last_dr->ticktime;
  if (!flag) {
    return true;
  }
  auto last_pe_heading = OrientationToHeading(pe->orientation);
  auto curr_pe_heading = OrientationToHeading(node.orientation);

  auto last_dr_heading = OrientationToHeading(last_dr->orientation);
  auto curr_dr_heading = OrientationToHeading(curr_dr->orientation);

  auto last_dr_node = Node2SE3(*last_dr).inverse() * Node2SE3(*curr_dr);
  auto last_dr_enu = last_dr_node.translation();

  auto dr_dist = last_dr_enu(0);
  auto diff_heading = std::fabs(std::fabs(last_pe_heading - curr_pe_heading) -
                                std::fabs(last_dr_heading - curr_dr_heading));
  auto diff_dist = std::fabs(dr_dist - pe_dist);
  return diff_heading > 0 && diff_heading < params_.max_dr_pe_heading_error &&
         diff_dist > 0 && diff_dist < params_.max_dr_pe_horizontal_dist_error;
}
}  // namespace fc
}  // namespace loc
}  // namespace mp
}  // namespace hozon
