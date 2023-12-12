/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： fusion_center.cc
 *   author     ： lilanxing
 *   date       ： 2023.09
 ******************************************************************************/

#include "modules/location/fusion_center/lib/fusion_center.h"
#include <yaml-cpp/yaml.h>

#include <boost/filesystem.hpp>

#include "modules/location/common/data_verify.h"
#include "modules/location/fusion_center/lib/eulerangle.h"
#include "modules/map_fusion/include/map_fusion/map_service/global_hd_map.h"
#include "modules/util/include/util/geo.h"
#include "modules/util/include/util/temp_log.h"

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
}

bool FusionCenter::Init(const std::string& configfile,
                        const std::string& filterconf,
                        const std::string& eskfconf) {
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

  th_fusion_ = std::make_shared<std::thread>(&FusionCenter::RunFusion, this);

  return true;
}

void FusionCenter::OnImu(const ImuIns& imuins) {
  if (!params_.recv_imu) {
    return;
  }
  // seq not used on orin
  // if (imuins.header().seq() == prev_imuins_.header().seq()) {
  //   return;
  // }

  std::unique_lock<std::mutex> lock(imuins_deque_mutex_);
  imuins_deque_.emplace_back(std::make_shared<ImuIns>(imuins));
  ShrinkQueue(&imuins_deque_, params_.imu_deque_max_size);
  prev_imuins_ = imuins;
  curr_imuins_ = imuins;
}

void FusionCenter::OnIns(const HafNodeInfo& ins) {
  if (!params_.recv_ins || !ins.valid_estimate()) {
    return;
  }
  // seq not used on orin
  // if (ins.header().seq() == prev_raw_ins_.header().seq()) {
  //   return;
  // }
  prev_raw_ins_ = ins;
  curr_raw_ins_ = ins;

  latest_ins_mutex_.lock();
  latest_ins_data_ = ins;
  latest_ins_mutex_.unlock();

  if (!ref_init_) {
    const Eigen::Vector3d refpoint(ins.pos_gcj02().x(), ins.pos_gcj02().y(),
                                   ins.pos_gcj02().z());
    SetRefpoint(refpoint);
    ref_init_ = true;
  }

  Node node;
  node.type = NodeType::INS;
  if (!ExtractBasicInfo(ins, &node)) {
    return;
  }

  std::unique_lock<std::mutex> lock(ins_deque_mutex_);
  ins_deque_.emplace_back(std::make_shared<Node>(node));
  ShrinkQueue(&ins_deque_, params_.ins_deque_max_size);
}

void FusionCenter::OnDR(const HafNodeInfo& dr) {
  if (!ref_init_ || !params_.recv_dr || !dr.valid_estimate()) {
    return;
  }
  // seq not used on orin
  // if (dr.header().seq() == prev_raw_dr_.header().seq()) {
  //   return;
  // }
  prev_raw_dr_ = dr;

  Node node;
  node.type = NodeType::DR;
  if (!ExtractBasicInfo(dr, &node)) {
    return;
  }
  std::unique_lock<std::mutex> lock(dr_deque_mutex_);
  dr_deque_.emplace_back(std::make_shared<Node>(node));
  ShrinkQueue(&dr_deque_, params_.dr_deque_max_size);
}

void FusionCenter::OnInitDR(const HafNodeInfo& initdr) {
  if (!cm::HasValidHeader(initdr)) {
    return;
  }
  if (coord_init_timestamp_ > 0) {
    return;
  }
  init_raw_dr_ = initdr;
  if (!ExtractBasicInfo(initdr, &init_dr_node_)) {
    HLOG_ERROR << "fc extract init dr info error";
    return;
  }
  coord_init_timestamp_ = init_dr_node_.ticktime;
}

void FusionCenter::OnPoseEstimate(const HafNodeInfo& pe) {
  if (!ref_init_ || !params_.recv_pe || !pe.valid_estimate()) {
    return;
  }
  // seq not used on orin
  // if (pe.header().seq() == prev_raw_pe_.header().seq()) {
  //   return;
  // }
  prev_raw_pe_ = pe;

  Node node;
  node.type = NodeType::POSE_ESTIMATE;
  if (!ExtractBasicInfo(pe, &node)) {
    return;
  }

  std::unique_lock<std::mutex> lock(pe_deque_mutex_);
  pe_deque_.emplace_back(std::make_shared<Node>(node));
  ShrinkQueue(&pe_deque_, params_.pe_deque_max_size);
}

void FusionCenter::SetEhpCounter(int32_t counter) { ehp_counter_ = counter; }

int32_t FusionCenter::GetEhpCounter() const { return ehp_counter_; }

void FusionCenter::SetCoordInitTimestamp(double t) {
  coord_init_timestamp_ = t;
}

bool FusionCenter::GetCurrentOutput(Localization* const location) {
  if (location == nullptr) {
    return false;
  }
  Context ctx;
  if (!GetCurrentContext(&ctx)) {
    HLOG_INFO << "get context failed";
    return false;
  }

  Node2Localization(ctx, location);
  return true;
}

bool FusionCenter::GetCurrentContext(Context* const ctx) {
  if (ctx == nullptr || ins_deque_.empty() || dr_deque_.empty()) {
    return false;
  }

  ctx->imuins = curr_imuins_;
  {
    std::unique_lock<std::mutex> lock(latest_ins_mutex_);
    ctx->ins = latest_ins_data_;
  }
  {
    std::unique_lock<std::mutex> lock(ins_deque_mutex_);
    ctx->ins_node = *(ins_deque_.back());
  }
  {
    std::unique_lock<std::mutex> lock(dr_deque_mutex_);
    ctx->dr_node = *(dr_deque_.back());
  }

  if (!GetLocalPose(ctx)) {
    HLOG_ERROR << "get loca pose failed";
    if (params_.require_local_pose) {
      return false;
    }
  }

  if (!GetGlobalPose(ctx)) {
    HLOG_ERROR << "get global pose failed";
  } else {
    ctx->global_node.location_state = GetGlobalLocationState();
  }

  return true;
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
  params_.no_mm_min_time = node["no_mm_min_time"].as<double>();
  params_.no_mm_max_time = node["no_mm_max_time"].as<double>();
  params_.use_dr_measurement = node["use_dr_measurement"].as<bool>();
  params_.run_fusion_interval_ms =
      node["run_fusion_interval_ms"].as<uint32_t>();
  params_.window_size = node["window_size"].as<uint32_t>();
  params_.require_local_pose = node["require_local_pose"].as<bool>();

  params_.ins_init_status.clear();
  const auto& init_status_node = node["ins_init_status"];
  for (const auto& init_status : init_status_node) {
    const auto sys_status = init_status["sys_status"].as<uint32_t>();
    const auto gps_status = init_status["gps_status"].as<uint32_t>();
    params_.ins_init_status.emplace_back(sys_status, gps_status);
  }
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

bool FusionCenter::ExtractBasicInfo(const HafNodeInfo& msg, Node* const node) {
  if (node == nullptr) {
    return false;
  }

  Eigen::Quaterniond q(msg.quaternion().w(), msg.quaternion().x(),
                       msg.quaternion().y(), msg.quaternion().z());
  if (q.norm() < 1e-10) {
    HLOG_ERROR << "HafNodeInfo quaternion(w,x,y,z) " << msg.quaternion().w()
               << "," << msg.quaternion().x() << "," << msg.quaternion().y()
               << "," << msg.quaternion().z() << " error";
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
    // Init
    if (!init_ins_ && !PoseInit()) {
      usleep(params_.run_fusion_interval_ms * 1000);
      continue;
    }

    // ESKF
    bool meas_flag = GenerateNewESKFMeas();
    bool pre_flag = GenerateNewESKFPre();

    if (pre_flag && meas_flag) {
      RunESKFFusion();
    }

    can_output_ = true;
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
  const double ticktime = fusion_deque_.front()->ticktime - 0.05;
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
  header->set_gnss_stamp(ticktime);

  location->set_measurement_time(ticktime);

  location->set_gps_week(ins.gps_week());
  location->set_gps_sec(ins.gps_sec());
  location->set_received_ehp_counter(ehp_counter_);
  location->set_rtk_status(global_node.rtk_status);
  location->set_location_state(global_node.location_state);

  location->mutable_mounting_error()->set_x(
      static_cast<float>(ins.mounting_error().x()));
  location->mutable_mounting_error()->set_y(
      static_cast<float>(ins.mounting_error().y()));
  location->mutable_mounting_error()->set_z(
      static_cast<float>(ins.mounting_error().z()));

  auto* const pose = location->mutable_pose();

  const Sophus::SO3d& rot = Sophus::SO3d::exp(global_node.orientation);
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

  Eigen::Vector3d vehicle_vel = rot.inverse() * global_node.velocity;
  pose->mutable_linear_velocity_vrf()->set_x(vehicle_vel.x());
  pose->mutable_linear_velocity_vrf()->set_y(vehicle_vel.y());
  pose->mutable_linear_velocity_vrf()->set_z(vehicle_vel.z());

  pose->mutable_linear_acceleration_vrf()->set_x(ins.linear_acceleration().x());
  pose->mutable_linear_acceleration_vrf()->set_y(ins.linear_acceleration().y());
  pose->mutable_linear_acceleration_vrf()->set_z(ins.linear_acceleration().z());

  pose->mutable_angular_velocity_vrf()->set_x(ins.angular_velocity().x());
  pose->mutable_angular_velocity_vrf()->set_y(ins.angular_velocity().y());
  pose->mutable_angular_velocity_vrf()->set_z(ins.angular_velocity().z());

  pose->mutable_wgs()->set_x(ins.pos_wgs().x());
  pose->mutable_wgs()->set_y(ins.pos_wgs().y());
  pose->mutable_wgs()->set_z(ins.pos_wgs().z());

  pose->mutable_gcj02()->set_x(global_node.blh(0));
  pose->mutable_gcj02()->set_y(global_node.blh(1));
  pose->mutable_gcj02()->set_z(global_node.blh(2));

  const double zone = pose->gcj02().y() / 6.0 + 31;
  const uint32_t curr_zone = std::floor(zone);
  uint32_t near_zone = 0;
  if ((zone - curr_zone) > 0.5) {
    near_zone = curr_zone + 1;
  } else {
    near_zone = curr_zone - 1;
  }

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
  const std::string laneid = GetHdCurrLaneId(curr_utm, ConvertHeading(heading));
  location->set_laneid(laneid);

  pose->mutable_linear_acceleration_raw_vrf()->set_x(
      imu.imuvb_linear_acceleration().x());
  pose->mutable_linear_acceleration_raw_vrf()->set_y(
      imu.imuvb_linear_acceleration().y());
  pose->mutable_linear_acceleration_raw_vrf()->set_z(
      imu.imuvb_linear_acceleration().z());

  pose->mutable_linear_velocity()->set_x(global_node.velocity(0));
  pose->mutable_linear_velocity()->set_y(global_node.velocity(1));
  pose->mutable_linear_velocity()->set_z(global_node.velocity(2));

  pose->mutable_angular_velocity_raw_vrf()->set_x(
      imu.imuvb_angular_velocity().x());
  pose->mutable_angular_velocity_raw_vrf()->set_y(
      imu.imuvb_angular_velocity().y());
  pose->mutable_angular_velocity_raw_vrf()->set_z(
      imu.imuvb_angular_velocity().z());

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
  pose_local->set_local_heading(local_node.heading);

  Eigen::Matrix<float, 6, 1> diag;
  diag << static_cast<float>(ins.sd_position().x()),
      static_cast<float>(ins.sd_position().y()),
      static_cast<float>(ins.sd_position().z()),
      static_cast<float>(ins.sd_attitude().x()),
      static_cast<float>(ins.sd_attitude().y()),
      static_cast<float>(ins.sd_attitude().z());
  Eigen::Matrix<float, 6, 6, Eigen::RowMajor> sd = diag.asDiagonal();
  Eigen::Matrix<float, 6, 6, Eigen::RowMajor> cov = sd * sd;

  location->clear_covariance();
  for (int i = 0; i < 36; ++i) {
    location->add_covariance(cov(i));
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
    return false;
  }

  if (!IsInterpolable(d[i - 1], d[i], dis_tol, ang_tol, time_tol)) {
    return false;
  }

  Sophus::SE3d p1 = Node2SE3(d[i - 1]);
  Sophus::SE3d p2 = Node2SE3(d[i]);
  Sophus::SE3d delta = p1.inverse() * p2;

  double ratio =
      (ticktime - d[i - 1]->ticktime) / (d[i]->ticktime - d[i - 1]->ticktime);
  if (std::isnan(ratio) || std::isinf(ratio)) {
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

bool FusionCenter::PoseInit() {
  // 使用ins初始化(if--->第一次初始化，else--->后续异常初始化)
  if (fusion_deque_.empty()) {
    std::unique_lock<std::mutex> lock(ins_deque_mutex_);
    for (auto iter = ins_deque_.rbegin(); iter != ins_deque_.rend(); ++iter) {
      if (AllowInit((*iter)->sys_status, (*iter)->rtk_status)) {
        (*iter)->enu = hmu::Geo::BlhToEnu((*iter)->blh, (*iter)->refpoint);
        InsertESKFFusionNode(**iter);
        init_ins_ = true;
        return true;
      }
    }
    return false;
  }

  fusion_deque_mutex_.lock();
  double latest_fc_ticktime = fusion_deque_.back()->ticktime;
  fusion_deque_mutex_.unlock();
  fusion_deque_.clear();

  std::unique_lock<std::mutex> lock(ins_deque_mutex_);
  for (auto iter = ins_deque_.rbegin(); iter != ins_deque_.rend(); ++iter) {
    if (AllowInit((*iter)->sys_status, (*iter)->rtk_status) &&
        (*iter)->ticktime > latest_fc_ticktime) {
      (*iter)->enu = hmu::Geo::BlhToEnu((*iter)->blh, (*iter)->refpoint);
      InsertESKFFusionNode(**iter);
      init_ins_ = true;
      return true;
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
        pre_deque_.push_back(ins_node);
        pre_flag = true;
      }
    }
  }

  return pre_flag;
}

bool FusionCenter::GenerateNewESKFMeas() {
  // 进行优势传感器的测量数据收集（加入各种判断条件，如MM不工作时用INS）
  fusion_deque_mutex_.lock();
  double cur_ticktime = fusion_deque_.back()->ticktime;
  fusion_deque_mutex_.unlock();
  bool meas_flag = false;
  meas_deque_.clear();

  // 1.1 MM工作时，MM测量加入
  {
    std::unique_lock<std::mutex> lock(pe_deque_mutex_);
    for (const auto& mm_node : pe_deque_) {
      if (mm_node->ticktime > cur_ticktime &&
          mm_node->ticktime > last_meas_time_) {
        meas_deque_.emplace_back(mm_node);
        meas_flag = true;
      }
    }
  }

  // 1.2 MM不工作时
  if (!meas_flag) {
    latest_ins_mutex_.lock();
    double ins_ticktime = latest_ins_data_.header().publish_stamp();
    latest_ins_mutex_.unlock();
    double time_diff = ins_ticktime - cur_ticktime;

    // (1)INS补帧MM测量
    pe_deque_mutex_.lock();
    int mm_size = pe_deque_.size();
    pe_deque_mutex_.unlock();
    if (params_.use_ins_predict_mm && mm_size > 0 &&
        time_diff > params_.no_mm_min_time &&
        time_diff < params_.no_mm_max_time) {
      if (PredictMMMeas()) {
        meas_flag = true;
        HLOG_ERROR << "No MM measurement,predict MM meas. time_diff:"
                   << time_diff;
      }

      // (2)INS测量加入
    } else if (mm_size == 0 || time_diff >= params_.no_mm_max_time) {
      std::unique_lock<std::mutex> lock(ins_deque_mutex_);
      for (const auto& ins_node : ins_deque_) {
        if (ins_node->ticktime > cur_ticktime &&
            ins_node->ticktime > last_meas_time_ &&
            AllowInsMeas(ins_node->sys_status, ins_node->rtk_status)) {
          meas_deque_.emplace_back(ins_node);
          meas_flag = true;
        }
      }
      HLOG_INFO << "No MM measurement,insert INS meas. time_diff:" << time_diff;
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

  // 2.50ms一个，根据INS测量递推出MM测量
  double now_ticktime = mm_ticktime;
  ins_deque_mutex_.lock();
  for (const auto& ins_node : ins_deque_) {
    if (ins_node->ticktime - now_ticktime > 0.1) {
      std::shared_ptr<Node> node = ins_node;
      const auto& T_delta =
          Node2SE3(*ins_refer).inverse() * Node2SE3(*ins_node);
      const auto& pose = Node2SE3(*mm_refer) * T_delta;
      node->enu = pose.translation();
      node->orientation = pose.so3().log();
      node->type = NodeType::POSE_ESTIMATE;
      meas_deque_.emplace_back(node);
      now_ticktime = ins_node->ticktime;
    }
  }
  ins_deque_mutex_.unlock();

  if (meas_deque_.size() > 0) {
    HLOG_ERROR << "Success,meas_deque_.size():" << meas_deque_.size();
    return true;
  }

  return false;
}

void FusionCenter::InsertESKFFusionNode(const Node& node) {
  auto new_node = std::make_shared<Node>();
  *new_node = node;
  fusion_deque_mutex_.lock();
  fusion_deque_.emplace_back(new_node);
  fusion_deque_mutex_.unlock();
}

void FusionCenter::RunESKFFusion() {
  HLOG_INFO << "-------eskf前-------"
            << "pre_deque_.size():" << pre_deque_.size()
            << ", meas_deque_.size():" << meas_deque_.size()
            << ", meas_type:" << meas_deque_.back()->type
            << ", eskf_fusion_deque_.size():" << fusion_deque_.size();

  // eskf开始
  // debug
  // static std::ofstream measured_file(data_file_path_ + "/mm_measurement.txt",
  //                                    std::ios::trunc);
  fusion_deque_mutex_.lock();
  eskf_->StateInit(fusion_deque_.back());
  fusion_deque_mutex_.unlock();
  while (!pre_deque_.empty() && !meas_deque_.empty()) {
    Node meas_node = *meas_deque_.front();
    Node predict_node = *pre_deque_.front();

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
      // debug
      // SaveTUMPose(measured_file, meas_node.quaternion, meas_node.enu,
      //             meas_node.ticktime);
    }

    State state = eskf_->GetState();
    InsertESKFFusionNode(State2Node(state));
  }

  HLOG_INFO << "-------eskf后-------"
            << "pre_deque_.size():" << pre_deque_.size()
            << ", meas_deque_.size():" << meas_deque_.size()
            << ", meas_type:" << meas_deque_.back()->type
            << ", eskf_fusion_deque_.size():" << fusion_deque_.size();
}

Node FusionCenter::State2Node(const State& state) {
  Node node;
  const auto refpoint = Refpoint();

  node.refpoint = refpoint;
  node.ticktime = state.ticktime;
  node.type = state.meas_type;
  node.enu = state.p;
  node.velocity = state.v;
  node.b_g = state.b_g;
  node.b_a = state.b_a;
  node.quaternion = state.R.unit_quaternion();
  node.orientation = state.R.log();

  node.sys_status = state.sys_status;
  node.rtk_status = state.rtk_status;

  return node;
}

bool FusionCenter::AllowInsMeas(uint32_t sys_status, uint32_t rtk_status) {
  if ((sys_status == 2 && (rtk_status == 4 || rtk_status == 5)) ||
      (sys_status == 3 && rtk_status == 3)) {
    return true;
  }
  return false;
}

bool FusionCenter::AllowInit(uint32_t sys_status, uint32_t rtk_status) {
  for (const auto& status : params_.ins_init_status) {
    if (sys_status == status.first && rtk_status == status.second) {
      return true;
    }
  }
  return false;
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
      std::shared_ptr<Node> node = ins_node;
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

void FusionCenter::KalmanFiltering(Node* const node) {
  if (!node) {
    return;
  }

  Eigen::VectorXd state(4, 1);
  state << node->enu(0), node->enu(1), node->enu(2),
      Rad2TwoPi(node->orientation(2));

  if (!kalman_filter_.IsInitialized()) {
    kalman_filter_.SetInitialState(state);
  }

  double delta_t = 0.0;
  if (prev_global_valid_) {
    delta_t = node->ticktime - prev_global_node_.ticktime;
  }

  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(4, 4);
  kalman_filter_.SetF(F);

  kalman_filter_.Predict(delta_t, node->velocity(0), node->velocity(1),
                         node->velocity(2), node->angular_velocity(2));
  kalman_filter_.MeasurementUpdate(state);

  const auto curr_state = kalman_filter_.GetState();
  node->enu << curr_state(0), curr_state(1), curr_state(2);
  if (params_.use_smooth_yaw) {
    double yaw_deg = curr_state(3);
    if (yaw_deg > 360.0) {
      yaw_deg -= 360.0;
    } else if (yaw_deg < 0.0) {
      yaw_deg += 360.0;
    }
    node->orientation(2) = TwoPi2Rad(yaw_deg);
  }
}

bool FusionCenter::GetGlobalPose(Context* const ctx) {
  if (!ctx || fusion_deque_.empty()) {
    return false;
  }

  if (params_.passthrough_ins) {
    ctx->global_node = ctx->ins_node;
    return true;
  }

  const auto refpoint = Refpoint();

  fusion_deque_mutex_.lock();
  const auto fusion_node = *(fusion_deque_.back());
  fusion_deque_mutex_.unlock();

  const double ins_fusion_diff = ctx->ins_node.ticktime - fusion_node.ticktime;
  if (ins_fusion_diff < 0) {
    HLOG_ERROR << "ins time behind newest fusion time, something wrong";
    return false;
  }
  if (ins_fusion_diff < 1e-3) {
    ctx->global_node = fusion_node;
    return true;
  }

  Node refer_node = fusion_node;

  Node ni;
  {
    std::lock_guard<std::mutex> lock(ins_deque_mutex_);
    if (!Interpolate(refer_node.ticktime, ins_deque_, &ni)) {
      HLOG_ERROR << "interpolate ins output failed";
      return false;
    }
  }

  ctx->global_node = ctx->ins_node;
  const auto& T_delta = Node2SE3(ni).inverse() * Node2SE3(ctx->ins_node);
  const auto& pose = Node2SE3(refer_node) * T_delta;
  ctx->global_node.enu = pose.translation();
  ctx->global_node.orientation = pose.so3().log();

  // KF滤波
  if (params_.smooth_outputs) {
    KalmanFiltering(&(ctx->global_node));
  }
  ctx->global_node.blh =
      hmu::Geo::EnuToBlh(ctx->global_node.enu, ctx->global_node.refpoint);
  ctx->global_node.rtk_status = ctx->ins_node.rtk_status;

  prev_global_node_ = ctx->global_node;
  if (!prev_global_valid_) {
    prev_global_valid_ = true;
  }

  return true;
}

uint32_t FusionCenter::GetGlobalLocationState() {
  uint32_t state = 5;
  uint32_t search_cnt = 0;

  std::unique_lock<std::mutex> lock(fusion_deque_mutex_);
  for (auto it = fusion_deque_.rbegin(); it != fusion_deque_.rend(); ++it) {
    if ((*it)->type == NodeType::POSE_ESTIMATE) {
      state = 2;
      return state;
    }
    if ((++search_cnt) > params_.search_state_cnt) {
      break;
    }
  }

  return state;
}

bool FusionCenter::GetLocalPose(Context* const ctx) {
  if (!ctx) {
    return false;
  }
  if (coord_init_timestamp_ < 0) {
    HLOG_ERROR << "local coordinate does not init yet";
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

}  // namespace fc
}  // namespace loc
}  // namespace mp
}  // namespace hozon
