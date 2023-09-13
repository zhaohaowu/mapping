/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： fusion_center.cc
 *   author     ： lilanxing
 *   date       ： 2023.09
 ******************************************************************************/

#include "modules/location/fusion_center/lib/fusion_center.h"

#include <yaml-cpp/yaml.h>

#include "modules/location/fusion_center/lib/eulerangle.h"
#include "modules/util/include/util/temp_log.h"
#include "modules/util/include/util/geo.h"

namespace hozon {
namespace mp {
namespace loc {
namespace fc {

namespace hmu = hozon::mp::util;

FusionCenter::~FusionCenter() {}

bool FusionCenter::Init(const std::string& configfile) {
  if (!LoadParams(configfile)) {
    HLOG_ERROR << "location fc load params from " << configfile << " error";
    return false;
  }
  return true;
}

void FusionCenter::OnImu(const AlgIMU& imu) {
  if (!params_.recv_imu || !imu.is_valid()) {
    return;
  }

  const uint32_t seq = imu.header().seq();
  if (seq == prev_imu_.header().seq()) {
    return;
  }

  std::unique_lock<std::mutex> lock(imu_deque_mutex_);
  imu_deque_.emplace_back(std::make_shared<AlgIMU>(imu));
  ShrinkQueue(&imu_deque_, params_.imu_deque_max_size);
  prev_imu_ = imu;
  curr_imu_ = imu;
}

void FusionCenter::OnIns(const HafNodeInfo& ins) {
  if (!params_.recv_ins || !ins.valid_estimate()) {
    return;
  }

  if (ins.header().seq() == prev_raw_ins_.header().seq()) {
    return;
  }
  prev_raw_ins_ = ins;
  curr_raw_ins_ = ins;

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
  if (dr.header().seq() == prev_raw_dr_.header().seq()) {
    return;
  }
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

void FusionCenter::OnPoseEstimate(const HafNodeInfo& pe) {
  if (!ref_init_ || !params_.recv_pe || !pe.valid_estimate()) {
    return;
  }
  if (pe.header().seq() == prev_raw_pe_.header().seq()) {
    return;
  }
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

void FusionCenter::SetEhpCounter(uint32_t counter) { ehp_counter_ = counter; }

uint32_t FusionCenter::GetEhpCounter() const { return ehp_counter_; }

void FusionCenter::SetCoordInitTimestamp(double t) {
  coord_init_timestamp_ = t;
}

bool FusionCenter::GetCurrentOutput(AlgLocation* const location) {
  if (!location) {
    return false;
  }
  if (!params_.passthrough_ins && coord_init_timestamp_ < 0) {
    return false;
  }

  Context ctx;
  ctx.imu = curr_imu_;
  ctx.ins = curr_raw_ins_;
  {
    std::unique_lock<std::mutex> lock(ins_deque_mutex_);
    if (ins_deque_.empty()) {
      return false;
    }
    ctx.ins_node = *(ins_deque_.back());
  }

  if (params_.passthrough_ins) {
    ctx.fusion_node = ctx.ins_node;
    Node2AlgLocation(ctx, location);
    return true;
  }

  return false;
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
  if (!node) {
    return false;
  }

  Eigen::Quaterniond q(msg.quaternion().w(), msg.quaternion().x(),
                       msg.quaternion().y(), msg.quaternion().z());
  if (q.norm() < 1e-10) {
    HLOG_ERROR << SETPRECISION(11) << "HafNodeInfo quaternion(w,x,y,z) "
               << msg.quaternion().w() << "," << msg.quaternion().x() << ","
               << msg.quaternion().y() << "," << msg.quaternion().z()
               << " error";
    return false;
  }

  node->seq = msg.header().seq();
  node->ticktime = static_cast<double>(msg.header().timestamp().sec()) +
                   static_cast<double>(msg.header().timestamp().nsec()) * 1e-9;
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

  node->refpoint = Refpoint();
  node->enu = hmu::Geo::BlhToEnu(node->blh, node->refpoint);

  for (int i = 0; i < msg.covariance_size(); ++i) {
    node->cov(i) = msg.covariance()[i];
  }

  return true;
}

void FusionCenter::SetRefpoint(const Eigen::Vector3d& blh) {
  std::unique_lock<std::mutex> lock(refpoint_mutex_);
  refpoint_ = blh;
}

const Eigen::Vector3d FusionCenter::Refpoint() {
  std::unique_lock<std::mutex> lock(refpoint_mutex_);
  return refpoint_;
}

void FusionCenter::Node2AlgLocation(const Context& ctx,
                                    AlgLocation* const location) {
  if (!location) {
    return;
  }

  location->Clear();

  const auto& ins = ctx.ins;
  const auto& imu = ctx.imu;
  const auto& fusion_node = ctx.fusion_node;
  const double ticktime = fusion_node.ticktime;
  const uint64_t timestamp_us = static_cast<uint64_t>(ticktime * 1e6);
  const uint32_t sec = static_cast<uint32_t>(ticktime);
  const uint32_t nsec = static_cast<uint32_t>((ticktime - sec) * 1e9);

  auto* const header = location->mutable_header();
  header->set_seq(seq_++);
  header->set_frameid("location");
  header->mutable_timestamp()->set_sec(sec);
  header->mutable_timestamp()->set_nsec(nsec);
  header->mutable_gnssstamp()->set_sec(sec);
  header->mutable_gnssstamp()->set_sec(sec);

  location->set_timestamp_us(timestamp_us);
  location->set_is_valid(true);
  location->set_gps_week(ins.gps_week());
  location->set_gps_sec(ins.gps_sec());
  location->set_received_ehp_counter(ehp_counter_);
  location->set_coordinate_type(adsfi_proto::hz_Adsfi::CoordType::ENU_COORD);

  location->mutable_mounting_error()->set_x(ins.mounting_error().x());
  location->mutable_mounting_error()->set_y(ins.mounting_error().y());
  location->mutable_mounting_error()->set_z(ins.mounting_error().z());

  auto* const pose = location->mutable_pose();

  // pose_wgs
  auto* const pose_wgs = pose->mutable_pose_wgs();
  pose_wgs->mutable_position()->set_x(ins.pos_wgs().x());
  pose_wgs->mutable_position()->set_y(ins.pos_wgs().y());
  pose_wgs->mutable_position()->set_z(ins.pos_wgs().z());

  const Sophus::SO3d& rot = Sophus::SO3d::exp(fusion_node.orientation);
  pose_wgs->mutable_quaternion()->set_w(rot.unit_quaternion().w());
  pose_wgs->mutable_quaternion()->set_x(rot.unit_quaternion().x());
  pose_wgs->mutable_quaternion()->set_y(rot.unit_quaternion().y());
  pose_wgs->mutable_quaternion()->set_z(rot.unit_quaternion().z());

  Eigen::Vector3d euler = Rot2Euler312(rot.matrix()) * 180.0 / M_PI;
  euler = euler - ((euler.array() > 180.).cast<double>() * 360.0).matrix();
  pose_wgs->mutable_euler_angle()->set_x(euler.x());
  pose_wgs->mutable_euler_angle()->set_y(euler.y());
  pose_wgs->mutable_euler_angle()->set_z(euler.z());

  pose_wgs->mutable_rotation_vrf()->set_x(rot.log().x());
  pose_wgs->mutable_rotation_vrf()->set_y(rot.log().y());
  pose_wgs->mutable_rotation_vrf()->set_z(rot.log().z());

  double heading = 90.0 - euler.z();
  if (heading < 0.0) {
    heading += 360.0;
  }
  pose_wgs->set_heading(heading);

  // pose_gcj02
  auto* const pose_gcj02 = pose->mutable_pose_gcj02();
  *pose_gcj02 = *pose_wgs;
  pose_gcj02->mutable_position()->set_x(fusion_node.blh.x());
  pose_gcj02->mutable_position()->set_y(fusion_node.blh.y());
  pose_gcj02->mutable_position()->set_z(fusion_node.blh.z());

  const double zone = pose_gcj02->position().y() / 6.0 + 31;
  const uint32_t curr_zone = std::floor(zone);
  uint32_t near_zone = 0;
  if ((zone - curr_zone) > 0.5) {
    near_zone = curr_zone + 1;
  } else {
    near_zone = curr_zone - 1;
  }

  Eigen::Vector3d curr_utm;
  Eigen::Vector3d near_utm;
  hmu::Geo::LatLonToUtmXy(curr_zone, pose_gcj02->position().y(),
                          pose_gcj02->position().x(), &curr_utm);
  hmu::Geo::LatLonToUtmXy(near_zone, pose_gcj02->position().y(),
                          pose_gcj02->position().x(), &near_utm);
  auto* const pose_utm_01 = pose->mutable_pose_utm_01();
  auto* const pose_utm_02 = pose->mutable_pose_utm_02();
  *pose_utm_01 = *pose_gcj02;
  *pose_utm_02 = *pose_gcj02;
  pose_utm_01->mutable_position()->set_x(curr_utm(0));
  pose_utm_01->mutable_position()->set_y(curr_utm(1));
  pose_utm_02->mutable_position()->set_x(near_utm(0));
  pose_utm_02->mutable_position()->set_y(near_utm(1));

  pose->set_utm_zone_id_01(curr_zone);
  pose->set_utm_zone_id_02(near_zone);

  Eigen::Matrix<float, 6, 1> diag;
  diag << ins.sd_position().x(), ins.sd_position().y(), ins.sd_position().z(),
      ins.sd_attitude().x(), ins.sd_attitude().y(), ins.sd_attitude().z();
  Eigen::Matrix<float, 6, 6, Eigen::RowMajor> sd = diag.asDiagonal();
  Eigen::Matrix<float, 6, 6, Eigen::RowMajor> cov = sd * sd;

  pose->clear_std();
  for (int i = 0; i < 36; ++i) {
    pose->add_std(cov(i));
  }

  Eigen::Vector3d local_vel = rot.inverse() * fusion_node.velocity;
  auto* const velocity = location->mutable_velocity();
  velocity->mutable_twist_vrf()->mutable_linear_vrf()->set_x(local_vel.x());
  velocity->mutable_twist_vrf()->mutable_linear_vrf()->set_y(local_vel.y());
  velocity->mutable_twist_vrf()->mutable_linear_vrf()->set_z(local_vel.z());

  velocity->mutable_twist_vrf()->mutable_angular_vrf()->set_x(
      imu.imu_vb_angular_velocity().x());
  velocity->mutable_twist_vrf()->mutable_angular_vrf()->set_y(
      imu.imu_vb_angular_velocity().y());
  velocity->mutable_twist_vrf()->mutable_angular_vrf()->set_z(
      imu.imu_vb_angular_velocity().z());
  velocity->mutable_twist_vrf()->mutable_angular_raw_vrf()->set_x(
      imu.imu_vb_angular_velocity().x());
  velocity->mutable_twist_vrf()->mutable_angular_raw_vrf()->set_y(
      imu.imu_vb_angular_velocity().y());
  velocity->mutable_twist_vrf()->mutable_angular_raw_vrf()->set_z(
      imu.imu_vb_angular_velocity().z());

  diag << ins.sd_velocity().x(), ins.sd_velocity().y(), ins.sd_velocity().z(),
      0, 0, 0;
  sd = diag.asDiagonal();
  Eigen::Matrix<float, 6, 6> trans = Eigen::Matrix<float, 6, 6>::Identity();
  trans.block<3, 3>(0, 0) = rot.matrix().cast<float>();
  sd = trans * sd * trans.transpose();
  cov = sd * sd;

  velocity->clear_std();
  for (int i = 0; i < 36; ++i) {
    velocity->add_std(cov(i));
  }

  auto* const acceleration = location->mutable_acceleration();
  Eigen::Vector3d acc(imu.linear_acceleration().x(),
                      imu.linear_acceleration().y(),
                      imu.linear_acceleration().z());
  acc = acc - fusion_node.b_a;
  acc = rot.inverse() * acc;
  acceleration->mutable_linear_vrf()->mutable_linear_raw_vrf()->set_x(acc.x());
  acceleration->mutable_linear_vrf()->mutable_linear_raw_vrf()->set_y(acc.y());
  acceleration->mutable_linear_vrf()->mutable_linear_raw_vrf()->set_z(acc.z());

  acceleration->mutable_linear_vrf()->mutable_linear_vrf()->set_x(
      ins.linear_acceleration().x());
  acceleration->mutable_linear_vrf()->mutable_linear_vrf()->set_y(
      ins.linear_acceleration().y());
  acceleration->mutable_linear_vrf()->mutable_linear_vrf()->set_z(
      ins.linear_acceleration().z());

  acceleration->clear_std();
  for (int i = 0; i < 36; ++i) {
    acceleration->add_std(0);
  }

  location->set_rtk_status(fusion_node.rtk_status);
  location->set_location_state(fusion_node.location_state);
}

}  // namespace fc
}  // namespace loc
}  // namespace mp
}  // namespace hozon
