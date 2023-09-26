/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： fusion_center.cc
 *   author     ： lilanxing
 *   date       ： 2023.09
 ******************************************************************************/

#include "modules/location/fusion_center/lib/fusion_center.h"

#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>

#include "modules/location/fusion_center/lib/eulerangle.h"
#include "modules/util/include/util/temp_log.h"
#include "modules/util/include/util/geo.h"

namespace hozon {
namespace mp {
namespace loc {
namespace fc {

namespace hmu = hozon::mp::util;

bool FusionCenter::Init(const std::string& configfile) {
  boost::filesystem::path path(configfile);
  if (!boost::filesystem::exists(path)) {
    HLOG_ERROR << "location fc conf:" << configfile << " not exist";
    return false;
  }
  if (!LoadParams(configfile)) {
    HLOG_ERROR << "location fc load params from " << configfile << " error";
    return false;
  }
  return true;
}

void FusionCenter::OnImu(const ImuIns& imuins) {
  if (!params_.recv_imu) {
    return;
  }
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

bool FusionCenter::GetCurrentOutput(Localization* const location) {
  if (!location) {
    return false;
  }
  if (!params_.passthrough_ins && coord_init_timestamp_ < 0) {
    return false;
  }

  Context ctx;
  ctx.imuins = curr_imuins_;
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
    location->set_location_state(5);
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
  node->ticktime = msg.header().publish_stamp();
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
                                    Localization* const location) {
  if (!location) {
    return;
  }

  location->Clear();

  const auto& ins = ctx.ins;
  const auto& imu = ctx.imuins.imu_info();
  const auto& fusion_node = ctx.fusion_node;
  const double ticktime = fusion_node.ticktime;

  auto* const header = location->mutable_header();
  header->set_seq(seq_++);
  header->set_frame_id("location");
  header->set_publish_stamp(ticktime);

  location->set_measurement_time(ticktime);

  location->set_gps_week(ins.gps_week());
  location->set_gps_sec(ins.gps_sec());
  location->set_received_ehp_counter(ehp_counter_);
  location->set_rtk_status(fusion_node.rtk_status);
  location->set_location_state(fusion_node.location_state);

  location->mutable_mounting_error()->set_x(ins.mounting_error().x());
  location->mutable_mounting_error()->set_y(ins.mounting_error().y());
  location->mutable_mounting_error()->set_z(ins.mounting_error().z());

  auto* const pose = location->mutable_pose();

  const Sophus::SO3d& rot = Sophus::SO3d::exp(fusion_node.orientation);
  pose->mutable_quaternion()->set_w(rot.unit_quaternion().w());
  pose->mutable_quaternion()->set_x(rot.unit_quaternion().x());
  pose->mutable_quaternion()->set_y(rot.unit_quaternion().y());
  pose->mutable_quaternion()->set_z(rot.unit_quaternion().z());

  Eigen::Vector3d euler = Rot2Euler312(rot.matrix()) * 180.0 / M_PI;
  euler = euler - ((euler.array() > 180.).cast<double>() * 360.0).matrix();
  pose->mutable_euler_angle()->set_x(euler.x());
  pose->mutable_euler_angle()->set_y(euler.y());
  pose->mutable_euler_angle()->set_z(euler.z());
  pose->mutable_euler_angles()->set_x(euler.x());
  pose->mutable_euler_angles()->set_y(euler.y());
  pose->mutable_euler_angles()->set_z(euler.z());

  pose->mutable_rotation_vrf()->set_x(rot.log().x());
  pose->mutable_rotation_vrf()->set_y(rot.log().y());
  pose->mutable_rotation_vrf()->set_z(rot.log().z());

  double heading = 90.0 - euler.z();
  if (heading < 0.0) {
    heading += 360.0;
  }
  pose->set_heading(heading);

  Eigen::Vector3d local_vel = rot.inverse() * fusion_node.velocity;
  pose->mutable_linear_velocity_vrf()->set_x(local_vel.x());
  pose->mutable_linear_velocity_vrf()->set_y(local_vel.y());
  pose->mutable_linear_velocity_vrf()->set_z(local_vel.z());

  pose->mutable_linear_acceleration_vrf()->set_x(ins.linear_acceleration().x());
  pose->mutable_linear_acceleration_vrf()->set_y(ins.linear_acceleration().y());
  pose->mutable_linear_acceleration_vrf()->set_z(ins.linear_acceleration().z());

  pose->mutable_angular_velocity_vrf()->set_x(ins.angular_velocity().x());
  pose->mutable_angular_velocity_vrf()->set_y(ins.angular_velocity().y());
  pose->mutable_angular_velocity_vrf()->set_z(ins.angular_velocity().z());

  pose->mutable_wgs()->set_x(ins.pos_wgs().x());
  pose->mutable_wgs()->set_y(ins.pos_wgs().y());
  pose->mutable_wgs()->set_z(ins.pos_wgs().z());

  pose->mutable_gcj02()->set_x(fusion_node.blh(0));
  pose->mutable_gcj02()->set_y(fusion_node.blh(1));
  pose->mutable_gcj02()->set_z(fusion_node.blh(2));

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

  pose->mutable_linear_acceleration_raw_vrf()->set_x(
      imu.imuvb_linear_acceleration().x());
  pose->mutable_linear_acceleration_raw_vrf()->set_y(
      imu.imuvb_linear_acceleration().y());
  pose->mutable_linear_acceleration_raw_vrf()->set_z(
      imu.imuvb_linear_acceleration().z());

  pose->mutable_linear_velocity()->set_x(fusion_node.velocity(0));
  pose->mutable_linear_velocity()->set_y(fusion_node.velocity(1));
  pose->mutable_linear_velocity()->set_z(fusion_node.velocity(2));

  pose->mutable_local_pose()->set_x(0);
  pose->mutable_local_pose()->set_y(1);
  pose->mutable_local_pose()->set_z(2);

  pose->mutable_angular_velocity_raw_vrf()->set_x(
      imu.imuvb_angular_velocity().x());
  pose->mutable_angular_velocity_raw_vrf()->set_y(
      imu.imuvb_angular_velocity().y());
  pose->mutable_angular_velocity_raw_vrf()->set_z(
      imu.imuvb_angular_velocity().z());

  Eigen::Matrix<float, 6, 1> diag;
  diag << ins.sd_position().x(), ins.sd_position().y(),
          ins.sd_position().z(), ins.sd_attitude().x(),
          ins.sd_attitude().y(), ins.sd_attitude().z();
  Eigen::Matrix<float, 6, 6, Eigen::RowMajor> sd = diag.asDiagonal();
  Eigen::Matrix<float, 6, 6, Eigen::RowMajor> cov = sd * sd;

  location->clear_covariance();
  for (int i = 0; i < 36; ++i) {
    location->add_covariance(cov(i));
  }
}

}  // namespace fc
}  // namespace loc
}  // namespace mp
}  // namespace hozon
