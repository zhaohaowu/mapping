/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： dr_fusion.cc
 *   author     ： nihongjie
 *   date       ： 2023.09
 ******************************************************************************/
#include "modules/location/dr_fusion/lib/dr_fusion.h"
#include <yaml-cpp/yaml.h>

#include <iomanip>
#include <chrono>
#include <Sophus/se3.hpp>
#include <boost/filesystem.hpp>

#include "modules/util/include/util/geo.h"
#include "modules/util/include/util/mapping_log.h"
#include "util/rviz_agent/rviz_agent.h"


namespace hozon {
namespace mp {
namespace loc {

namespace hmu = hozon::mp::util;
const char* const kNewestDrOdom = "/dr/fusion";

DrInitStatus DrFusion::Init(const std::string& dr_configfile) {
  boost::filesystem::path dr_path(dr_configfile);
  if (!boost::filesystem::exists(dr_path)) {
    return DrInitStatus::CONFIG_NOT_FOUND;
  }

  LoadConfigParams(dr_configfile);

  if (use_rviz_bridge_) {
    int ret = util::RvizAgent::Instance().Register<adsfi_proto::viz::Odometry>(
        kNewestDrOdom);
    if (ret < 0) {
      HLOG_ERROR << "not found:" << kNewestDrOdom;
    }
  }
  return DrInitStatus::OK;
}

void DrFusion::LoadConfigParams(const std::string& configfile) {
  YAML::Node config_parser = YAML::LoadFile(configfile);
  use_rviz_bridge_ = config_parser["use_rviz_bridge"].as<bool>();
  use_dr_ = config_parser["use_dr"].as<bool>();
  use_ins_fusion_ = config_parser["use_ins_fusion"].as<bool>();
}

void DrFusion::OnInsFusion(
    const hozon::localization::HafNodeInfo& ins_fusion_node) {
  if (!ins_fusion_node.is_valid()) {
    return;
  }
  // std::unique_lock<std::mutex> lock(ins_fusion_mutex_);
  // seq not used on orin
  // if (ins_fusion_node.header().seq() <=
  //     latest_ins_fusion_node_.header().seq()) {
  //   return;
  // }
  if (!ref_ins_fusion_node_init_) {
    Eigen::Vector3d blh(ins_fusion_node.pos_gcj02().x(),
                        ins_fusion_node.pos_gcj02().y(),
                        ins_fusion_node.pos_gcj02().z());
    SetRefpoint(blh);
    Extract02InsNode(ins_fusion_node, &ref_ins_fusion_node_);
    ref_ins_fusion_node_init_ = true;
  }
  latest_ins_fusion_node_ = ins_fusion_node;
}

void DrFusion::OnDr(const hozon::dead_reckoning::DeadReckoning& dr_node) {
  hozon::localization::HafNodeInfo haf_dr_node;
  DrNode2DrFusionNode(dr_node, &haf_dr_node);
  // seq not used on orin
  std::unique_lock<std::mutex> lock(dr_mutex_);
  if (haf_dr_node.header().seq() <= latest_dr_node_.header().seq() ||
      haf_dr_node.header().data_stamp() <=
          latest_dr_node_.header().data_stamp()) {
    HLOG_WARN << "latest_dr_node \n" << latest_dr_node_.DebugString();
    HLOG_WARN << "haf_dr_node \n" << haf_dr_node.DebugString();
    return;
  }
  latest_dr_node_ = haf_dr_node;
}

bool DrFusion::GetResult(hozon::localization::HafNodeInfo* const node) {
  if (node == nullptr) {
    HLOG_ERROR << "Get Dr Fusion result failed";
    return false;
  }
  if (!use_dr_ && !use_ins_fusion_) {
    HLOG_ERROR << "Get Dr Fusion result failed";
    return false;
  }
  if (init_ == false) {
    return false;
  }
  if (use_rviz_bridge_) {
    PublishTopic();
  }
  node->Clear();
  if (use_dr_) {
    std::unique_lock<std::mutex> lock(dr_mutex_);
    *node = latest_dr_node_;
  } else if (use_ins_fusion_) {
    std::unique_lock<std::mutex> lock(ins_fusion_mutex_);
    *node = latest_ins_fusion_node_;
    node->set_type(hozon::localization::HafNodeInfo_NodeType_DR);
    node->set_is_valid(true);

    node->mutable_header()->set_seq(latest_ins_fusion_node_.header().seq());
    node->mutable_header()->set_frame_id("dr_fusion");
    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>
        tp = std::chrono::time_point_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now());
    node->mutable_header()->set_publish_stamp(tp.time_since_epoch().count() *
                                              1.0e-9);
    node->mutable_header()->set_gnss_stamp(
        latest_ins_fusion_node_.header().gnss_stamp());
    node->mutable_header()->set_data_stamp(
        latest_ins_fusion_node_.header().data_stamp());
    Eigen::Vector3d blh(latest_ins_fusion_node_.pos_gcj02().x(),
                        latest_ins_fusion_node_.pos_gcj02().y(),
                        latest_ins_fusion_node_.pos_gcj02().z());
    auto enu = hmu::Geo::BlhToEnu(blh, ref_ins_fusion_node_.refpoint);
    node->mutable_pos_gcj02()->set_x(enu[0]);
    node->mutable_pos_gcj02()->set_y(enu[1]);
    node->mutable_pos_gcj02()->set_z(enu[2]);
    node->set_valid_estimate(true);
  }

  return true;
}

bool DrFusion::PublishTopic() {
  if (!mp::util::RvizAgent::Instance().Ok()) {
    return false;
  }
  if (!use_dr_ && !use_ins_fusion_) {
    return false;
  }
  adsfi_proto::viz::Odometry odom;
  InsNode node;
  if (use_ins_fusion_) {
    std::unique_lock<std::mutex> lock(ins_fusion_mutex_);
    if (!Extract02InsNode(latest_ins_fusion_node_, &node)) {
      return false;
    }
    odom.mutable_header()->set_frameid("use_ins_fusion");
  } else if (use_dr_) {
    std::unique_lock<std::mutex> lock(dr_mutex_);
    if (!Extract02InsNode(latest_dr_node_, &node)) {
      return false;
    }
    odom.mutable_header()->set_frameid("use_dr");
  }

  const auto sec = static_cast<uint32_t>(node.ticktime);
  const auto nsec = static_cast<uint32_t>((node.ticktime - sec) * 1e9);
  odom.mutable_header()->mutable_timestamp()->set_sec(sec);
  odom.mutable_header()->mutable_timestamp()->set_nsec(nsec);
  odom.mutable_pose()->mutable_pose()->mutable_position()->set_x(node.enu(0));
  odom.mutable_pose()->mutable_pose()->mutable_position()->set_y(node.enu(1));
  odom.mutable_pose()->mutable_pose()->mutable_position()->set_z(0);

  odom.mutable_pose()->mutable_pose()->mutable_orientation()->set_x(
      node.orientation.x());
  odom.mutable_pose()->mutable_pose()->mutable_orientation()->set_y(
      node.orientation.y());
  odom.mutable_pose()->mutable_pose()->mutable_orientation()->set_z(
      node.orientation.z());
  for (int i = 0; i < 36; ++i) {
    odom.mutable_pose()->add_covariance(0.);
  }
  mp::util::RvizAgent::Instance().Publish(kNewestDrOdom, odom);
  return true;
}

bool DrFusion::Extract02InsNode(
    const hozon::localization::HafNodeInfo& origin_node, InsNode* const node) {
  if (node == nullptr) {
    return false;
  }

  node->seq = origin_node.header().seq();
  node->ticktime = origin_node.header().data_stamp();

  node->refpoint = GetRefpoint();
  node->blh << origin_node.pos_gcj02().x(), origin_node.pos_gcj02().y(),
      origin_node.pos_gcj02().z();
  node->org_blh = node->blh;
  node->enu = hmu::Geo::BlhToEnu(node->blh, node->refpoint);
  node->orientation << origin_node.attitude().x(), origin_node.attitude().y(),
      origin_node.attitude().z();
  node->velocity << origin_node.linear_velocity().x(),
      origin_node.linear_velocity().y(), origin_node.linear_velocity().z();
  node->linear_accel << origin_node.linear_acceleration().x(),
      origin_node.linear_acceleration().y(),
      origin_node.linear_acceleration().z();
  return true;
}

int DrFusion::DrFusionState() const {
  if (!use_dr_ && !use_ins_fusion_) {
    return -1;
  }
  if (use_dr_) {
    return 1;
  }
  return 2;
}

bool DrFusion::DrNode2DrFusionNode(
    const hozon::dead_reckoning::DeadReckoning& origin_node,
    hozon::localization::HafNodeInfo* const node) {
  if (!node) {
    return false;
  }

  const auto& mq = origin_node.pose().pose_local().quaternion();
  if (std::isnan(mq.w()) || std::isnan(mq.x()) || std::isnan(mq.y()) || std::isnan(mq.z())) {
    HLOG_WARN << "Dr_quaternion is nan";
    return false;
  }

  Eigen::Quaterniond q(origin_node.pose().pose_local().quaternion().w(),
                       origin_node.pose().pose_local().quaternion().x(),
                       origin_node.pose().pose_local().quaternion().y(),
                       origin_node.pose().pose_local().quaternion().z());
  if (q.norm() < 1e-10) {
    HLOG_ERROR << "Dr_HafNodeInfo quaternion(w,x,y,z) "
               << origin_node.pose().pose_local().quaternion().w() << ","
               << origin_node.pose().pose_local().quaternion().x() << ","
               << origin_node.pose().pose_local().quaternion().y() << ","
               << origin_node.pose().pose_local().quaternion().z();
    return false;
  }
  if (std::fabs(q.norm() - 1) > 1e-3) {
    HLOG_ERROR << "Dr_HafNodeInfo quaternion(w,x,y,z) "
               << origin_node.pose().pose_local().quaternion().w() << ","
               << origin_node.pose().pose_local().quaternion().x() << ","
               << origin_node.pose().pose_local().quaternion().y() << ","
               << origin_node.pose().pose_local().quaternion().z()
               << ",norm:" << q.norm();
    return false;
  }

  node->mutable_header()->set_seq(origin_node.header().seq());
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>
      tp = std::chrono::time_point_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now());
  node->mutable_header()->set_publish_stamp(tp.time_since_epoch().count() *
                                            1.0e-9);
  node->mutable_header()->set_gnss_stamp(origin_node.header().gnss_stamp());
  node->mutable_header()->set_data_stamp(origin_node.header().data_stamp());
  node->mutable_header()->set_frame_id("dr");
  node->set_valid_estimate(true);

  node->set_type(hozon::localization::HafNodeInfo_NodeType_DR);
  node->set_is_valid(true);

  node->mutable_mounting_error()->set_x(origin_node.mounting_error().x());
  node->mutable_mounting_error()->set_y(origin_node.mounting_error().y());
  node->mutable_mounting_error()->set_z(origin_node.mounting_error().z());

  node->mutable_pos_gcj02()->set_x(
      origin_node.pose().pose_local().position().x());
  node->mutable_pos_gcj02()->set_y(
      origin_node.pose().pose_local().position().y());
  node->mutable_pos_gcj02()->set_z(
      origin_node.pose().pose_local().position().z());

  node->mutable_quaternion()->set_x(
      origin_node.pose().pose_local().quaternion().x());
  node->mutable_quaternion()->set_y(
      origin_node.pose().pose_local().quaternion().y());
  node->mutable_quaternion()->set_z(
      origin_node.pose().pose_local().quaternion().z());
  node->mutable_quaternion()->set_w(
      origin_node.pose().pose_local().quaternion().w());

  node->mutable_attitude()->set_x(
      origin_node.pose().pose_local().euler_angle().x());
  node->mutable_attitude()->set_y(
      origin_node.pose().pose_local().euler_angle().y());
  node->mutable_attitude()->set_z(
      origin_node.pose().pose_local().euler_angle().z());
  // dr是逆时针（0-360），fc是顺时针（-180~180)
  // 局部坐标系是以x轴为0度，逆时针。
  auto heading = origin_node.pose().pose_local().heading();
  heading = heading > 180.0F ? heading - 360.0F : heading;
  while (heading < -180) {
    heading += 360;
  }
  while (heading > 180) {
    heading -= 360;
  }
  node->set_heading(heading);

  // node->mutable_pos_gcj02()->set_x(origin_node.pose().pose_local().gcj02().x());
  // node->mutable_pos_gcj02()->set_y(origin_node.pose().pose_local().gcj02().y());
  // node->mutable_pos_gcj02()->set_z(origin_node.pose().pose_local().gcj02().z());

  node->mutable_linear_velocity()->set_x(
      origin_node.velocity().twist_vrf().linear_vrf().x());
  node->mutable_linear_velocity()->set_y(
      origin_node.velocity().twist_vrf().linear_vrf().y());
  node->mutable_linear_velocity()->set_z(
      origin_node.velocity().twist_vrf().linear_vrf().z());

  node->mutable_angular_velocity()->set_x(
      origin_node.velocity().twist_vrf().angular_vrf().x());
  node->mutable_angular_velocity()->set_y(
      origin_node.velocity().twist_vrf().angular_vrf().y());
  node->mutable_angular_velocity()->set_z(
      origin_node.velocity().twist_vrf().angular_vrf().z());

  node->mutable_linear_acceleration()->set_x(
      origin_node.acceleration().linear_vrf().linear_raw_vrf().x());
  node->mutable_linear_acceleration()->set_y(
      origin_node.acceleration().linear_vrf().linear_raw_vrf().y());
  node->mutable_linear_acceleration()->set_z(
      origin_node.acceleration().linear_vrf().linear_raw_vrf().z());

  if (init_ == false) {
    init_ = true;
  }
  return true;
}

void DrFusion::SetRefpoint(const Eigen::Vector3d& blh) { refpoint_ = blh; }

Eigen::Vector3d DrFusion::GetRefpoint() const { return refpoint_; }

}  // namespace loc
}  // namespace mp
}  // namespace hozon
