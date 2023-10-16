/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： dr_fusion.cc
 *   author     ： nihongjie
 *   date       ： 2023.09
 ******************************************************************************/
#include "modules/location/dr_fusion/lib/dr_fusion.h"

#include <yaml-cpp/yaml.h>
#include <iomanip>

#include <Sophus/se3.hpp>
#include <boost/filesystem.hpp>


#include "modules/util/include/util/geo.h"
#include "modules/util/include/util/temp_log.h"
#include "util/rviz_agent/rviz_agent.h"

namespace hozon {
namespace mp {
namespace loc {

namespace hmu = hozon::mp::util;
const char kNewestDrOdom[] = "/dr/fusion";

DrFusion::~DrFusion() {}

InsInitStatus DrFusion::Init(const std::string& dr_configfile) {
  boost::filesystem::path dr_path(dr_configfile);
  if (!boost::filesystem::exists(dr_path)) {
    return InsInitStatus::CONFIG_NOT_FOUND;
  }

  LoadConfigParams(dr_configfile);

  if (use_rviz_bridge_) {
    int ret = util::RvizAgent::Instance().Register<adsfi_proto::viz::Odometry>(
        kNewestDrOdom);
    if (ret < 0) {
      HLOG_ERROR << "not found:" << kNewestDrOdom;
    }
  }
  return InsInitStatus::OK;
}

void DrFusion::LoadConfigParams(const std::string& configfile) {
  YAML::Node config_parser = YAML::LoadFile(configfile);
  use_rviz_bridge_ = config_parser["use_rviz_bridge"].as<bool>();
  use_dr_ = config_parser["use_dr"].as<bool>();
  use_inspva_ = config_parser["use_inspva"].as<bool>();
}

void DrFusion::OnInspva(const hozon::localization::HafNodeInfo& inspva_node) {
  if (!inspva_node.is_valid()) {
    return;
  }
  std::unique_lock<std::mutex> lock(ins_mutex_);
  if (inspva_node.header().seq() <= latest_ins_node_.header().seq()) {
    return;
  }
  if (!ref_inspva_node_init_) {
    Eigen::Vector3d blh(inspva_node.pos_gcj02().x(), inspva_node.pos_gcj02().y(),
                        inspva_node.pos_gcj02().z());
    SetRefpoint(blh);
    Extract02InsNode(inspva_node, &ref_ins_node_);
    ref_inspva_node_init_ = true;
  }
  latest_ins_node_ = inspva_node;
}

void DrFusion::OnDr(const hozon::dead_reckoning::DeadReckoning& dr_node) {
  hozon::localization::HafNodeInfo haf_dr_node;
  DrNode2DrFusionNode(dr_node, &haf_dr_node);
  std::unique_lock<std::mutex> lock(dr_mutex_);
  if (haf_dr_node.header().seq() <= latest_dr_node_.header().seq()) {
    return;
  }
  latest_dr_node_ = haf_dr_node;
}

bool DrFusion::GetResult(hozon::localization::HafNodeInfo* const node) {
  if (!node) {
    HLOG_ERROR << "Get Dr Fusion result failed";
    return false;
  }
  if (!use_dr_ && !use_inspva_) {
    HLOG_ERROR << "Get Dr Fusion result failed";
    return false;
  }
  if (use_rviz_bridge_) {
    PublishTopic();
  }
  node->Clear();
  if (use_dr_) {
    std::unique_lock<std::mutex> lock(dr_mutex_);
    *node = latest_dr_node_;
  } else if (use_inspva_) {
    std::unique_lock<std::mutex> lock(ins_mutex_);
    *node = latest_ins_node_;
    node->set_type(hozon::localization::HafNodeInfo_NodeType_DR);
    node->set_is_valid(true);

    node->mutable_header()->set_seq(latest_ins_node_.header().seq());
    node->mutable_header()->set_frame_id("dr_fusion");
    node->mutable_header()->set_publish_stamp(
        latest_ins_node_.header().publish_stamp());

    Eigen::Vector3d blh(latest_ins_node_.pos_gcj02().x(),
                        latest_ins_node_.pos_gcj02().y(),
                        latest_ins_node_.pos_gcj02().z());
    auto enu = hmu::Geo::BlhToEnu(blh, ref_ins_node_.refpoint);
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
  if (!use_dr_ && !use_inspva_) {
    return false;
  }
  adsfi_proto::viz::Odometry odom;
  InsNode node;
  if (use_inspva_) {
    std::unique_lock<std::mutex> lock(ins_mutex_);
    if (!Extract02InsNode(latest_ins_node_, &node)) {
      return false;
    }
    odom.mutable_header()->set_frameid("ins_map");
  } else if (use_dr_) {
    std::unique_lock<std::mutex> lock(dr_mutex_);
    if (!Extract02InsNode(latest_dr_node_, &node)) {
      return false;
    }
    odom.mutable_header()->set_frameid("dr_map");
  }

  uint64_t sec = uint64_t(node.ticktime);
  uint64_t nsec = uint64_t((node.ticktime - sec) * 1e9);
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
  if (!node) {
    return false;
  }

  Eigen::Quaterniond q(
      origin_node.quaternion().w(), origin_node.quaternion().x(),
      origin_node.quaternion().y(), origin_node.quaternion().z());
  if (q.norm() < 1e-10) {
    return false;
  }

  node->seq = origin_node.header().seq();
  node->ticktime = origin_node.header().publish_stamp();

  node->refpoint = GetRefpoint();
  node->blh << origin_node.pos_gcj02().x(), origin_node.pos_gcj02().y(),
      origin_node.pos_gcj02().z();
  node->org_blh = node->blh;
  node->enu = hmu::Geo::BlhToEnu(node->blh, node->refpoint);
  node->orientation = Sophus::SO3d(q).log();
  node->velocity << origin_node.linear_velocity().x(),
      origin_node.linear_velocity().y(), origin_node.linear_velocity().z();
  node->linear_accel << origin_node.linear_acceleration().x(),
      origin_node.linear_acceleration().y(),
      origin_node.linear_acceleration().z();
  return true;
}

int DrFusion::DrFusionState() {
  if (!use_dr_ && !use_inspva_) {
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
  node->mutable_header()->set_seq(origin_node.header().seq());
  node->mutable_header()->set_publish_stamp(origin_node.gnss_timestamp());
  node->mutable_header()->set_frame_id("dr");

  node->mutable_mounting_error()->set_x(origin_node.mounting_error().x());
  node->mutable_mounting_error()->set_y(origin_node.mounting_error().y());
  node->mutable_mounting_error()->set_z(origin_node.mounting_error().z());

  node->mutable_pos_wgs()->set_x(
      origin_node.pose().pose_local().position().x());
  node->mutable_pos_wgs()->set_y(
      origin_node.pose().pose_local().position().y());
  node->mutable_pos_wgs()->set_z(
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
  node->set_heading(origin_node.pose().pose_local().heading());

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

  return true;
}

void DrFusion::SetRefpoint(const Eigen::Vector3d& blh) { refpoint_ = blh; }

Eigen::Vector3d DrFusion::GetRefpoint() const { return refpoint_; }

}  // namespace loc
}  // namespace mp
}  // namespace hozon
