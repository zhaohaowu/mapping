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

#include "modules/util/include/util/temp_log.h"
#include "util/rviz_agent/rviz_agent.h"

namespace hozon {
namespace mp {
namespace loc {

namespace hlu = hozon::mp::util;
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
}

void DrFusion::OnInspva(const adsfi_proto::internal::HafNodeInfo& inspva_node) {
  if (!inspva_node.is_valid()) {
    return;
  }
  std::unique_lock<std::mutex> lock(ins_mutex_);
  if (inspva_node.header().seq() <= latest_ins_node_.header().seq()) {
    return;
  }
  latest_ins_node_ = inspva_node;
}

void DrFusion::OnDr(const adsfi_proto::internal::HafNodeInfo& dr_node) {
  std::unique_lock<std::mutex> lock(loc_dr_mutex_);
  if (dr_node.header().seq() <= latest_dr_node_.header().seq()) {
    return;
  }
  latest_dr_node_ = dr_node;
}

bool DrFusion::GetResult(adsfi_proto::internal::HafNodeInfo* const dr_node) {
  if (!dr_node) {
    HLOG_ERROR << "no dr_node";
    return false;
  }

  if (use_rviz_bridge_) {
    PublishTopic();
  }
  HLOG_ERROR << "has dr_node";
  dr_node->Clear();
  // this temp, dr fusion node will replace it late.
  std::unique_lock<std::mutex> lock(ins_mutex_);
  *dr_node = latest_ins_node_;

  return true;
}

bool DrFusion::PublishTopic() {
  if (!mp::util::RvizAgent::Instance().Ok()) {
    return false;
  }
  adsfi_proto::viz::Odometry odom;
  odom.mutable_header()->set_frameid("map");
  odom.mutable_header()->set_frameid("map");
  InsNode node;
  // this temp, dr fusion node will replace it late.
  std::unique_lock<std::mutex> lock(ins_mutex_);
  if (!Extract02InsNode(latest_ins_node_, &node)) {
    return false;
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
    const adsfi_proto::internal::HafNodeInfo& dr_node, InsNode* const node) {
  if (!node) {
    return false;
  }

  Eigen::Quaterniond q(dr_node.quaternion().w(), dr_node.quaternion().x(),
                       dr_node.quaternion().y(), dr_node.quaternion().z());
  if (q.norm() < 1e-10) {
    return false;
  }

  node->seq = dr_node.header().seq();
  node->ticktime = ToSeconds(dr_node.header().timestamp().sec(),
                             dr_node.header().timestamp().nsec());

  node->blh << dr_node.pos_gcj02().x(), dr_node.pos_gcj02().y(),
      dr_node.pos_gcj02().z();
  node->org_blh = node->blh;
  node->orientation = Sophus::SO3d(q).log();
  node->velocity << dr_node.linear_velocity().x(),
      dr_node.linear_velocity().y(), dr_node.linear_velocity().z();
  node->linear_accel << dr_node.linear_acceleration().x(),
      dr_node.linear_acceleration().y(), dr_node.linear_acceleration().z();
  return true;
}

double DrFusion::ToSeconds(const uint32_t& sec, const uint32_t& nsec) {
  return static_cast<double>(sec) + static_cast<double>(nsec) * 1e-9;
}

}  // namespace loc
}  // namespace mp
}  // namespace hozon
