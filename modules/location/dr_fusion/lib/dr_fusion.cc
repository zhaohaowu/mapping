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
  use_fixed_quat_ = config_parser["use_fixed_quat"].as<bool>();
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
    Extract02InsNode(inspva_node, &ref_ins_node_);
    ref_inspva_node_init_ = true;
  }
  latest_ins_node_ = inspva_node;
}

void DrFusion::OnDr(const hozon::localization::HafNodeInfo& dr_node) {
  std::unique_lock<std::mutex> lock(dr_mutex_);
  if (dr_node.header().seq() <= latest_dr_node_.header().seq()) {
    return;
  }
  latest_dr_node_ = dr_node;
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

    if (use_fixed_quat_) {
      node->mutable_quaternion()->set_x(1.0);
      node->mutable_quaternion()->set_y(1.0);
      node->mutable_quaternion()->set_z(1.0);
      node->mutable_quaternion()->set_w(1.0);
    } else {
      // 弧度 roll pitch yaw
      Eigen::Matrix<double, 3, 1> attitude = Eigen::MatrixXd::Zero(3, 1);
      attitude(0) = latest_ins_node_.attitude().x() * M_PI / 180.0;
      attitude(1) = latest_ins_node_.attitude().y() * M_PI / 180.0;
      attitude(2) = latest_ins_node_.attitude().z() * M_PI / 180.0;

      Eigen::AngleAxisd roll(attitude[0], Eigen::Vector3d::UnitX());
      Eigen::AngleAxisd pitch(attitude[1], Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd yaw(attitude[2], Eigen::Vector3d::UnitZ());
      Eigen::Quaterniond quat = yaw * roll * pitch;
      node->mutable_quaternion()->set_x(quat.x());
      node->mutable_quaternion()->set_y(quat.y());
      node->mutable_quaternion()->set_z(quat.z());
      node->mutable_quaternion()->set_w(quat.w());
    }

    Eigen::Quaterniond q(node->quaternion().w(), node->quaternion().x(),
                         node->quaternion().y(), node->quaternion().z());
    auto orientation = Sophus::SO3d(q).log() - ref_ins_node_.orientation;
    node->mutable_attitude()->set_x(orientation[0]);
    node->mutable_attitude()->set_y(orientation[1]);
    node->mutable_attitude()->set_z(orientation[2]);

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

  node->blh << origin_node.pos_gcj02().x(), origin_node.pos_gcj02().y(),
      origin_node.pos_gcj02().z();
  node->org_blh = node->blh;
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

}  // namespace loc
}  // namespace mp
}  // namespace hozon
