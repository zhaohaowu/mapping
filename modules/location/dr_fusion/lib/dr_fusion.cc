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

bool DrFusion::OnDr(const hozon::dead_reckoning::DeadReckoning& dr_node,
                    hozon::localization::HafNodeInfo* const node) {
  static int dr_count = 0;
  hozon::localization::HafNodeInfo haf_dr_node;
  if (!DrNode2DrFusionNode(dr_node, &haf_dr_node)) {
    return false;
  }
  // seq not used on orin
  if (haf_dr_node.header().seq() <= latest_dr_node_.header().seq() ||
      haf_dr_node.header().data_stamp() <=
          latest_dr_node_.header().data_stamp()) {
    HLOG_WARN << "latest_dr_node \n" << latest_dr_node_.DebugString();
    HLOG_WARN << "haf_dr_node \n" << haf_dr_node.DebugString();
    return false;
  }
  latest_dr_node_ = haf_dr_node;
  *node = haf_dr_node;
  ++dr_count;
  if (dr_count >= 100) {
    dr_count = 0;
    HLOG_ERROR << "rev dr heartbeat";
  }
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
  return true;
}

void DrFusion::SetRefpoint(const Eigen::Vector3d& blh) { refpoint_ = blh; }

Eigen::Vector3d DrFusion::GetRefpoint() const { return refpoint_; }

}  // namespace loc
}  // namespace mp
}  // namespace hozon
