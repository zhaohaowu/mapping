/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： coord_adapter.cc
 *   author     ： lilanxing
 *   date       ： 2023.09
 ******************************************************************************/

#include "modules/location/coord_adapter/lib/coord_adapter.h"
#include <yaml-cpp/yaml.h>

#include <boost/filesystem.hpp>

#include "modules/location/common/data_verify.h"
#include "modules/location/common/stl_op.h"
#include "modules/util/include/util/temp_log.h"

namespace hozon {
namespace mp {
namespace loc {
namespace ca {

bool CoordAdapter::Init(const std::string& configfile) {
  boost::filesystem::path path(configfile);
  if (!boost::filesystem::exists(path)) {
    HLOG_ERROR << "location coord conf:" << configfile << " not exist";
    return false;
  }
  if (!LoadParams(configfile)) {
    HLOG_ERROR << "location coord load params from " << configfile << " error";
    return false;
  }
  return true;
}

void CoordAdapter::SetCoordInitTimestamp(double t) {
  coord_init_timestamp_ = t;
  if (sys_init_) {
    return;
  }

  if (!Interpolate(coord_init_timestamp_, dr_deque_, &init_dr_node_)) {
    return;
  }

  sys_init_ = true;
  init_raw_dr_ = curr_raw_dr_;
}

double CoordAdapter::GetCoordInitTimestamp() const {
  return coord_init_timestamp_;
}

bool CoordAdapter::IsCoordInitSucc() const { return sys_init_; }

HafNodeInfo CoordAdapter::GetSysInitDrFusion() const {
  HafNodeInfo dr = init_raw_dr_;

  dr.set_type(hozon::localization::HafNodeInfo_NodeType_DR);
  // replace publish time with init time
  dr.mutable_header()->set_publish_stamp(init_dr_node_.ticktime);
  dr.mutable_header()->set_gnss_stamp(init_dr_node_.ticktime);
  dr.mutable_header()->set_data_stamp(init_dr_node_.ticktime);
  dr.set_is_valid(true);

  dr.mutable_pos_gcj02()->set_x(init_dr_node_.enu(0));
  dr.mutable_pos_gcj02()->set_y(init_dr_node_.enu(1));
  dr.mutable_pos_gcj02()->set_z(init_dr_node_.enu(2));

  dr.mutable_attitude()->set_x(init_dr_node_.orientation(0));
  dr.mutable_attitude()->set_y(init_dr_node_.orientation(1));
  dr.mutable_attitude()->set_z(init_dr_node_.orientation(2));

  Eigen::AngleAxisd roll(dr.attitude().x(), Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch(dr.attitude().y(), Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw(dr.attitude().z(), Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond quat = yaw * roll * pitch;
  dr.mutable_quaternion()->set_x(static_cast<float>(quat.x()));
  dr.mutable_quaternion()->set_y(static_cast<float>(quat.y()));
  dr.mutable_quaternion()->set_z(static_cast<float>(quat.z()));
  dr.mutable_quaternion()->set_w(static_cast<float>(quat.w()));

  dr.mutable_linear_velocity()->set_x(init_dr_node_.velocity(0));
  dr.mutable_linear_velocity()->set_y(init_dr_node_.velocity(1));
  dr.mutable_linear_velocity()->set_z(init_dr_node_.velocity(2));

  return dr;
}

void CoordAdapter::OnLocalMap(const LocalMap& local_map) {
  if (!cm::HasValidHeader(local_map)) {
    return;
  }
  if (sys_init_) {
    return;
  }
  SetCoordInitTimestamp(local_map.init_timestamp());
}

void CoordAdapter::OnPerception(const TransportElement& percep) {
  curr_percep_ = percep;
}

void CoordAdapter::OnDrFusion(const HafNodeInfo& dr) {
  if (!cm::HasValidHeader(dr)) {
    return;
  }
  curr_raw_dr_ = dr;

  if (!dr_deque_.empty() &&
      fabs(dr_deque_.back()->ticktime - dr.header().gnss_stamp()) < 1e-3) {
    return;
  }

  Node node;
  if (!ConvertDrNode(dr, &node)) {
    return;
  }

  dr_deque_.emplace_back(std::make_shared<Node>(node));
  cm::ShrinkQueue(&dr_deque_, params_.dr_deque_capacity);
}

bool CoordAdapter::LoadParams(const std::string& configfile) {
  YAML::Node node = YAML::LoadFile(configfile);
  params_.dr_deque_capacity = node["dr_deque_capacity"].as<uint32_t>();

  return true;
}

bool CoordAdapter::ConvertDrNode(const HafNodeInfo& msg, Node* const node) {
  if (node == nullptr) {
    return false;
  }

  Eigen::Quaterniond q(msg.quaternion().w(), msg.quaternion().x(),
                       msg.quaternion().y(), msg.quaternion().z());
  if (q.norm() < 1e-10) {
    HLOG_WARN << "HafNodeInfo quaternion(w,x,y,z) (" << msg.quaternion().w()
              << "," << msg.quaternion().x() << "," << msg.quaternion().y()
              << "," << msg.quaternion().z() << ") error";
    return false;
  }
  node->ticktime = msg.header().gnss_stamp();
  node->enu << msg.pos_gcj02().x(), msg.pos_gcj02().y(), msg.pos_gcj02().z();
  node->orientation = Sophus::SO3d(q).log();
  node->velocity << msg.linear_velocity().x(), msg.linear_velocity().y(),
      msg.linear_velocity().z();
  return true;
}

bool CoordAdapter::IsInterpolable(const Node& n1, const Node& n2,
                                  double dis_tol, double ang_tol,
                                  double time_tol) {
  const double dis_delta = (n1.enu - n2.enu).norm();
  const double ang_delta = (Sophus::SO3d::exp(n1.orientation).inverse() *
                            Sophus::SO3d::exp(n2.orientation))
                               .log()
                               .norm();
  const double time_delta = fabs(n2.ticktime - n1.ticktime);
  return (dis_delta < dis_tol && ang_delta < ang_tol && time_delta < time_tol);
}

bool CoordAdapter::Interpolate(double ticktime,
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

  if (!IsInterpolable(*(d[i - 1]), *(d[i]), dis_tol, ang_tol, time_tol)) {
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
  node->velocity = (1 - ratio) * d[i - 1]->velocity + ratio * d[i]->velocity;
  node->ticktime = ticktime;

  return true;
}

Sophus::SE3d CoordAdapter::Node2SE3(const Node& node) {
  return {Sophus::SO3d::exp(node.orientation), node.enu};
}

Sophus::SE3d CoordAdapter::Node2SE3(const std::shared_ptr<Node>& node) {
  return Node2SE3(*node);
}

}  // namespace ca
}  // namespace loc
}  // namespace mp
}  // namespace hozon
