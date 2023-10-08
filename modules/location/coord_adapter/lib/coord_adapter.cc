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
#include "modules/location/common/interpolate.h"
#include "modules/location/common/stl_op.h"
#include "modules/location/coord_adapter/lib/defines.h"
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
  dr.mutable_quaternion()->set_x(quat.x());
  dr.mutable_quaternion()->set_y(quat.y());
  dr.mutable_quaternion()->set_z(quat.z());
  dr.mutable_quaternion()->set_w(quat.w());

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
      fabs(dr_deque_.back()->ticktime - dr.header().publish_stamp()) < 1e-3) {
    return;
  }

  cm::BaseNode node;
  if (!ConvertDrNode(dr, &node)) {
    return;
  }

  dr_deque_.emplace_back(std::make_shared<cm::BaseNode>(node));
  cm::ShrinkQueue(&dr_deque_, params_.dr_deque_capacity);
}

bool CoordAdapter::LoadParams(const std::string& configfile) {
  YAML::Node node = YAML::LoadFile(configfile);
  params_.dr_deque_capacity = node["dr_deque_capacity"].as<uint32_t>();

  return true;
}

bool CoordAdapter::ConvertDrNode(const HafNodeInfo& msg,
                                 cm::BaseNode* const node) {
  if (!node) {
    return false;
  }

  Eigen::Quaterniond q(msg.quaternion().w(), msg.quaternion().x(),
                       msg.quaternion().y(), msg.quaternion().z());
  if (q.norm() < 1e-10) {
    HLOG_WARN << SETPRECISION(11) << "HafNodeInfo quaternion(w,x,y,z) ("
              << msg.quaternion().w() << "," << msg.quaternion().x() << ","
              << msg.quaternion().y() << "," << msg.quaternion().z()
              << ") error";
    return false;
  }

  node->ticktime = msg.header().publish_stamp();
  node->enu << msg.pos_gcj02().x(), msg.pos_gcj02().y(), msg.pos_gcj02().z();
  node->orientation = Sophus::SO3d(q).log();
  node->velocity << msg.linear_velocity().x(), msg.linear_velocity().y(),
      msg.linear_velocity().z();
  return true;
}

}  // namespace ca
}  // namespace loc
}  // namespace mp
}  // namespace hozon
