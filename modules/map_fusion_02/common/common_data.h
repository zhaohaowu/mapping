/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： common_data.h
 *   author     ： likuan
 *   date       ： 2024.08
 ******************************************************************************/
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>

namespace hozon {
namespace mp {
namespace mf {

enum class PoseState {
  NORMAL = 0,
  UTURN = 1,
  REVERS = 2,
  STAY = 3,
};

struct Pose {
  Pose() : stamp(0) {
    pos.setZero();
    quat.setIdentity();
  }

  virtual void Reset(double reset_stamp) {
    stamp = reset_stamp;
    pos.setZero();
    quat.setIdentity();
  }

  Eigen::Vector3f TransformPoint(const Eigen::Vector3f& pt) const {
    Eigen::Vector3f new_pt = quat * pt + pos;
    return new_pt;
  }

  Pose Inverse() const {
    Pose inv;
    inv.stamp = stamp;
    inv.quat = quat.inverse();
    inv.pos = inv.quat * pos * (-1);
    return inv;
  }

  double stamp = 0;
  Eigen::Vector3f pos;
  Eigen::Quaternionf quat;
  PoseState state = PoseState::NORMAL;
};

struct KinePose : public Pose {
  Eigen::Vector3f vel;
  Eigen::Vector3f acc;
  Eigen::Vector3f ang_vel;

  void Reset(double reset_stamp) override {
    Pose::Reset(reset_stamp);
    vel.setZero();
    acc.setZero();
    ang_vel.setZero();
  }
};

using KinePosePtr = std::shared_ptr<KinePose>;
using KinePoseConstPtr = std::shared_ptr<const KinePose>;

}  // namespace mf
}  // namespace mp
}  // namespace hozon