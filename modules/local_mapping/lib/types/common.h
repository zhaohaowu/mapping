/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: shenliangchen
 *Date: 2023-09-05
 *****************************************************************************/
#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <vector>

#include "interface/adsfi_proto/location/location.pb.h"
#include "interface/adsfi_proto/map/local_map.pb.h"
#include "interface/adsfi_proto/perception/lanes.pb.h"
#include "modules/local_mapping/lib/types/types.h"

namespace hozon {
namespace mp {
namespace lm {

class Location {
 public:
  Location() = default;
  double timestamp_;
  Eigen::Vector3d position_;
  Eigen::Quaterniond quaternion_;
  Eigen::Vector3d euler_angle_;
  Eigen::Vector3d linear_vrf_;
  Eigen::Vector3d angular_vrf_;
  float heading_;
};

class Lane {
 public:
  Lane() = default;
  int lane_id_;
  double lane_fit_a_;
  double lane_fit_b_;
  double lane_fit_c_;
  double lane_fit_d_;
  double x_start_vrf_;
  double x_end_vrf_;
  std::vector<Eigen::Vector3d> control_points_;

  Lane(const Lane& lane)
      : lane_id_(lane.lane_id_),
        lane_fit_a_(lane.lane_fit_a_),
        lane_fit_b_(lane.lane_fit_b_),
        lane_fit_c_(lane.lane_fit_c_),
        lane_fit_d_(lane.lane_fit_d_),
        x_start_vrf_(lane.x_start_vrf_),
        x_end_vrf_(lane.x_end_vrf_),
        control_points_(lane.control_points_) {}
};

class Lanes {
 public:
  Lanes() = default;
  double timestamp_;
  std::vector<Lane> front_lanes_;
  std::vector<Lane> rear_lanes_;
};

typedef std::shared_ptr<Lanes> LanesPtr;
typedef std::shared_ptr<Lanes const> ConstLanesPtr;

enum ObjUpdateType { MERGE_OLD = 0, ADD_NEW = 1 };

class LaneCubicSpline {
 public:
  double start_point_x_;
  double end_point_x_;
  double c0_;
  double c1_;
  double c2_;
  double c3_;
};

class LocalMapLane {
 public:
  int track_id_;
  std::vector<Eigen::Vector3d> points_;
  std::vector<LaneCubicSpline> lane_param_;
  KDTreePtr kdtree_;

  LocalMapLane() = default;
  LocalMapLane(const LocalMapLane& local_map_lane)
      : track_id_(local_map_lane.track_id_),
        points_(local_map_lane.points_),
        lane_param_(local_map_lane.lane_param_),
        kdtree_(local_map_lane.kdtree_) {}
};

class LocalMap {
 public:
  double timestamp;
  std::vector<LocalMapLane> local_map_lane_;
  std::vector<LocalMapLane> local_map_edge_;
};

class LaneMatchInfo {
 public:
  std::shared_ptr<Lane> frame_lane_;
  std::shared_ptr<LocalMapLane> map_lane_;
  ObjUpdateType update_type_;
};

struct DrData {
 public:
  double timestamp = -1.0;
  Eigen::Vector3d pose;
  Eigen::Quaterniond quaternion;
  Eigen::Vector3d local_vel;
  Eigen::Vector3d local_omg;

  DrData Interpolate(const double scale, const DrData& end,
                     double timestamp) const {
    DrData res;
    res.timestamp = timestamp;
    res.pose = this->pose + (end.pose - this->pose) * scale;
    res.quaternion = this->quaternion.slerp(scale, end.quaternion);

    if (scale >= 0.5) {
      res.local_vel = end.local_vel;
      res.local_omg = end.local_omg;
    } else {
      res.local_vel = this->local_vel;
      res.local_omg = this->local_omg;
    }

    return res;
  }
};

typedef std::shared_ptr<DrData> DrDataPtr;
typedef std::shared_ptr<DrData const> ConstDrDataPtr;

}  // namespace lm
}  // namespace mp
}  // namespace hozon
