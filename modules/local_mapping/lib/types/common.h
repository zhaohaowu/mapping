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

namespace hozon {
namespace mp {
namespace lm {

class Location {
 public:
  Location() = default;
  double timestamp_;
  Eigen::Vector3d position_;
  Eigen::Quaterniond quaternion_;
  Eigen::Vector3f euler_angle_;
  Eigen::Vector3f rotation_vrf_;
  float heading_;
};

class Lane {
 public:
  Lane() = default;
  int lane_id_;
  float lane_fit_a_;
  float lane_fit_b_;
  float lane_fit_c_;
  float lane_fit_d_;
  float x_start_vrf_;
  float x_end_vrf_;
  std::vector<Eigen::Vector3d> control_points_;
};

class Lanes {
 public:
  Lanes() = default;
  double timestamp_;
  std::vector<Lane> front_lanes_;
  std::vector<Lane> rear_lanes_;
};

enum ObjUpdateType { MERGE_OLD = 0, ADD_NEW = 1 };

class LaneCubicSpline {
 public:
  float start_point_x_;
  float end_point_x_;
  float c0_;
  float c1_;
  float c2_;
  float c3_;
};

class LocalMapLane {
 public:
  int track_id_;
  std::vector<Eigen::Vector3d> points_;
  std::vector<LaneCubicSpline> lane_param_;
};

class LocalMaps {
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

}  // namespace lm
}  // namespace mp
}  // namespace hozon
