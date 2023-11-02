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

#include "modules/local_mapping/types/types.h"
#include "proto/dead_reckoning/dr.pb.h"
#include "proto/local_mapping/local_map.pb.h"
#include "proto/localization/localization.pb.h"
#include "proto/perception/transport_element.pb.h"

namespace hozon {
namespace mp {
namespace lm {

enum LanePositionType {
  BOLLARD_LEFT = -5,
  FOURTH_LEFT = -4,
  THIRD_LEFT = -3,
  ADJACENT_LEFT = -2,  // ego左边第二个
  EGO_LEFT = -1,       // ego左边
  EGO_RIGHT = 1,       // ego右边
  ADJACENT_RIGHT = 2,  // ego右边第二个
  THIRD_RIGHT = 3,
  FOURTH_RIGHT = 4,
  BOLLARD_RIGHT = 5,
  OTHER = 6
};

enum LaneType {
  Unknown = 0,                     // 未知
  SolidLine = 1,                   // 单实线
  DashedLine = 2,                  // 单虚线
  ShortDashedLine = 3,             // 短虚线
  DoubleSolidLine = 4,             // 双实线
  DoubleDashedLine = 5,            // 双虚线
  LeftSolidRightDashed = 6,        // 左实右虚
  RightSolidLeftDashed = 7,        // 右实左虚
  ShadedArea = 8,                  // 导流线
  LaneVirtualMarking = 9,          // 车道虚拟线
  IntersectionVirualMarking = 10,  // 路口虚拟线
  CurbVirtualMarking = 11,         // 路边缘虚拟线
  UnclosedRoad = 12,               // 非封闭路段线
  RoadVirtualLine = 13,            // 道路虚拟线
  LaneChangeVirtualLine = 14,      // 变道虚拟线
  RoadEdge = 15,                   // 路沿
  Other = 99                       // 其他
};

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
  int track_id_;
  LanePositionType lanepos_;
  LaneType lanetype_;
  double lane_fit_a_;
  double lane_fit_b_;
  double lane_fit_c_;
  double lane_fit_d_;
  double x_start_vrf_;
  double x_end_vrf_;
  std::vector<Eigen::Vector3d> control_points_;

  Lane(const Lane& lane)
      : track_id_(lane.track_id_),
        lanepos_(lane.lanepos_),
        lanetype_(lane.lanetype_),
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
  std::vector<Lane> lanes_;
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
  std::vector<double> sample_x_;
};

class LocalMapLane {
 public:
  bool eval_vehicle(double x, double* y) const {
    if (x < vehicle_lane_param_->start_point_x_ ||
        x > vehicle_lane_param_->end_point_x_) {
      return false;
    }
    *y = vehicle_lane_param_->c3_ + vehicle_lane_param_->c2_ * x +
         vehicle_lane_param_->c1_ * x * x +
         vehicle_lane_param_->c0_ * x * x * x;
    return true;
  }

 public:
  int track_id_;
  LanePositionType lanepos_;
  LaneType lanetype_;
  std::vector<Eigen::Vector3d> points_;
  std::vector<Eigen::Vector3d> fit_points_;
  std::vector<LaneCubicSpline> lane_param_;
  std::shared_ptr<LaneCubicSpline> vehicle_lane_param_;
  KDTreePtr kdtree_;
  bool need_fit_;

  LocalMapLane() = default;
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
