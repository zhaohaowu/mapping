/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: likuan
 *Date: 2023-09-06
 *****************************************************************************/
#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <vector>

#include "opencv2/core/core.hpp"
#include "opencv2/features2d.hpp"
#include "proto/dead_reckoning/dr.pb.h"
#include "proto/local_mapping/local_map.pb.h"
#include "proto/localization/localization.pb.h"
#include "proto/perception/transport_element.pb.h"

namespace hozon {
namespace mp {
namespace lm {

using Vec2d = Eigen::Vector2d;
using Vec3d = Eigen::Vector3d;
using Mat3d = Eigen::Matrix3d;
using Matxd = Eigen::MatrixXd;

using VecMatxd = std::vector<Matxd>;

using KDTreePtr = std::shared_ptr<cv::flann::Index>;

using LanePointsPtr = std::shared_ptr<std::vector<Eigen::Vector3d>>;

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

enum TurnType {
  UNKNOWN_TURN_TYPE = 0,        // 未知转弯类型
  FORWARD = 1,                  // 直行
  LEFT_TURN = 2,                // 左转
  RIGHT_TURN = 3,               // 右转
  U_TURN = 4,                   // u形转弯
  FORWARD_LEFT_TURN = 5,        // 直行或左转
  FORWARD_RIGHT_TURN = 6,       // 直行或右转
  FORWARD_U_TURN = 7,           // 直行或u形转弯
  FORWARD_LEFT_RIGHT_TURN = 8,  // 直行或左转或右转
  LEFT_U_TURN = 9               // 左转或u形转弯
};

class Localization {
 public:
  double timestamp_;
  Eigen::Vector3d position_;
  Eigen::Quaterniond quaternion_;
  Eigen::Vector3d euler_angle_;
  Eigen::Vector3d linear_vrf_;
  Eigen::Vector3d angular_vrf_;
  float heading_;
};

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

class LaneLine {
 public:
  int track_id_;
  LanePositionType lanepos_;
  LaneType lanetype_;
  std::vector<Eigen::Vector3d> points_;
  std::vector<Eigen::Vector3d> control_points_;
  std::vector<Eigen::Vector3d> fit_points_;
  std::vector<LaneCubicSpline> lane_param_;
  KDTreePtr kdtree_;
  bool need_fit_;
  bool need_merge_;
  bool need_delete_;

  double c3_;
  double c2_;
  double c1_;
  double c0_;
  double start_point_x_;
  double end_point_x_;
  int edge_laneline_count_;
  bool ismature_;
};

class CenterLine {
 public:
  std::vector<Eigen::Vector3d> points_;
  std::vector<LaneCubicSpline> lane_param_;
};

class Perception {
 public:
  double timestamp_;
  std::vector<LaneLine> front_lane_lines_;
  std::vector<LaneLine> rear_lane_lines_;
  std::vector<LaneLine> lane_lines_;
};

class Lane {
 public:
  int lane_id_ = 1000;
  double width_ = 1000.0;
  TurnType turn_type_ = TurnType::UNKNOWN_TURN_TYPE;
  LaneLine left_line_;
  LaneLine right_line_;
  CenterLine center_line_;
  int left_lane_id_ = 1000;
  int right_lane_id_ = 1000;
};

class LocalMap {
 public:
  double timestamp;
  std::vector<LaneLine> lane_lines_;
  std::vector<LaneLine> edge_lines_;
  std::vector<Lane> lanes_;
  std::vector<Lane> map_lanes_;
};

class LaneMatchInfo {
 public:
  std::shared_ptr<LaneLine> frame_lane_line_;
  std::shared_ptr<LaneLine> map_lane_line_;
  ObjUpdateType update_type_;
};

class DrData {
 public:
  double timestamp = -1.0;
  Eigen::Vector3d pose;
  Eigen::Quaterniond quaternion;
  Eigen::Vector3d local_vel;
  Eigen::Vector3d local_omg;
  Eigen::Vector3d local_acc;
  int gear = 100;

  DrData Interpolate(const double scale, const DrData& end,
                     double timestamp) const {
    DrData res;
    res.timestamp = timestamp;
    res.pose = this->pose + (end.pose - this->pose) * scale;
    res.quaternion = this->quaternion.slerp(scale, end.quaternion);

    if (scale >= 0.5) {
      res.local_vel = end.local_vel;
      res.local_omg = end.local_omg;
      res.local_acc = end.local_acc;
      res.gear = end.gear;
    } else {
      res.local_vel = this->local_vel;
      res.local_omg = this->local_omg;
      res.local_acc = this->local_acc;
      res.gear = this->gear;
    }
    return res;
  }
};

using DrDataPtr = std::shared_ptr<DrData>;
using ConstDrDataPtr = std::shared_ptr<DrData const>;

}  // namespace lm
}  // namespace mp
}  // namespace hozon
