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

#include "Sophus/se3.hpp"
// #include "boost/circular_buffer/base.hpp"
#include "boost/circular_buffer.hpp"
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

enum EdgeType {
  ROAD_EDGE = 0,        // 马路路沿
  GROUND_EDGE = 1,      // 地面路沿
  CONE_EDGE = 2,        // 锥桶路沿
  WATERHORSE_EDGE = 3,  // 水马路沿
  FENCE_EDGE = 4,       // 围栏路沿
  UNKNOWN_EDGE = 5
};

enum Color {
  UNKNOWN = 0,
  WHITE = 1,
  YELLOW = 2,
  GREEN = 3,
  RED = 4,
  BLACK = 5
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

enum ArrowType {
  STRAIGHT_FORWARD = 0,                              // 直行箭头
  STRAIGHT_FORWARD_OR_TURN_LEFT = 1,                 // 直行或左转
  STRAIGHT_FORWARD_OR_TURN_RIGHT = 2,                // 直行或右转
  STRAIGHT_FORWARD_OR_TURN_LEFT_OR_TURN_RIGHT = 3,   // 直行或左转或右转
  STRAIGHT_FORWARD_OR_TURN_AROUND = 4,               // 直行或掉头
  STRAIGHT_FORWARD_OR_TURN_AROUND_OR_TURN_LEFT = 5,  // 直行或掉头或左转
  TURN_LEFT = 6,                                     // 左转
  TURN_LEFT_OR_MERGE_LEFT = 7,                       // 左转或向左合流
  TURN_LEFT_OR_TURN_AROUND = 8,                      // 左转或掉头
  TURN_LEFT_OR_TURN_RIGHT = 9,                       // 左转或右转
  TURN_RIGHT = 10,                                   // 右转
  TURN_RIGHT_OR_MERGE_RIGHT = 11,                    // 右转或向右合流
  TURN_RIGHT_OR_TURN_AROUND = 12,                    // 右转或掉头
  TURN_AROUND = 13,                                  // 掉头
  FORBID_TURN_LEFT = 14,                             // 禁止左转
  FORBID_TURN_RIGHT = 15,                            // 禁止右转
  FORBID_TURN_AROUND = 16,                           // 禁止掉头
  FRONT_NEAR_CROSSWALK = 17,                         // 前向斑马线
  ARROWTYPE_UNKNOWN = 18
};

class Polygon {
 public:
  std::vector<Eigen::Vector3d> points_;
  bool is_closure = false;
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

class LaneLine {
 public:
  int track_id_ = 1000;
  double length_ = 1000.0;
  double heading_ = 1000.0;
  double confidence_ = 1000.0;
  LanePositionType lanepos_ = LanePositionType::OTHER;
  LaneType lanetype_;
  EdgeType edgetype_;
  Color color_;
  std::vector<Eigen::Vector3d> points_;
  std::vector<Eigen::Vector3d> control_points_;
  std::vector<Eigen::Vector3d> fit_points_;
  bool need_delete_ = false;
  bool has_matched_ = false;
  bool ismature_ = false;
  bool is_after_stop_line_ = false;
  int count_;
  double c3_ = 1000.0;
  double c2_ = 1000.0;
  double c1_ = 1000.0;
  double c0_ = 1000.0;
  double c0_for_lanepos_ = 1000.0;
  double start_point_x_;
  double end_point_x_;
};

class StopLine {
 public:
  int track_id_ = 1000;
  Eigen::Vector3d left_point_;
  Eigen::Vector3d right_point_;
  Eigen::Vector3d mid_point_;
  double length_;
  double heading_;
  double confidence_ = 1000.0;
  bool has_matched_ = false;
  bool ismature_ = false;
  bool isstable_ = false;
  int count_ = 0;
};

class Arrow {
 public:
  ArrowType type_ = ArrowType::ARROWTYPE_UNKNOWN;
  int track_id_ = 1000;      // ID
  double heading_ = 1000.0;  // 路面箭头heading
  Polygon points_;
  double confidence_ = 1000.0;
  double length_ = 1000.0;
  double width_ = 1000.0;
  Eigen::Vector3d mid_point_;
  bool has_matched_ = false;
  bool ismature_ = false;
  int count_ = 0;
};

class ZebraCrossing {
 public:
  int track_id_ = 1000;
  double heading_ = 1000.0;  // 斑马线heading
  Polygon points_;
  double confidence_ = 1000.0;
  double length_ = 1000.0;
  double width_ = 1000.0;
  Eigen::Vector3d mid_point_;
  bool has_matched_ = false;
  bool ismature_ = false;
  int count_ = 0;
};
class Perception {
 public:
  double timestamp_;
  std::vector<LaneLine> lane_lines_;
  std::vector<LaneLine> edge_lines_;
  std::vector<StopLine> stop_lines_;
  std::vector<Arrow> arrows_;
  std::vector<ZebraCrossing> zebra_crossings_;
};

class Lane {
 public:
  int lane_id_ = 1000;
  double width_ = 1000.0;
  double length_ = 1000.0;
  TurnType turn_type_ = TurnType::UNKNOWN_TURN_TYPE;
  LaneLine left_line_;
  LaneLine right_line_;
  LaneLine center_line_;
  int left_lane_id_ = 1000;
  int right_lane_id_ = 1000;
};

class LocalMap {
 public:
  double timestamp;
  std::vector<LaneLine> lane_lines_;
  std::vector<LaneLine> edge_lines_;
  std::vector<StopLine> stop_lines_;
  std::vector<Arrow> arrows_;
  std::vector<ZebraCrossing> zebra_crossings_;
  std::vector<Lane> map_lanes_;

  std::vector<boost::circular_buffer<StopLine>>
      history_per_stop_lines_;  // 保存10帧感知停止线
  std::vector<boost::circular_buffer<StopLine>>
      history_per_arrows_;  // 保存10帧感知箭头
  std::vector<boost::circular_buffer<StopLine>>
      history_per_zebra_crossings_;  // 保存10帧感知斑马线
};

class LaneLineMatchInfo {
 public:
  LaneLine per_lane_line_;
  LaneLine map_lane_line_;
  ObjUpdateType update_type_ = ObjUpdateType::ADD_NEW;
};

class EdgeLineMatchInfo {
 public:
  LaneLine per_edge_line_;
  LaneLine map_edge_line_;
  ObjUpdateType update_type_ = ObjUpdateType::ADD_NEW;
};

class StopLineMatchInfo {
 public:
  StopLine per_stop_line_;
  StopLine map_stop_line_;
  ObjUpdateType update_type_ = ObjUpdateType::ADD_NEW;
};

class ArrowMatchInfo {
 public:
  Arrow per_arrow_;
  Arrow map_arrow_;
  ObjUpdateType update_type_ = ObjUpdateType::ADD_NEW;
};

class ZebraCrossingMatchInfo {
 public:
  ZebraCrossing per_zebra_crossing_;
  ZebraCrossing map_zebra_crossing_;
  ObjUpdateType update_type_ = ObjUpdateType::ADD_NEW;
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
