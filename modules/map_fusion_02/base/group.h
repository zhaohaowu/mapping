/***************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： group.h
 *   author     ： taoshaoyuan
 *   date       ： 2024.01
 ******************************************************************************/

#pragma once

#include <depend/common/math/vec2d.h>

#include <Eigen/Core>
#include <deque>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "modules/map_fusion_02/base/element_base.h"
// #include "opencv2/core/core.hpp"
// #include "opencv2/features2d.hpp"

// using hozon::common::math::Vec2d;

namespace hozon {
namespace mp {
namespace mf {

// enum PointType {
//   RAW = 0,
//   INTERPOLATED = 1,
//   PREDICTED = 2,  // 预测车道的点
//   VIRTUAL = 3,    // 虚拟车道的点
// };
// struct Point {
//   PointType type = RAW;
//   em::Point pt{0.0, 0.0, 0.0};
//
//   Point() = default;
//   Point(PointType pt_type, float x, float y, float z)
//       : type(pt_type), pt(x, y, z) {}
// };
//
// struct Line {
//   em::Id id;
//   em::Id id_next = -1000;           // 下一根line的id号
//   std::vector<em::Id> deteled_ids;  // 这根id的被删的其他ids
//   em::LineType type;
//   em::Color color;
//   em::LanePos lanepos;
//   em::IsEgo isego;
//   std::deque<Point> pts;
//   bool is_near_road_edge;
//   // 末端方向向量与x轴夹角
//   double mean_end_heading = 0;  // in rad, [-pi, +pi]
//   // 1.heading 2.kappa 3.dkappa
//   std::tuple<double, double, double> pred_end_heading{};
//   // 末端heading角的标准差
//   double mean_end_heading_std_dev = 0;
//   // 末端平均点间距
//   double mean_end_interval = 0;
//   DEFINE_PTR(Line)
// };
//
// struct Zebra {
//   em::Id id;
//   em::Polygon polygon;
//   std::vector<int> group_index;
//   std::vector<int> lane_index;
//   std::vector<std::string> lane_id;  // 关联的车道
//   DEFINE_PTR(Zebra);
// };
//
// struct Arrow {
//   em::Id id;
//   em::ArrowType type = em::UNKNOWN_TURN_ARROW;
//   em::Polygon polygon;
//   // id of the lane this arrow relates to
//   std::vector<int> group_index;
//   std::vector<int> lane_index;
//   std::vector<std::string> lane_id;  // 关联的车道
//   DEFINE_PTR(Arrow);
// };
//
// struct Stpl {
//   em::Id id;
//   std::vector<em::Point> points;
//   std::vector<int> group_index;
//   std::vector<int> lane_index;
//   std::vector<std::string> lane_id;  // 关联的车道
//   DEFINE_PTR(Stpl);
// };

enum OverlapsType {
  ARROW = 0,
  STOPLINE = 1,
  ZEBRA = 2,
};
struct Overlaps {
  Id id;
  std::vector<int> group_index;
  std::vector<int> lane_index;
  std::vector<int> laps_index;
  OverlapsType overlaps;
  double start_s;
  double end_s;
  DEFINE_PTR(Overlaps);
};

struct LineSegment : public Line {
  void Copy(const LineSegment& line_segment, const LineType& line_type,
            const Color& color) {
    this->id = line_segment.id;
    this->lanepos = line_segment.lanepos;
    this->type = line_type;  // lane_in_curr->right_boundary->type;
    this->color = color;     // lane_in_curr->right_boundary->color;
    this->isego = line_segment.isego;
    this->is_near_road_edge = line_segment.is_near_road_edge;
    this->mean_end_heading = line_segment.mean_end_heading;
    this->pred_end_heading = line_segment.pred_end_heading;
    this->mean_end_heading_std_dev = line_segment.mean_end_heading_std_dev;
    this->mean_end_interval = line_segment.mean_end_interval;
    for (const auto& delete_id : line_segment.deteled_ids) {
      this->deteled_ids.emplace_back(delete_id);
    }
  }

  Eigen::Vector3f center;
  double dist_to_path;
  DEFINE_PTR(LineSegment)
};

struct EdgeSegment : public LineSegment {
  bool is_left = false;
  bool is_right = false;
  RoadEdgeType road_edge_type;

  DEFINE_PTR(EdgeSegment)
};

struct Lane {
  LineSegment::Ptr left_boundary = nullptr;
  LineSegment::Ptr right_boundary = nullptr;
  std::vector<Point> center_line_pts;
  std::vector<std::string> left_lane_str_id_with_group;
  std::vector<std::string> right_lane_str_id_with_group;
  std::vector<std::string> prev_lane_str_id_with_group;
  std::vector<std::string> next_lane_str_id_with_group;
  TurnType turn_type;
  DirectionType direction_type;
  std::string str_id;
  std::string str_id_with_group;
  std::string lanepos_id;
  std::vector<double> center_line_param;  // 线段末尾的y = kx + b (b,k)
  std::vector<double> center_line_param_front;  // 线段前向的y = kx + b (b,k)
  int is_trans = 0;        // 是否当前朝向，用黄实现判断
  int is_ego = 0;          // 是否当前道路，用路沿判断
  bool is_smooth = false;  // 是否平滑过了
  std::vector<Id> arrow_relate;
  std::vector<Id> zebra_relate;
  std::vector<Id> stl_relate;
  DEFINE_PTR(Lane)
};

// 切分GroupSegment的切分线，这条线可以用当时指向车体左侧的向量po->pl表示，
// 也可以用当时指向车体右侧的向量po->pr表示.
struct SliceLine {
  Eigen::Vector3f po;  // 中心原点
  Eigen::Vector3f pl;  // 左边点
  Eigen::Vector3f pr;  // 右边点
  CutPointType cut_type;
};

struct Group {
  enum GroupState {
    NORMAL = 0,
    DELETE = 1,
    VIRTUAL = 2,
  };

  SliceLine start_slice;
  SliceLine end_slice;
  std::vector<LineSegment::Ptr> line_segments;
  std::vector<EdgeSegment::Ptr> occ_segments;
  std::vector<EdgeSegment::Ptr> model_edge_segments;
  std::vector<EdgeSegment::Ptr> all_edge_segments;
  std::vector<RoadEdge::Ptr> road_edges;

  std::vector<Lane::Ptr> lanes;
  std::string str_id;
  std::string seg_str_id;
  double stamp = 0;
  bool is_last_after_erased = false;

  int broken_id = 0;

  GroupState group_state = GroupState::NORMAL;

  // 临时增加， 用于可视化
  std::vector<Point> guide_points_toviz;
  void Clear() {
    guide_points_toviz.clear();
    lanes.clear();
  }
  DEFINE_PTR(Group)
  DEFINE_CONST_PTR(Group)
};

struct IsCross {
  int is_crossing_ = 0;   // 是否是路口场景
  Point along_path_dis_;  // 自车与路口前的group的位移（向量）
  size_t cross_before_lane_ = 0;  // 路口前group的lanes数量
  size_t cross_after_lane_ = 0;   // 路口后group的lanes数量
  Id next_lane_left = -1000;  // 记录与路口后某条lane连接的左边线trackid
  Id next_lane_right = -1000;  // 记录与路口后某条lane连接的右边线trackid
  int is_connect_ = 0;  // 防抖，防止这帧连下一帧不连导致画龙
  std::set<std::string>
      next_satisefy_lane_seg;  // 记录路口后满足连接条件的所有lane名称
};

struct HistoryId {
  int lane_id = 0;
  int road_id = 0;
  int cicle = 2000;
};
struct EgoLane {
  int left_id = -200;
  int right_id = -200;
};

enum RoadScene {
  NON_JUNCTION = 0,
  BIG_JUNCTUIN = 1,
  SMALL_JUNCTION = 2,
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
