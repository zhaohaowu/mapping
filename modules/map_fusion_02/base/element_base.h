/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： element_base.h
 *   author     ： chenlongxi
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <Eigen/Core>
#include <deque>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>

namespace hozon {
namespace mp {
namespace mf {

#define DEFINE_PTR(Type) typedef std::shared_ptr<Type> Ptr;
#define DEFINE_CONST_PTR(Type) typedef std::shared_ptr<const Type> ConstPtr;

using Id = int32_t;  // std::string;

enum Color {
  UNKNOWN_COLOR = 0,
  WHITE,
  RED,
  GREEN,
  YELLOW,
  BLACK,
};

struct Polygon {
  std::vector<Eigen::Vector3f> points;
};

struct Zebra {
  Id id;
  Polygon polygon;
  std::vector<int> group_index;
  std::vector<int> lane_index;
  std::vector<std::string> lane_id;  // 关联的车道
  DEFINE_PTR(Zebra);
};

enum ArrowType {
  UNKNOWN_TURN_ARROW = 0,
  STRAIGHT_ARROW,
  RIGHT_TURN_ARROW,
  LEFT_TURN_ARROW,
  U_TURN_ARROW,
  STRAIGHT_LEFT_TURN_ARROW,
  STRAIGHT_RIGHT_TURN_ARROW,
  STRAIGHT_U_TURN_ARROW,
  LEFT_U_TURN_ARROW,
  LEFT_RIGHT_TURN_ARROW,
  LEFT_FRONT_TURN_ARROW,
  RIGHT_FRONT_TURN_ARROW,
  STRAIGHT_LEFT_RIGHT_TURN_ARROW,
  STRAIGHT_LEFT_U_TURN_ARROW,
  RIGHT_U_TURN_ARROW,
  FORBID_LEFT_TURN_ARROW,
  FORBID_RIGHT_TURN_ARROW,
  FORBID_U_TURN_ARROW,
  FRONT_NEAR_CROSSWALK_ARROW,
};

struct Arrow {
  Id id;
  ArrowType type = UNKNOWN_TURN_ARROW;
  Polygon polygon;
  // id of the lane this arrow relates to
  std::vector<int> group_index;
  std::vector<int> lane_index;
  std::vector<std::string> lane_id;  // 关联的车道
  double heading;
  DEFINE_PTR(Arrow);
};

struct CrossWalk {
  Id id;
  Polygon polygon;

  DEFINE_PTR(CrossWalk)
};

enum DashSegType {
  UNKNOWN_DASH_SEG = 0,
  DASH_START_POINT,
  DASH_END_POINT,
  NORMAL_POINT,
};

struct BoundaryNode {
  Id id;
  DashSegType dash_seg = UNKNOWN_DASH_SEG;
  Eigen::Vector3f point;

  DEFINE_PTR(BoundaryNode)
};

enum BoundaryType {
  UNKNOWN_BOUNDARY_TYPE = 0,
  SINGLE_SOLID,
  SINGLE_DASH,
  SHORT_DASHED,
  DOUBLE_SOLID,
  DOUBLE_DASH,
  LEFT_SOLID_RIGHT_DASH,
  LEFT_DASH_RIGHT_SOLID,
  SHARED_AREA,                   // 导流线
  LANE_VIRTUAL_MARKING,          // 车道虚拟线
  INTERSECTION_VIRTUAL_MARKING,  // 路口虚拟线
  CURB_VIRTUAL_MARKING,          // 路边缘虚拟线
  UNCLOSED_ROAD,                 // 非封闭路段线
  ROAD_VIRTUAL,                  // 道路虚拟线
  LANE_CHANG_VIRTUAL,            // 变道虚拟线
  OTHER = 99,
};

enum LanePos {
  LanePositionType_BOLLARD_LEFT = -5,
  LanePositionType_FOURTH_LEFT = -4,
  LanePositionType_THIRD_LEFT = -3,
  LanePositionType_ADJACENT_LEFT = -2,  // ego左边第二个
  LanePositionType_EGO_LEFT = -1,       // ego左边
  LanePositionType_EGO_RIGHT = 1,       // ego右边
  LanePositionType_ADJACENT_RIGHT = 2,  // ego又边第二个
  LanePositionType_THIRD_RIGHT = 3,
  LanePositionType_FOURTH_RIGHT = 4,
  LanePositionType_BOLLARD_RIGHT = 5,
  LanePositionType_OTHER = 99,
};

enum LineType {
  LaneType_UNKNOWN = 0,                        // 未知
  LaneType_SOLID = 1,                          // 单实线
  LaneType_DASHED = 2,                         // 单虚线
  LaneType_SHORT_DASHED = 3,                   // 短虚线
  LaneType_DOUBLE_SOLID = 4,                   // 双实线
  LaneType_DOUBLE_DASHED = 5,                  // 双虚线
  LaneType_LEFT_SOLID_RIGHT_DASHED = 6,        // 左实右虚
  LaneType_RIGHT_SOLID_LEFT_DASHED = 7,        // 右实左虚
  LaneType_SHADED_AREA = 8,                    // 导流线
  LaneType_LANE_VIRTUAL_MARKING = 9,           // 车道虚拟线
  LaneType_INTERSECTION_VIRTUAL_MARKING = 10,  // 路口虚拟线
  LaneType_CURB_VIRTUAL_MARKING = 11,          // 路边缘虚拟线
  LaneType_UNCLOSED_ROAD = 12,                 // 非封闭路段线
  LaneType_ROAD_VIRTUAL = 13,                  // 道路虚拟线
  LaneType_LANE_CHANG_VIRTUAL = 14,            // 变道虚拟线
  LaneType_FISHBONE_SOLID = 15,                // 鱼骨实线
  LaneType_FISHBONE_DASHED = 16,               // 鱼骨虚线
  LaneType_OTHER = 99,                         // 其他
};

enum class IsEgo {
  Other_Road = 0,  // 非自车路段
  Ego_Road = 1,    // 在自车路段
};

struct Boundary {
  Id id;
  Color color = UNKNOWN_COLOR;
  BoundaryType boundary_type = UNKNOWN_BOUNDARY_TYPE;
  std::deque<BoundaryNode::Ptr> nodes;
  // ids of previous boundaries
  std::vector<Id> prev_boundary_ids;
  // ids of next boundaries
  std::vector<Id> next_boundary_ids;
  // 存储历史的id 对应localmap删除的线
  std::vector<Id> delete_ids;
  LanePos lanepos = LanePositionType_OTHER;
  LineType linetype = LaneType_UNKNOWN;
  IsEgo is_ego = IsEgo::Ego_Road;
  bool is_near_road_edge = false;
  DEFINE_PTR(Boundary)
};

struct Stpl {
  Id id;
  std::vector<Eigen::Vector3f> points;
  std::vector<int> group_index;
  std::vector<int> lane_index;
  std::vector<std::string> lane_id;  // 关联的车道
  DEFINE_PTR(Stpl);
};

struct LineCubic {
  // y = coefs[3] * x^3 + coefs[2] * x^2 + coefs[1] * x + coefs[0]
  std::vector<float> coefs;
  // start of x
  float start = 0.;
  // end of x
  float end = 0.;
};

// Refer to:
// https://en.wikipedia.org/wiki/Centripetal_Catmull%E2%80%93Rom_spline
struct CatmullRom {
  CatmullRom() = default;
  explicit CatmullRom(float para) : u(para) {}

  Eigen::Vector3f p0;
  Eigen::Vector3f p1;
  Eigen::Vector3f p2;
  Eigen::Vector3f p3;

  // parameter for parameterization
  float u = 0.5;
};

struct GeometryAttribute {
  // in 1/m
  float curvature = 0.;
  // latitude slope, in rad
  float lat_slope = 0.;
  // longitude slope, in rad
  float lon_slope = 0.;
  // heading angle w.r.t north
  float heading = 0.;
};

struct CenterLine {
  // id of the lane this center line relates to
  Id id;
  Id lane_id;
  GeometryAttribute attr;
  std::vector<Eigen::Vector3f> points;

  DEFINE_PTR(CenterLine)
};

enum LaneStatus {
  UNKNOWN_LANE_STATUS = 0,
  NORMAL_STATUS,
  CONSTRUCTION,
};

enum LaneType {
  UNKNOWN_LANE_TYPE = 0,
  MOTOR_WAY,
  NON_MOTOR_WAY,
};

enum DirectionType {
  DIRECTION_FORWARD = 0,
  DIRECTION_BACKWARD,
  BIDIRECTION,
};

enum TurnType {
  UNKNOWN_TURN_TYPE = 0,
  FORWARD,
  LEFT_TURN,
  RIGHT_TURN,
  U_TURN,
  FORWARD_LEFT_TURN,
  FORWARD_RIGHT_TURN,
  FORWARD_U_TURN,
  FORWARD_LEFT_RIGHT_TURN,
  LEFT_U_TURN,
};

enum ObjType {
  UNKNOWN = 0,
  UNKNOWN_UNMOVABLE,
  UNKNOWN_MOVABLE,
  PEDESTRIAN,
  BICYCLE,
  VEHICLE,
  CYCLIST,
  STATIC_OBSTACLE,
  TRANSPORT_ELEMENT,
  ANIMAL,
};

enum CutPointType : std::uint8_t {
  Unknown = 0,
  Split = 1,
  Merge = 2,
  Multi2Single = 3,  // 少变多, 多变少点
  Single2Multi = 4,
  V_Shaped = 5,
  V_Shaped_Inv = 6,  // 倒V字形状
  Broken = 7,
  Broken_Start = 8
};

struct LaneIDSet {
  Id id;
  LaneType lane_type = UNKNOWN_LANE_TYPE;
  TurnType turn_type = UNKNOWN_TURN_TYPE;
  LaneStatus lane_status = UNKNOWN_LANE_STATUS;
  float length = 0.;
  Id center_line_id;
  Id left_track_id;
  Id right_track_id;
  // ids of previous lanes
  std::vector<Id> prev_lane_ids;
  // ids of next lanes
  std::vector<Id> next_lane_ids;
  // ids of left lanes in same direction
  std::vector<Id> left_forward_lane_ids;
  // ids of right lanes in same direction
  std::vector<Id> right_forward_lane_ids;
  // ids of left lanes in reversed direction
  std::vector<Id> left_reverse_lane_ids;
  // ids of right lanes in reversed direction
  std::vector<Id> right_reverse_lane_ids;
  // ids of left boundaries this lane contains
  std::vector<Id> left_boundary_ids;
  // ids of right boundaries this lane contains
  std::vector<Id> right_boundary_ids;

  DEFINE_PTR(LaneIDSet)
};

enum RoadType {
  UNKNOWN_ROAD_TYPE = 0,
  HIGHWAY,
  URBAN,
};

enum SceneType {
  UNKNOWN_SCENE_TYPE = 0,
  NORMAL_SCENE,
  BRIDGE,
  TUNNEL,
};

enum RoadEdgeType {
  LINE = 0,   // 车道线
  OCC = 1,    // OCC路沿
  MODEL = 2,  // 感知路沿
};

struct RoadEdge {
  Id id;
  std::vector<Eigen::Vector3f> points;
  bool is_left = false;
  bool is_right = false;
  double dist_to_path;
  RoadEdgeType road_edge_type;

  DEFINE_PTR(RoadEdge)
};

struct StopLine {
  Id id;
  std::vector<Eigen::Vector3f> points;
  // ids of lanes this stop line relates to
  std::vector<Id> lane_ids;

  DEFINE_PTR(StopLine)
};

struct Object {
  Id id;
  ObjType type = UNKNOWN;
  Eigen::Vector3d position;
  Polygon polygon;  // obstacle corner points.
  Eigen::Vector3f velocity;
  double heading;
  double length;
  double width;
  // Size of obstacle bounding box.
  DEFINE_PTR(Object)
};

enum TrafficLightType {
  UNKNOWN_TRAFFIC_LIGHT_TYPE = 0,
  MOTOR_LIGHT,
  NON_MOTOR_LIGHT,
  PEDESTRIAN_LIGHT,
};

struct TrafficLight {
  Id id;
  TrafficLightType traffic_light_type = UNKNOWN_TRAFFIC_LIGHT_TYPE;
  Eigen::Vector3f point;
  // ids of lanes this traffic light controls
  std::vector<Id> lane_ids;

  DEFINE_PTR(TrafficLight)
};

// enum CoordType { kCoordLocalEnu = 0, kCoordWgs, kCoordGcj, kCoordUtm };

struct OccRoad {
  Id track_id;
  Id detect_id;
  Id group_id;  // 车后方最近一个group或车所在的group为0,车前方最近一个group为1,以此向前向后推
  std::vector<Eigen::Vector3d> road_points;      // 默认front为豁口点
  std::vector<Eigen::Vector3d> ori_road_points;  // 原始occ点，调试使用
  std::vector<Eigen::Vector3d> ori_detect_points;  // 原始detect occ点，调试使用
  // int road_flag;  // 路沿类型0-->隔断路沿，1-->实线路沿
  Id left_occ_id = -1;      // 左侧能构成道的id
  Id right_occ_id = -1;     // 右侧能构成道的id
  bool is_forward = false;  // 车前方true,车侧或车后false(且满足条件)
  int guide_index = -1;     // occ下引導點的index
  LineCubic curve_params;

  DEFINE_PTR(OccRoad)
};

enum SymbolType {
  UNKNOWN_SYMBOL_TYPE = 0,
  MIN_SPEED,
  MAX_SPEED,
  NUMBERS,
  CHARACTERS,
  BUMP,
  PROHIBITED_AREA,
};

struct Symbol {
  Id id;
  SymbolType symbol_type = UNKNOWN_SYMBOL_TYPE;
  Polygon polygon;
  // id of the lane this symbol relates to
  Id lane_id;

  DEFINE_PTR(Symbol)
};

enum PointType {
  RAW = 0,
  INTERPOLATED = 1,
  PREDICTED = 2,  // 预测车道的点
  VIRTUAL = 3,    // 虚拟车道的点
};

enum class RelativePosition { LEFT = 0, RIGHT = 1, UNCERTAIN = 2 };
struct Point {
  PointType type = RAW;
  Eigen::Vector3f pt{0.0, 0.0, 0.0};

  Point() = default;
  Point(PointType pt_type, float x, float y, float z)
      : type(pt_type), pt(x, y, z) {}
  DEFINE_PTR(Point)
};

struct Line {
  Id id;
  Id id_next = -1000;           // 下一根line的id号
  std::vector<Id> deteled_ids;  // 这根id的被删的其他ids
  LineType type;
  Color color;
  LanePos lanepos;
  IsEgo isego;
  std::deque<Point> pts;
  bool is_near_road_edge;
  // 末端方向向量与x轴夹角
  double mean_end_heading = 0;  // in rad, [-pi, +pi]
  // 1.heading 2.kappa 3.dkappa
  std::tuple<double, double, double> pred_end_heading{};
  // 末端heading角的标准差
  double mean_end_heading_std_dev = 0;
  // 末端平均点间距
  double mean_end_interval = 0;
  DEFINE_PTR(Line)
};

struct GeoLineInfo {
  int line_track_id = -999;
  int flag = 2;  // 0-->最左车道线，1-->最右车道线，2-->普通车道线
  LineType line_type;
  Color color;
  bool store = true;  // 是否存储该线
  bool is_continue = false;  // 是否跟别的线融合了，如果被融合则不添加该线
  bool is_merge = false;  // 是否是汇入的线，如果是汇入的线则保留
  IsEgo is_ego = IsEgo::Ego_Road;  // 是否主路的线
  bool is_updated = false;         // 更新至输出element map
  std::shared_ptr<cv::flann::Index> line_kdtree = nullptr;
  std::vector<Eigen::Vector3f> line_pts;
  std::vector<double> right_width{0.f};       // 距离右边线的距离
  std::vector<double> right_road_width{0.f};  // 距离右侧路沿距离
  std::vector<double> left_road_width{0.f};   // 距离左侧路沿距离
  std::vector<double> param = std::vector<double>(3, 0.0);  // 存储c0 c1 c2
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
