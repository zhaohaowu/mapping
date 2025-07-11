/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： group_map.h
 *   author     ： taoshaoyuan
 *   date       ： 2024.01
 ******************************************************************************/

#pragma once

#include <depend/common/math/vec2d.h>

#include <deque>
#include <map>
#include <memory>
#include <queue>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "boost/circular_buffer.hpp"
#include "map_fusion/fusion_common/common_data.h"
#include "map_fusion/road_recognition/base_data.h"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d.hpp"

using hozon::common::math::Vec2d;

namespace hozon {
namespace mp {
namespace mf {
namespace gm {

enum PointType {
  RAW = 0,
  INTERPOLATED = 1,
  PREDICTED = 2,  // 预测车道的点
  VIRTUAL = 3,    // 虚拟车道的点
};
struct Point {
  PointType type = RAW;
  em::Point pt{0.0, 0.0, 0.0};

  Point() = default;
  Point(PointType pt_type, float x, float y, float z)
      : type(pt_type), pt(x, y, z) {}
};

struct Line {
  em::Id id;
  em::Id id_next = -1000;           // 下一根line的id号
  std::vector<em::Id> deteled_ids;  // 这根id的被删的其他ids
  em::LineType type;
  em::Color color;
  em::LanePos lanepos;
  em::IsEgo isego;
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

struct Zebra {
  em::Id id;
  em::Polygon polygon;
  std::vector<int> group_index;
  std::vector<int> lane_index;
  std::vector<std::string> lane_id;  // 关联的车道
  DEFINE_PTR(Zebra);
};

struct Arrow {
  em::Id id;
  em::ArrowType type = em::UNKNOWN_TURN_ARROW;
  em::Polygon polygon;
  // id of the lane this arrow relates to
  std::vector<int> group_index;
  std::vector<int> lane_index;
  std::vector<std::string> lane_id;  // 关联的车道
  DEFINE_PTR(Arrow);
};

struct Stpl {
  em::Id id;
  std::vector<em::Point> points;
  std::vector<int> group_index;
  std::vector<int> lane_index;
  std::vector<std::string> lane_id;  // 关联的车道
  DEFINE_PTR(Stpl);
};

enum OverlapsType {
  ARROW = 0,
  STOPLINE = 1,
  ZEBRA = 2,
};
struct Overlaps {
  em::Id id;
  std::vector<int> group_index;
  std::vector<int> lane_index;
  std::vector<int> laps_index;
  OverlapsType overlaps;
  double start_s;
  double end_s;
  DEFINE_PTR(Overlaps);
};

struct LineSegment : public Line {
  em::Point center;
  double dist_to_path;
  DEFINE_PTR(LineSegment)
};

struct LaneSegment {
  LineSegment::Ptr left_boundary;
  LineSegment::Ptr right_boundary;

  std::string str_id;
  std::string lanepos_id;
  DEFINE_PTR(LaneSegment)
};

struct Lane {
  LineSegment::Ptr left_boundary;
  LineSegment::Ptr right_boundary;
  std::vector<Point> center_line_pts;
  std::vector<std::string> left_lane_str_id_with_group;
  std::vector<std::string> right_lane_str_id_with_group;
  std::vector<std::string> prev_lane_str_id_with_group;
  std::vector<std::string> next_lane_str_id_with_group;
  em::TurnType turn_type;
  em::DirectionType direction_type;
  std::string str_id;
  std::string str_id_with_group;
  std::string lanepos_id;
  std::vector<double> center_line_param;  // 线段末尾的y = kx + b (b,k)
  std::vector<double> center_line_param_front;  // 线段前向的y = kx + b (b,k)
  int is_trans = 0;        // 是否当前朝向，用黄实现判断
  int is_ego = 0;          // 是否当前道路，用路沿判断
  bool is_smooth = false;  // 是否平滑过了
  std::vector<em::Id> arrow_relate;
  std::vector<em::Id> zebra_relate;
  std::vector<em::Id> stl_relate;
  DEFINE_PTR(Lane)
};

// 切分GroupSegment的切分线，这条线可以用当时指向车体左侧的向量po->pl表示，
// 也可以用当时指向车体右侧的向量po->pr表示.
struct SliceLine {
  em::Point po;  // 中心原点
  em::Point pl;  // 左边点
  em::Point pr;  // 右边点
};

struct GroupSegment {
  //  void CropAndBuild(const std::vector<Line::Ptr>& lines, )

  SliceLine start_slice;
  SliceLine end_slice;
  //  std::map<em::Id, LineSegment::Ptr> line_segments;
  std::vector<LineSegment::Ptr> line_segments;
  std::vector<LaneSegment::Ptr> lane_segments;
  std::string str_id;

  DEFINE_PTR(GroupSegment)
};

struct Group {
  enum GroupState {
    NORMAL = 0,
    DELETE = 1,
    VIRTUAL = 2,
  };
  std::vector<GroupSegment::Ptr> group_segments;
  std::vector<Lane::Ptr> lanes;
  std::string str_id;
  std::string seg_str_id;
  double stamp = 0;
  bool is_last_after_erased = false;

  GroupState group_state = GroupState::NORMAL;

  // 临时增加， 用于可视化
  std::vector<Point> guide_points_toviz;

  DEFINE_PTR(Group)
};

struct GroupMapConf {
  // 对上游车道线进行插值的点间距，-1时不插值
  float lane_line_interp_dist = 0;
  // 用于分割GroupSegment的分割线的1/2长度
  float half_slice_length = 0;
  // 从线聚合到lane的宽度阈值
  float min_lane_width = 0;
  float max_lane_width = 0;
  /// 以下用于填充proto里的lane和road限速:
  float lane_speed_limit_kmph = 0;
  float road_min_max_speed_kmph = 0;
  float road_max_max_speed_kmph = 0;
  /// 以下用于预测远端车道线:
  // 预测的最远距离，如果小于robust_percep_dist就不预测
  float predict_farthest_dist = 0;
  // 可信赖的感知区间距离
  float robust_percep_dist = 0;
  // 车道线末端heading阈值
  float max_heading_rad = 0;  // in rad
  // 车道线末端heading标准差阈值
  float max_heading_std_dev = 0;
  // 车道线末端点间距阈值
  float min_predict_interval = 0;
  float junction_heading_diff = 0;

  // 路口引导点相关配置参数
  float junction_guide_angle_ratio = 0.2;
  float junction_heading_along_length = 25.0;
  float junction_predict_distance = 40;
  float next_group_max_distance = 30;
  bool use_occ_roadedge = false;
  bool use_bev_roadedge = false;

  float junction_guide_min_dis = 25.0;
  float junction_guide_max_degree = 5;
};

struct IsCross {
  int is_crossing_ = 0;       // 是否是路口场景
  em::Point along_path_dis_;  // 自车与路口前的group的位移（向量）
  size_t cross_before_lane_ = 0;  // 路口前group的lanes数量
  size_t cross_after_lane_ = 0;   // 路口后group的lanes数量
  em::Id next_lane_left = -1000;  // 记录与路口后某条lane连接的左边线trackid
  em::Id next_lane_right = -1000;  // 记录与路口后某条lane连接的右边线trackid
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

class GroupMap {
 public:
  explicit GroupMap(const GroupMapConf& conf) : conf_(conf) {}
  ~GroupMap() = default;

  bool Build(const std::shared_ptr<std::vector<KinePose::Ptr>>& path,
             const KinePose::Ptr& curr_pose, const KinePose::Ptr& last_pose,
             const em::ElementMap::Ptr& ele_map, IsCross* is_cross);
  void GetGroups(std::vector<Group::Ptr>* groups);

  void Clear();

  void SetCurrentRoadScene(const std::vector<Group::Ptr>* groups);

  RoadScene GetCurrentRoadScene() { return road_scene_; }

  std::vector<Point> GetJunctionGuidePoint();
  std::shared_ptr<hozon::hdmap::Map> Export(const em::ElementMap::Ptr& ele_map,
                                            HistoryId* history_id);
  void SetSpeedLimit(const std::pair<double, double>& map_speed_limit);
  std::shared_ptr<hozon::mp::mf::em::ElementMapOut> AddElementMap(
      const em::ElementMap::Ptr& ele_map);

  bool ego_line_exist_ = false;
  std::vector<double> predict_line_params_;  // 三次样条

 private:
  void CollectGroupPossibleLanes(Group::Ptr grp,
                                 std::vector<Lane::Ptr>* possible_lanes);
  bool FilterGroupBadLane(const std::vector<Lane::Ptr>& possible_lanes,
                          Group::Ptr grp);
  bool MatchLRLane(Group::Ptr grp);
  bool MatchStopLineWithGroup(Group::Ptr grp);
  bool MatchZebraWithGroup(Group::Ptr grp);
  bool SetGroupLaneOrient(Group::Ptr grp);
  bool GenLaneCenterLine(std::vector<Group::Ptr>* groups);
  bool OptiPreNextLaneBoundaryPoint(std::vector<Group::Ptr>* groups);
  bool SetLaneStatus(std::vector<Group::Ptr>* groups);
  void ComputeLineHeadingPredict(
      std::vector<Group::Ptr>* groups,
      std::vector<LineSegment::Ptr>* lines_need_pred);
  bool LaneForwardPredict(std::vector<Group::Ptr>* groups, const double& stamp);
  bool OptiNextLane(std::vector<Group::Ptr>* groups);
  bool InferenceLaneLength(std::vector<Group::Ptr>* groups);

  void RetrieveBoundaries(const em::ElementMap::Ptr& ele_map, float interp_dist,
                          std::deque<Line::Ptr>* lines);
  void BuildKDtrees(std::deque<Line::Ptr>* lines);
  float DistByKDtree(const em::Point& ref_point, const LineSegment& line);
  float DistPointNew(const em::Point& ref_point,
                     const LineSegment& lineSegment);
  float GetDistPointLane(const em::Point& point_a, const em::Point& point_b,
                         const em::Point& point_c);
  void BuildGroupSegments(
      const std::shared_ptr<std::vector<KinePose::Ptr>>& path,
      const KinePose::Ptr& curr_pose, std::deque<Line::Ptr>* lines,
      std::vector<GroupSegment::Ptr>* group_segments,
      const em::ElementMap::Ptr& ele_map);
  void ProcessUTurn(const std::shared_ptr<std::vector<KinePose::Ptr>>& path);
  void CreateGroupSegFromPath(
      const std::shared_ptr<std::vector<KinePose::Ptr>>& path,
      const KinePose& curr_pose, std::vector<GroupSegment::Ptr>* segments);
  void SplitPtsToGroupSeg(std::deque<Line::Ptr>* lines,
                          std::vector<GroupSegment::Ptr>* segments);
  void GenLaneSegInGroupSeg(std::vector<GroupSegment::Ptr>* segments);
  void BuildGroups(const em::ElementMap::Ptr& ele_map,
                   std::vector<GroupSegment::Ptr> group_segments,
                   std::vector<Group::Ptr>* groups);
  void UniteGroupSegmentsToGroups(double stamp,
                                  std::vector<GroupSegment::Ptr> group_segments,
                                  std::vector<Group::Ptr>* groups);
  void GenLanesInGroups(std::vector<Group::Ptr>* groups,
                        std::map<em::Id, em::OccRoad::Ptr> occ_roads,
                        double stamp);
  bool LaneLineNeedToPredict(const LineSegment& line, bool check_back = true);
  void PredictLaneLine(std::vector<Lane::Ptr>* pred_lane,
                       const Lane::Ptr curr_lane);
  std::shared_ptr<hozon::hdmap::Map> ConvertToProtoMap(
      const std::vector<Group::Ptr>& groups, const KinePose::Ptr& curr_pose,
      const em::ElementMap::Ptr& ele, HistoryId* history_id);
  std::shared_ptr<hozon::mp::mf::em::ElementMapOut> ConvertToElementMap(
      const std::vector<Group::Ptr>& groups, const KinePose::Ptr& curr_pose,
      const em::ElementMap::Ptr& ele_map);

  hozon::hdmap::ArrowData_Type FillArrowType(em::ArrowType arrowtype);
  void EgoLineTrajectory(std::vector<GroupSegment::Ptr>* grp_segment,
                         const em::ElementMap::Ptr& ele_map);
  void FitLaneline(const em::ElementMap::Ptr& ele_map, int id_1, int id_2,
                   int near_line);
  std::vector<double> FitLaneline(const std::vector<Point>& centerline);
  std::vector<double> FitLanelinefront(const std::vector<Point>& centerline);
  void BuildVirtualLaneAfter(Group::Ptr curr_group, Group::Ptr next_group);
  void BuildVirtualLaneBefore(Group::Ptr curr_group, Group::Ptr next_group);
  void BuildVirtualMergeLane(Group::Ptr curr_group, Group::Ptr next_group,
                             size_t curr_lane, size_t curr_lane_next);
  void BuildVirtualProSucLane(Lane::Ptr lane_in_curr, Lane::Ptr lane_in_next,
                              float dis_pt);
  void BuildVirtualGroup(std::vector<Lane::Ptr> lane_virtual,
                         std::vector<Group::Ptr>* group_virtual, double stamp);
  void SetAllOverlaps(std::map<em::Id, Zebra::Ptr>* zebra,
                      std::map<em::Id, Arrow::Ptr>* arrow,
                      std::map<em::Id, Stpl::Ptr>* stopline,
                      std::map<em::Id, Overlaps::Ptr>* overlap, Lane::Ptr lane);
  void CatmullRom(const std::vector<Eigen::Vector3f>& pts,
                  std::vector<Eigen::Vector3f>* fit_points, int num);
  float PointToLaneDis(const Lane::Ptr& lane_ptr, Eigen::Vector3f point);
  void HeadingCluster(const std::vector<Lane::Ptr>& lanes_need_pred,
                      std::vector<LineSegment::Ptr>* lines_need_pred,
                      double threshold, bool need_pred_kappa);
  void RelateGroups(std::vector<Group::Ptr>* groups, double stamp);
  std::vector<Point> PredictGuidewirePath(
      std::vector<Group::Ptr>* groups,
      std::map<em::Id, em::OccRoad::Ptr> occ_roads, const double stamp);
  void RemoveNullGroup(std::vector<Group::Ptr>* groups);
  void RemainOnlyOneForwardCrossWalk(std::vector<Group::Ptr>* groups);
  bool MatchLanePtAndStopLine(const em::Point& left_pt,
                              const em::Point& right_pt, const Stpl& stop_line);
  bool MatchLanePtAndZebraLine(const em::Point& left_pt,
                               const em::Point& right_pt,
                               const Zebra& zebra_line);
  bool MatchLaneAndStopLine(const Lane::Ptr& lane, const Stpl::Ptr& stop_line);
  bool MatchLaneAndZebraLine(const Lane::Ptr& lane,
                             const Zebra::Ptr& zebra_line);
  Stpl::Ptr MatchedStopLine(const std::string& lane_id);
  void FindGroupNextLane(Group::Ptr curr_group, Group::Ptr next_group);
  void GenerateTransitionLane(Lane::Ptr lane_in_curr, Lane::Ptr lane_in_next,
                              std::vector<Lane::Ptr>* lane_virtual);
  void GenerateTransitionLaneGeo(Lane::Ptr lane_in_curr, Lane::Ptr lane_in_next,
                                 Lane::Ptr transition_lane);
  void GenerateTransitionLaneToPo(Lane::Ptr lane_in_curr,
                                  Lane::Ptr lane_in_next,
                                  Lane::Ptr transition_lane);
  void GenerateTransitionLaneToPo2(Lane::Ptr lane_in_curr,
                                   Lane::Ptr lane_in_next,
                                   Lane::Ptr transition_lane);
  float CalculateDistPt(Lane::Ptr lane_in_next, Lane::Ptr lane_in_curr,
                        size_t sizet);
  float CalculatePoint2CenterLine(Lane::Ptr lane_in_next,
                                  Lane::Ptr lane_in_curr);
  float Calculate2CenterlineAngle(Lane::Ptr lane_in_next,
                                  Lane::Ptr lane_in_curr, size_t sizet);
  bool AreLaneConnect(Group::Ptr curr_group, Group::Ptr next_group, int i,
                      int j,
                      std::map<int, std::vector<int>>* curr_group_next_lane,
                      std::map<int, std::vector<int>>* next_group_prev_lane);
  bool AreLaneConnect(Lane::Ptr lane_in_curr, Lane::Ptr lane_in_next);
  void NextGroupLaneConnect(
      Group::Ptr curr_group, Group::Ptr next_group,
      const std::map<int, std::vector<int>>& curr_group_next_lane);
  bool IsLaneShrink(Lane::Ptr lane);
  double CalcLaneLength(Lane::Ptr lane);
  bool IsAccessLane(Lane::Ptr lane_in_curr, Lane::Ptr lane_in_next);
  float LaneDist(Lane::Ptr lane_in_curr, Lane::Ptr lane_in_next);
  void BuildConnectLane(Lane::Ptr lane_in_curr, Group::Ptr next_group,
                        Lane::Ptr lane_in_next_next);
  void FitCenterLine(Lane::Ptr lane);
  void FillLineSegment(LineSegment::Ptr line, LineSegment* line_set);
  void UpdatePathInCurrPose(const std::vector<KinePose::Ptr>& path,
                            const KinePose& curr_pose);
  void ForwardCrossVirtual(Group::Ptr curr_group,
                           std::vector<Group::Ptr>* groups, double stamp);
  bool IsZebraIn(const Eigen::Vector2f& curr_start_pl,
                 const Eigen::Vector2f& curr_start_pr,
                 const Eigen::Vector2f& next_start_pl,
                 const Eigen::Vector2f& next_start_pr);
  bool IsVehicleInJunction(Group::Ptr curr_group, Group::Ptr next_group);

  void UpdateLaneBoundaryId(Group::Ptr curr_group);
  void DelLaneNextStrIdInGroup(Group::Ptr curr_group);
  void DelLanePrevStrIdInGroup(Group::Ptr curr_group);
  void FindNearestLaneToHisVehiclePosition(Group::Ptr curr_group,
                                           Lane::Ptr* ego_curr_lane);
  void FindBestNextLane(Group::Ptr next_group, const float& dist_to_slice,
                        Lane::Ptr* best_next_lane);
  void ComputeLineCurvatureV2(std::vector<Point>* guide_points,
                              std::vector<Group::Ptr>* groups, double stamp);
  void GenetateGuideLaneGeo(std::vector<Vec2d>* fit_points,
                            Lane::Ptr* guide_lane,
                            const Lane::Ptr& last_ego_lane);
  void GenerateGuideLaneToPo(Lane::Ptr lane_in_curr, Lane::Ptr guide_lane);
  void ComputeLineHeading(const Line::Ptr& line);
  void SmoothCenterline(std::vector<Group::Ptr>* groups);
  void VirtualLaneLeftRight(Group::Ptr curr_group, Group::Ptr next_group);
  std::vector<Point> SlidingWindow(std::vector<Point> centerline, int w);
  std::vector<Point> SigmoidFunc(std::vector<Point> centerline, float sigma);
  bool IsIntersect(Lane::Ptr line1, Lane::Ptr line2);
  void EraseIntersectLane(Group::Ptr curr_group, Group::Ptr next_group);
  bool BoundaryIsValid(const LineSegment& line);
  bool LineIdConsistant(LineSegment::Ptr line, em::Id id);
  void GenerateTransitionLaneToBefore(Lane::Ptr lane_in_curr,
                                      Lane::Ptr transition_lane);
  void GenerateTransitionLaneToAfter(Lane::Ptr lane_in_curr,
                                     Lane::Ptr lane_in_next,
                                     Lane::Ptr transition_lane);
  bool DistanceInferenceLane(const LineSegment& left_line,
                             const LineSegment& right_line);
  bool Distanceline(const LineSegment& left_line, float line_front_x,
                    float line_front_y);
  bool IsInGroupAndNoLane(Group::Ptr group);
  bool IsGroupsNoEgoLane(std::vector<Group::Ptr>* groups, int curr_group_index);
  bool IsGroupNoEgoLane(Group::Ptr group);
  void GenerateTransitionLaneToPo3(Lane::Ptr lane_in_curr,
                                   Lane::Ptr lane_in_next,
                                   Lane::Ptr transition_lane);
  void GenerateLane(Lane::Ptr lane_next, Lane::Ptr transition_lane);
  void FindSatisefyNextLane(Group::Ptr next_group, const float& dist_to_slice,
                            std::vector<Lane::Ptr>* satisfy_next_lane);
  void GenerateAllSatisfyTransitionLane(
      Lane::Ptr lane_in_curr, std::vector<Lane::Ptr>* virtual_lanes,
      std::vector<Lane::Ptr> history_satisfy_lane_,
      float dist_to_next_group_slice);
  void BuildVirtualGroup2(std::vector<Lane::Ptr> virtual_lanes,
                          std::vector<Group::Ptr>* group_virtual, double stamp);
  bool IsLeftLane(Group::Ptr next_group, int cur_lane_index,
                  int left_lane_index);
  bool IsRightLane(Group::Ptr next_group, int cur_lane_index,
                   int right_lane_inex);
  void AvoidSplitMergeLane(std::vector<Group::Ptr>* groups);
  // 删除next_group->lanes[next_index]的包含
  // curr_group->lanes[curr_erase_index]->str_id_with_group的前继
  void ErasePrevRelation(Group::Ptr curr_group, int curr_erase_index,
                         Group::Ptr next_group, int next_index);
  // 删除curr_group->lanes[curr_erase_index]的包含
  //  next_group->lanes[next_index]->str_id_with_group的后继
  void EraseSucessorRelation(Group::Ptr curr_group, int curr_erase_index,
                             Group::Ptr next_group, int next_index);
  bool ContainEgoLane(std::vector<Group::Ptr>* groups, int next_grp_index);
  int FindEgoGroup(const std::vector<Group::Ptr>*
                       groups);  // 找到自车所在的group 返回值是groups的index
  float PointToLineDis(const LineSegment& line, float line_front_x,
                       float line_front_y);
  void EraseEgoGroupWithNoEgoLane(
      std::vector<Group::Ptr>*
          groups);  // 把自车所在group但是没有自车道和自车邻车道的group删除

  bool IsAngleOkOfCurGrpAndNextGrp(Group::Ptr curr_group,
                                   Group::Ptr next_group);
  bool AreAdjacentLaneGroupsDisconnected(Group::Ptr curr_group,
                                         Group::Ptr next_group);
  double GetLaneLength(const std::vector<Group::Ptr>& groups,
                       std::string str_id_with_group);

  bool CloseToLaneEnd(const std::vector<Group::Ptr>& groups, std::string str_id,
                      const Eigen::Vector3f& target_point);
  float LengthToLaneStart(const std::vector<Group::Ptr>& groups,
                          std::string str_id,
                          const Eigen::Vector3f& target_point);
  void ExtendFrontCenterLine(std::vector<Group::Ptr>* groups);
  void AddVirtualLine(std::vector<Group::Ptr>* groups);
  int BesideGroup(
      Group::Ptr
          group);  // 是否在group所有lane的左边或右边，是左边1，是右边2，在中间0
  void BuildVirtualLaneLeft(Group::Ptr group);
  void BuildVirtualLaneRight(Group::Ptr group);
  void AddCrossLaneNeighbor(Group::Ptr cross_group,
                            Group::Ptr next_group);  // 路口生成车道左右邻添加
  void NeighborLane(std::vector<Group::Ptr>* groups);
  void VirtualLaneNeighborBefore(
      Group::Ptr curr_group,
      Group::Ptr next_group);  // 从后往前补的车道左右邻添加
  bool IsNearLine(LineSegment::Ptr line1, LineSegment::Ptr line2);
  void ComputeCenterPoints(Lane::Ptr lane);
  void ComputeCenterPointVirtualLane(Lane::Ptr lane);
  std::vector<double> FitVirtualLaneline(const std::vector<Point>& centerline);
  std::vector<double> FitVirtualLanelinefront(
      const std::vector<Point>& centerline);
  const double pi_ = acos(-1);
  std::map<em::Id, Zebra::Ptr> zebra_;
  std::map<em::Id, Arrow::Ptr> arrow_;
  std::map<em::Id, Stpl::Ptr> stopline_;
  std::map<em::Id, Overlaps::Ptr> overlaps_;
  std::map<em::Id, LineSegment::Ptr> lines_;
  GroupMapConf conf_;
  IsCross is_cross_;
  const double kMergeLengthThreshold = 10.;
  const double kSplitLengthThreshold = 10.;
  const float kLaneWidth = 3.5;
  std::vector<Pose> path_in_curr_pose_;
  std::vector<Group::Ptr> groups_;
  KinePose::Ptr curr_pose_ = nullptr;
  double delta_pose_heading_ = 0.;
  std::vector<GroupSegment::Ptr> group_segments_;
  EgoLane ego_line_id_;
  Lane::Ptr history_best_lane_ = nullptr;
  Lane::Ptr ego_curr_lane_ = nullptr;
  std::map<int, std::shared_ptr<cv::flann::Index>> KDTrees_;
  std::map<int, std::shared_ptr<std::vector<cv::Point2f>>> line_points_;
  float incross_before_virtual_lane_length_ =
      6.0;  // 过路口多条选择时，车后方的虚拟车道往前延伸长度
  std::pair<double, double> speed_limit_{0., 0.};
  inline bool IsSpeedLimitValid(const std::pair<double, double>& speed_limit) {
    return (speed_limit.first > 0. && speed_limit.second);
  }
  RoadScene road_scene_ = RoadScene::NON_JUNCTION;
  float big_junction_dis_thresh_ = 30.0;
};

}  // namespace gm
}  // namespace mf
}  // namespace mp
}  // namespace hozon
