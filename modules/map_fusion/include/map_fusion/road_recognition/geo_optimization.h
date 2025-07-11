/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： geo_optimization.h
 *   author     ： zhangzhike
 *   date       ： 2023.12
 ******************************************************************************/

#pragma once
#include <cstdint>
#include <map>
#include <memory>
#include <random>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "map_fusion/fusion_common/element_map.h"
#include "map_fusion/road_recognition/occ_guideline_manager.h"
#include "map_fusion/topo_assignment/topo_assignment.h"

// #include <depend/map/hdmap/hdmap.h>
// #include <depend/proto/local_mapping/local_map.pb.h>
// #include <depend/proto/localization/localization.pb.h>
// #include <depend/proto/localization/node_info.pb.h>
// #include <depend/proto/map/map.pb.h>

// #include <map>
// #include <memory>
// #include <mutex>
// #include <string>
// #include <utility>
// #include <vector>

#include <boost/circular_buffer.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "Eigen/src/Core/Matrix.h"
#include "map_fusion/map_service/map_proto_maker.h"
#include "map_fusion/map_service/map_table.h"
#include "map_fusion/road_recognition/base_data.h"
#include "opencv2/core/hal/interface.h"
#include "util/geo.h"
#include "util/rviz_agent/rviz_agent.h"

#define PRIDICT_BACK_LANES 10.0
#define PRIDICT_FRONT_LANES 10.0

namespace hozon {
namespace mp {
namespace mf {

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

enum Color {
  UNKNOWN = 0,
  WHITE = 1,
  YELLOW = 2,
  GREEN = 3,
  RED = 4,
  BLACK = 5
};

enum class RelativePosition { LEFT = 0, RIGHT = 1, UNCERTAIN = 2 };

class LanePilot {
 public:
  int lane_id_ = 1000;
  double width_ = 1000.0;
  TurnType turn_type_ = TurnType::UNKNOWN_TURN_TYPE;
  hozon::mapping::LaneLine left_line_;
  hozon::mapping::LaneLine right_line_;
  hozon::mapping::LaneLine center_line_;
  hozon::mapping::LaneLine pilot_left_line_;
  hozon::mapping::LaneLine pilot_right_line_;
  hozon::mapping::LaneLine pilot_center_line_;
  int left_lane_id_ = 1000;
  int right_lane_id_ = 1000;
};

struct base_line_info {
  int base_line_flag =
      0;  // 0-->无base_line,1-->左线为base_line,2-->右线为base_line
  int base_line_left_id = -1;   // base_line左id
  int base_line_right_id = -1;  // base_line右id;
  double base_width = -1;
};

// class Line_kd {
//  public:
//   std::shared_ptr<hozon::mapping::LaneLine> line;
//   // std::vector<Eigen::Vector3d> line_points;
//   std::shared_ptr<cv::flann::Index> line_kdtree;
//   bool store = true;  // 是否存储该线
//   bool is_continue = false;  // 是否跟别的线融合了，如果被融合则不添加该线
//   bool is_merge = false;  // 是否是汇入的线，如果是汇入的线则保留
//   bool is_ego_road =
//       true;  // 是否是主路段的线。即road_edge lanepos为-1到1之间的line
//   std::vector<double> param = std::vector<double>(3, 0.0);  // 存储c0 c1 c2
// };

struct local_line_info {
  int line_track_id;
  hozon::mapping::LanePositionType lane_pos;
  std::vector<Eigen::Vector3d> local_line_pts;
  std::vector<double> right_width;       // 距离右边线的距离
  std::vector<double> right_road_width;  // 距离右侧路沿距离
  std::vector<double> left_road_width;   // 距离左侧路沿距离
  uint32_t flag;  // 0-->最左车道线，1-->最右车道线，2-->普通车道线
  Line_kd kd_line;
};

struct opening {
  int opening_id;
  std::vector<Eigen::Vector3d> left_boundary;
  std::vector<Eigen::Vector3d> right_boundary;
  std::vector<Eigen::Vector3d> boundary_center_points;
  double start_x;
  double end_x;
  std::vector<int> line_ids;
  double heading;
  double heading_err;
  struct objection {
    bool exist = false;
    std::vector<int> ids;
    int left_id;
    int right_id;
    Eigen::Vector3d left_point;
    Eigen::Vector3d right_point;
    double left_dis;
    double right_dis;
  };

  objection opposite_obs;
  objection same_direction_obs;
  objection stop_line;
  objection double_solid_yellow;
  objection perception_line;
};

struct geoobjs {
  int id;
  Eigen::Vector3d position;
  boost::circular_buffer<Eigen::Vector3d> his_positions;
  Eigen::Vector3d velocity;
  double theta;
  double length;
  double width;
  bool is_inverse;
  bool is_update;
  void Init(int size) {
    his_positions.set_capacity(size);
    his_positions.clear();
  }
};

class GeoOptimization {
 public:
  GeoOptimization() = default;
  ~GeoOptimization() = default;
  int Init();
  //! input
  void OnLocalization(
      const std::shared_ptr<hozon::localization::Localization>& msg);
  //! input
  void OnLocalMap(
      const std::shared_ptr<hozon::mapping::LocalMap>& msg,
      const std::shared_ptr<hozon::perception::PerceptionObstacles>& obj_msg);
  // void OnPilotMap();  // 输出给规控用的pilotmap topo_generation发
  //! output
  std::shared_ptr<hozon::mp::mf::em::ElementMap> GetElemMap();
  // std::shared_ptr<hozon::hdmap::Map> GetPilotMap();

  inline void SetRoadScene(RoadScene road_scene) { road_scene_ = road_scene; }

  inline RoadScene GetRoadScene() { return road_scene_; }

  inline em::ExitLaneInfo GetExitLane() { return exit_lane_info_; }

  inline std::map<int, em::Boundary::Ptr> GetStableOcc() {
    return stable_occ_roads_;
  }

  inline void SetPose(const KinePose& curr_pose) { curr_pose_ = curr_pose; }

  inline KinePose GetPose() { return curr_pose_; }

 private:
  RoadScene road_scene_;
  em::ExitLaneInfo exit_lane_info_;  // 路口退出车道在local系下的几何点
  std::map<int, em::Boundary::Ptr> stable_occ_roads_;  // 稳定的occ路沿
  KinePose curr_pose_;
  std::shared_ptr<OccGuideLineManager> occ_guideline_manager_ = nullptr;
  std::shared_ptr<hozon::mp::mf::em::ElementMap> elem_map_ = nullptr;
  std::shared_ptr<hozon::mapping::LocalMap> local_map_ = nullptr;
  std::shared_ptr<hozon::perception::PerceptionObstacles> per_objs_ = nullptr;
  boost::circular_buffer<
      std::shared_ptr<hozon::perception::PerceptionObstacles>>
      history_objs_;
  boost::circular_buffer<hozon::perception::PerceptionObstacle>
      inverse_history_objs_;
  int history_objs_size_ = 10;
  int inverse_history_objs_size_ = 20;
  std::unordered_map<int, geoobjs> geo_objs_;
  std::unordered_set<int> geo_obj_ids_;
  int objs_size_{50};
  std::shared_ptr<hozon::mapping::LocalMap> local_map_use_ =
      nullptr;  // 滤除一些不对的laneline,最终是可用的一些线
  // std::shared_ptr<std::vector<LanePilot>> map_lanes_ = nullptr;
  // std::shared_ptr<hozon::hdmap::Map> pilot_map_ = nullptr;
  std::map<int, std::vector<Line_kd>> all_lines_;  // 用于存储当前所有line信息
  std::unordered_map<int, Line_kd> opening_all_lines_;
  std::vector<int> opening_all_line_ids_;
  int last_opening_stopline_correlated_line_id_ = -1;
  std::unordered_map<int, local_line_info> local_line_table_;
  int32_t extra_val_ = 0;  // 暂时临时这样设定，后面需要整合优化
  base_line_info base_line_;
  std::set<int> last_track_id_;  // 记录上一帧不进行跟踪的trackid
  double slope_ = 0.;
  // gcj02 global position
  Eigen::Vector3d vehicle_pose_;
  Eigen::Vector3d ref_point_;
  std::mutex pose_mtx_;
  std::mutex vehicle_pose_mtx_;
  std::mutex local_map_mtx_;
  std::mutex mtx_;
  std::mutex mtxp_;
  const std::string kTopicRoadRecognitionTf = "/roadr/tf";
  const std::string kTopicRoadRecognitionLocation = "/roadr/location";
  const std::string KTopicRoadRecognitionLocalMap = "/roadr/local_map";
  const std::string KTopicRoadRecognitionTopoMapRoad = "/roadr/topo_map_road";
  const std::string KTopicRoadRecognitionTopoMapLane = "/roadr/topo_map_lane";
  const std::string KTopicRoadRecognitionElementMap = "/roadr/element_line";
  const std::string KTopicRoadRecognitionLineLable = "/roadr/line_lable";
  const std::string KTopicRoadRecognitionOccRoad = "/roadr/occ_road";

  const std::unordered_map<char, std::vector<float>> color_palette = {
      {'r', {1.0, 0.0, 0.0}}, {'g', {0.0, 1.0, 0.0}},   {'b', {0.0, 0.0, 1.0}},
      {'w', {1.0, 1.0, 1.0}}, {'o', {1.0, 0.647, 0.0}}, {'c', {0.0, 1.0, 1.0}},
      {'y', {1.0, 1.0, 0.0}}};

  double cur_timestamp_ = 0.;
  int utm_zone_ = 0;
  hozon::common::Pose init_pose_;
  Eigen::Isometry3d T_local_enu_to_local_;
  Eigen::Isometry3d T_utm_to_local_;
  Eigen::Vector3d pos_local_;
  Eigen::Quaterniond quat_local_;
  // position in local enu frame
  Eigen::Vector3d ins_pose_;
  // orientation in local enu frame
  Eigen::Quaterniond ins_q_w_v_;
  Eigen::Isometry3d T_U_V_;

  bool init_ = false;
  std::string init_pose_ser_;
  adsfi_proto::viz::Path location_path_;
  std::unordered_set<int> is_not_ego_lane_track_id_;
  std::vector<opening> openings_;
  void VizLocation(const Eigen::Vector3d& pose, const Eigen::Quaterniond& q_W_V,
                   const double stamp);

  void VisulPos(const Eigen::Vector3d& pose, const Eigen::Quaterniond& q_W_V,
                const double stamp);
  // std::vector
  void VizLocalMap();

  void VizElementMap();

  void VizOccRoad();

  void VizRRMapRoad(const std::shared_ptr<hozon::hdmap::Map>& msg,
                    adsfi_proto::viz::MarkerArray* markers_road) const;

  void VizRRMapLane(const std::shared_ptr<hozon::hdmap::Map>& msg,
                    adsfi_proto::viz::MarkerArray* markers_lane) const;
  void VizHQMap();

  // 可视化pilotmap topo_generation
  // void VizRRMap(const std::shared_ptr<hozon::hdmap::Map>& msg);
  static void PointsToMarker(const double stamp,
                             const std::vector<Eigen::Vector3d>& points,
                             adsfi_proto::viz::Marker* marker,
                             const std::vector<float>& color_type);

  static void LineIdToMarker(const double stamp, const Eigen::Vector3d& point,
                             const std::string& id,
                             adsfi_proto::viz::Marker* marker);
  void ConvertInnerMapLaneType(const Color& color, const bool& is_left,
                               const LaneType& inner_lanetype,
                               hozon::hdmap::LaneBoundaryType::Type* lanetype);
  void ConvertInnerLaneType(const LaneType& inner_lanetype,
                            hozon::mapping::LaneType* lanetype);
  // // 处理简单的topo关系以及预测前后车道线 topo_generation发

  // void UpdateLaneByLocalmap(
  //     const std::shared_ptr<hozon::mapping::LocalMap>& local_map);
  // void PridictFrontMapLanes();

  // void PridictBackMapLanes();

  // void PridictCenterMapLanes();

  void CompareOccLines(
      const std::vector<std::pair<int, em::OccRoad>>& left_group,
      const std::vector<std::pair<int, em::OccRoad>>& right_group,
      std::vector<std::pair<int, int>>* line_pairs);
  void CompareGroupLines(
      std::vector<std::vector<std::pair<int, em::OccRoad>>>* groupedLines,
      std::vector<std::pair<int, int>>* line_pairs);

  bool LineCublicSampling(const hozon::mapping::Occ& occ,
                          std::vector<float>* curve_params,
                          std::vector<Eigen::Vector3d>* new_line_pts);

  double OccLineFitError(const hozon::mapping::Occ& occ,
                         const std::vector<float>& curve_param);

  void CompleteLocalMap();

  void ExtractOccRoadGap();

  void ConstructOccGuideLine();

  void UpdateLocalMapLine();

  void CreateLocalLineTable();

  void MakeRoadEdgeToLaneLine();

  bool IsBetweenLinesMid(const std::vector<Eigen::Vector3d>& new_road_pts,
                         const hozon::mapping::LaneLine& target_line,
                         const int& direction);

  void CompareRoadAndLines(const std::vector<Eigen::Vector3d>& road_pts,
                           const int& road_id);

  void ConstructLaneLine(
      const std::vector<std::vector<Eigen::Vector3d>>& new_lines);

  void AlignmentVecLane();

  bool ComputerLineDisNoOverlap(
      const std::vector<Eigen::Vector3d>& edge_line_pts,
      const std::vector<Eigen::Vector3d>& line_pts, double* avg_width);

  bool ComputerLineDis(const std::vector<Eigen::Vector3d>& line_pts,
                       const std::vector<Eigen::Vector3d>& right_line_pts,
                       double* avg_width, int pts_interval = 2);

  bool ComputerLineDis(const std::vector<Eigen::Vector3d>& line_pts,
                       const std::vector<Eigen::Vector3d>& right_line_pts,
                       std::vector<double>* line_dis, double* avg_width,
                       int pts_interval = 2);

  void HandleExtraWideLane();

  void VerifyEgoLane(const int& left_id, const int& right_id,
                     const std::vector<int>& other_ids, bool* flag,
                     std::pair<int, int>* ego_other_ids);
  bool CheckEgoLane(const int& left_id, const int& right_id,
                    const std::vector<int>& other_ids);

  Eigen::Vector3f FindMinDisLinePoint(
      const Line_kd& kd_line,
      std::vector<float> query_point = std::vector<float>(2, 0));

  void FitMissedLaneLine(const std::pair<int, int>& ex);

  double ComputeLineHeading(const std::vector<Eigen::Vector3d>& line_pts);

  double ComputeCurvature(const std::vector<Eigen::Vector3d>& line_pts);

  Eigen::Vector3d ComputeLaneLineHeading(
      const hozon::mapping::LaneLine& lane_line);

  double ComputeAngleBetweenVectors(const Eigen::Vector3d& v1,
                                    const Eigen::Vector3d& v2);

  void ObtainBaseLine(std::vector<Eigen::Vector3d>* base_line,
                      std::vector<double>* base_width,
                      const std::vector<Eigen::Vector3d>& base_pts,
                      const std::vector<Eigen::Vector3d>& line_pts);

  void HandleSingleSideLine();

  bool CheckOppisiteLineByObj(const std::vector<Eigen::Vector3d>& points);
  static double ComputeVecToLaneDis(
      const std::vector<Eigen::Vector3d>& base_line);

  void FitSingleSideLine(std::vector<Eigen::Vector3d>* base_line,
                         std::vector<std::vector<Eigen::Vector3d>>* new_lines,
                         const int& lane_num, const bool& flag,
                         const int& is_boundary, const double& lane_width);

  bool JudgeLineOverRoad(const std::vector<Eigen::Vector3d>& lane_line,
                         const std::vector<Eigen::Vector3d>& road_line);

  void AppendElemtMap(
      const std::shared_ptr<hozon::mapping::LocalMap>& msg,
      const std::shared_ptr<hozon::perception::PerceptionObstacles>& obj_msg);

  bool IsNeighberLine(int lanei_pos, int lanej_pos);

  double ComputerPoint2Line(const hozon::mapping::LaneLine& lane,
                            const hozon::common::Point3D& point);

  bool ComputerPointIsInLine(const Eigen::Vector3d& P, const Eigen::Vector3d& A,
                             const Eigen::Vector3d& B);

  double ComputerPoint2Line(const Eigen::Vector3d& P, const Eigen::Vector3d& A,
                            const Eigen::Vector3d& B);
  void FillObjType(hozon::mp::mf::em::Obj* obj,
                   hozon::perception::PerceptionObstacle_Type objtype);
  void FillIsNearRoadLine(
      hozon::mp::mf::em::Boundary* lane_line,
      const std::vector<std::vector<Eigen::Vector3d>>& roads,
      const hozon::mapping::LaneLine& local_line);
  void FilterLocalMapLine(
      const std::shared_ptr<hozon::mapping::LocalMap>& local_map);
  // 将Line_kd填充到local_map_use_中，并定义规则是否是同一根线被拆分
  void ContinueLocalMapUseLine();
  void FilterShortLine();
  void FilterOppositeLine();
  void FilterNoEgoLineNoCrossing();
  void FilterReverseLine();
  void FilterOpeningLine();
  void CollectOpenings();
  void DealOpenings();
  opening SelectOpening();
  void FilterOpening(const opening& selected_opening);
  void OpeningDealObs();
  void OpeningDealStopline();
  void OpeningDealSolidYellow();
  void OpeningDealHeading();
  bool GetBoundaryLineObs(const std::vector<int> line_ids,
                          const Eigen::Vector3d& point, int obj_id,
                          std::unordered_map<int, std::vector<int>>* ids);

  void HandleOppisiteLine(const std::vector<Eigen::Vector3d>& target_line);
  void HandleOppisiteLineByStopline();
  void HandleOppisiteLineByObj();
  void HandleOppisiteLineByObjAndYelloLine();
  bool IsOppisiteLine(Line_kd* line);
  bool IsRight(const Eigen::Vector3d& P, const Eigen::Vector3d& A,
               const Eigen::Vector3d& B);
  double CalMeanLineHeading(const std::vector<Eigen::Vector3d>& points);
  RelativePosition IsRoadEdgeOnVehicleRight(
      const std::vector<Eigen::Vector3d>& points, const double& heading);
  RelativePosition IsRoadEdgeOnVehicleRightNocrossing(
      const std::vector<Eigen::Vector3d>& points);
  void MergeSplitLine();
  void FitLaneLine(const std::vector<Eigen::Vector3d>& pts,
                   std::vector<double>* c);
  void FitLaneLine(const std::shared_ptr<hozon::mapping::LaneLine>& pts,
                   std::vector<double>* c);
  double Dis2Line(const Line_kd& line1, const Line_kd& line2);
  double Angledis(double a1, double a2);
  void ExpandCluster(int dataIndex, int* clusterID,
                     const Eigen::MatrixXi& distance_mat,
                     std::vector<int>* label);
  int IsNeighbor(const std::vector<double>& line_param1,
                 const std::vector<double>& line_param2, double b);
  void VizLableLine(const std::vector<int>& index,
                    const std::vector<int>& lable);
  double cubicCurveModel(const Eigen::Vector3d& coefficients, double x);
  Eigen::Vector3d fitCubicCurveRANSAC(const std::vector<Eigen::Vector3d>& data,
                                      int maxIterations, double threshold);
  Eigen::Vector3d FitLaneLine(const std::vector<Eigen::Vector3d>& pts);

  double LineLength(const std::vector<Eigen::Vector3d>& pts);

  void Fillego(hozon::mp::mf::em::Boundary* lane_line, bool is_ego);

  void FillArrowType(em::Arrow* arrow, hozon::hdmap::ArrowData_Type arrowtype);

  void FilterIntersectLine();

  bool Intersect(const Line_kd& line1, const Line_kd& line2);
  bool Intersect(const hozon::common::Point3D& line1_s,
                 const hozon::common::Point3D& line1_e,
                 const hozon::common::Point3D& line2_s,
                 const hozon::common::Point3D& line2_e);
  void MergeCount(int* num, Eigen::MatrixXi* intersect_mat);
  std::vector<Eigen::Vector3d> FindTargetPoints(
      const std::vector<std::vector<Eigen::Vector3d>>& forward_road_edges);

  RelativePosition IsTargetOnLineRight(
      const std::vector<Eigen::Vector3d>& target_line,
      const std::vector<Eigen::Vector3d>& line);
  bool FindOCCGuidePoint();
  void UpdateOCCRoadPoints();
  double OccWidth(const em::OccRoad::Ptr& occ_road_ptr);
  bool GetFirstOCCPoints(const em::OccRoad::Ptr& occ_road_ptr,
                         int* first_point_index);
  bool GetFirstNearIndex(const std::vector<em::OccRoad::Ptr>& vec_occs,
                         const em::OccRoad::Ptr& occ_road_ptr,
                         int* first_point_index);
  void CalDistNearOcc(const std::vector<em::OccRoad::Ptr>& vec_occs, bool left,
                      const em::OccRoad::Ptr& occ_road_ptr, int* index,
                      double* distance);
  bool CaluclateSlope(const Eigen::Vector3d& point1,
                      const Eigen::Vector3d& point2, double* slope_value);
  template <typename T1, typename T2>
  float evalueHeadingDiff(const T1& x, const std::vector<T2>& params);
  void HandleOppisiteLineNoCrossing(
      const std::vector<Eigen::Vector3d>& target_line);
  std::vector<Eigen::Vector3d> FindTargetPointsNoCrossing(
      const std::vector<std::vector<Eigen::Vector3d>>& nearby_road_edges);
};
}  // namespace mf
}  // namespace mp
}  // namespace hozon
