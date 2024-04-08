/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： map_prediction.h
 *   author     ： zhangshuo
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once
// #include "depend/map/hdmap/hdmap.h"
#include <adsfi_proto/internal/node_info.pb.h>
// #include <adsfi_proto/map/local_map.pb.h>
#include <depend/map/hdmap/hdmap.h>
#include <depend/proto/localization/node_info.pb.h>
#include <depend/proto/map/map.pb.h>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <future>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"
// #include "Eigen/src/Core/Matrix.h"

#include "Eigen/src/Core/Matrix.h"
#include "common/math/vec2d.h"
#include "depend/common/math/line_segment2d.h"
#include "map_fusion/map_prediction/viz_map.h"
#include "map_fusion/map_service/map_table.h"
#include "proto/map/map_lane.pb.h"
#include "proto/map/map_road.pb.h"

namespace hozon {
namespace mp {
namespace mf {

struct FarLane {
  std::string lane_id;
  uint32_t flag;  // 0-->left, 1-->right, 2-->left和right
  std::vector<Eigen::Vector3d> left_line;
  std::vector<Eigen::Vector3d> right_line;
};

class MapPrediction {
 public:
  MapPrediction() = default;
  ~MapPrediction() = default;

  int Init();
  void Stop();
  void OnHqMap(const std::shared_ptr<hozon::hdmap::Map>& hqmap);
  //! 临时使用，用原始ins作为全局系定位，以后统一用OnLocalization
  void OnInsNodeInfo(
      const std::shared_ptr<hozon::localization::HafNodeInfo>& msg);
  void OnLocalization(
      const std::shared_ptr<hozon::localization::Localization>& msg);
  void OnTopoMap(
      const std::shared_ptr<hozon::hdmap::Map>& msg,
      const std::tuple<std::unordered_map<std::string, LaneInfo>,
                       std::unordered_map<std::string, RoadInfo>>& map_info,
      hozon::routing::RoutingResponse* routing);
  std::shared_ptr<hozon::hdmap::Map> GetHdMapNNP(
      bool need_update_global_hd, hozon::routing::RoutingResponse* routing);
  std::shared_ptr<hozon::hdmap::Map> GetHdMapNCP(
      bool need_update_global_hd, hozon::routing::RoutingResponse* routing);
  std::shared_ptr<hozon::hdmap::Map> GetPredictionMap();

  void Prediction();

 private:
  void Clear();
  void OnLocationInGlobal(int utm_zone, double utm_x, double utm_y);
  void CalculateTanTheta(const std::string& idd);
  void CreatRoadTable(
      const std::vector<hozon::hdmap::RoadInfoConstPtr>& roads_in_range);
  void ConstructLaneLine(const std::vector<Eigen::Vector3d>& road_boundary,
                         const std::vector<std::string>& sec_lane_id);
  void CreatIdVector(const std::shared_ptr<hozon::hdmap::Map>& local_msg_);
  void CreatendIdVector(const hozon::hdmap::Lane& it, std::string* lane_id,
                        const LaneInfo& local_lane);
  void CreatAddIdVector(const std::vector<std::string>& end_lane_ids_);
  void FitLaneCenterline();
  static void GenerateCenterPoint(const std::vector<Vec2d>& left_point,
                                  const std::vector<Vec2d>& right_point,
                                  std::vector<Vec2d>* cent_point);
  static void CenterPoint(const std::vector<Vec2d>& project_points,
                          const std::vector<Vec2d>& points,
                          std::vector<Vec2d>* cent_point);
  static bool CalculateLambda(const Vec2d& p1, const Vec2d& p2, const Vec2d& p3,
                              const Vec2d& p4, double* lambda);
  static void InsertCenterPoint(hozon::hdmap::Lane* lane,
                                const std::vector<Vec2d>& cent_points);
  static void StoreLaneline(const hozon::hdmap::Lane& lane,
                            std::vector<Vec2d>* left_point,
                            std::vector<Vec2d>* right_point);
  void FusionLocalAndMap();
  void FusionLanePoint();
  //   static void FusionRoadEdgePoint(
  //       hozon::hdmap::BoundaryEdge* edge,
  //       const std::vector<Eigen::Vector3d>& left_boundary,
  //       const std::vector<Eigen::Vector3d>& right_boundary);
  static void ComputePerpendicularPoint(
      const Eigen::Vector3d& P, const std::vector<Eigen::Vector3d>& left_points,
      Eigen::Vector3d* C, bool* flag);
  void FusionStartOrEndLeftPoint(const Eigen::Vector3d& P,
                                 const std::string& id, Eigen::Vector3d* C,
                                 bool* flag);
  void FusionStartOrEndRightPoint(const Eigen::Vector3d& P,
                                  const std::string& id, Eigen::Vector3d* C,
                                  bool* flag);
  void CompleteLaneline(
      const std::vector<std::string>& end_lane_ids_,
      const std::set<std::string>& end_section_id,
      const std::unordered_map<std::string, RoadInfo>& road_table);
  void GetCorresId(std::vector<std::string>* com_id, std::string* road_id,
                   const std::string& sec);
  void ExpansionLaneLine(const std::vector<std::string>& com_id,
                         const std::vector<std::string>& sec_id);
  void ExpansionLeft(hdmap::Lane* lane);
  void ExpansionRight(hdmap::Lane* lane);
  void JudgeDiection(uint32_t* com_fit, const std::string& end_id);
  static void PointToLineDist(const Eigen::Vector3d& end_point, uint32_t* index,
                              double* min_distance,
                              const std::vector<Eigen::Vector3d>& line_points);
  void AddResTopo();
  static void CatmullRoom(const std::vector<Eigen::Vector3d>& compan_point,
                          std::vector<Eigen::Vector3d>* cat_points);
  void SmoothAlignment();
  Eigen::Vector3d UtmPtToLocalEnu(const hozon::common::PointENU& point_utm);
  Eigen::Vector3d GcjPtToLocalEnu(const hozon::common::PointENU& point_gcj);
  void ConvertToLocal();
  void HDMapLaneToLocal();
  void RoutingPointToLocal(bool need_update_global_hd,
                           hozon::routing::RoutingResponse* routing,
                           const std::string& id);
  void AppendRoutingLane(const hozon::hdmap::LaneInfoConstPtr& lane_ptr);
  void FusionMapLaneToLocal();
  void ArrawStopLineToLocal();
  void CrossWalkToLocal();
  void JunctionToLocal();
  void RoadToLocal();
  void DeelEdge(hozon::hdmap::BoundaryEdge* edge);
  void VizLocAndHqMap();
  void DealHqLine(adsfi_proto::viz::MarkerArray* ma);
  void LineToMarker(const std::vector<Eigen::Vector3d>& center_pts,
                    const std::vector<Eigen::Vector3d>& left_pts,
                    const std::vector<Eigen::Vector3d>& right_pts,
                    adsfi_proto::viz::MarkerArray* ma, const std::string& ns);
  void CheckLocalLane(std::set<std::string>* side_miss_ids);
  void PredLeftRight(const std::set<std::string>& side_miss_ids);
  void PredAheadLanes();
  //   void PredLocalRoads();
  //   void AddLeftRoadBoundary(hozon::hdmap::BoundaryEdge* edge,
  //                            const std::string& road_id,
  //                            const std::string& sec_id);
  //   void AddRightRoadBoundary(hozon::hdmap::BoundaryEdge* edge,
  //                             const std::string& road_id,
  //                             const std::string& sec_id);
  //   void PredAheadRoads();
  //   void AddAheadEdgeWithSameId(
  //       const std::vector<Eigen::Vector3d>& left_boundary,
  //       const std::vector<Eigen::Vector3d>& right_boundary,
  //       const std::string& road_id, const std::string& section_id,
  //       const std::vector<std::string>& lane_ids, bool* flag);
  //   void AddAheadEdgeWithNewId(const std::vector<Eigen::Vector3d>&
  //   left_boundary,
  //                              const std::vector<Eigen::Vector3d>&
  //                              right_boundary, const std::string& road_id,
  //                              const std::string& section_id,
  //                              const std::vector<std::string>& lane_ids);
  void AddArrawStopLine();
  void AddCrossWalk();
  void AddRoadEdge();

  std::mutex mtx_;
  std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>
      hqmap_road_edge;
  std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>
      localmap_lanelines;
  std::shared_ptr<hozon::hdmap::HDMap> hq_map_server_ = nullptr;
  std::shared_ptr<hozon::hdmap::Map> topo_map_ = nullptr;
  // gcj02
  Eigen::Vector3d location_;
  //   std::shared_ptr<LocalMap> local_msg;
  Eigen::Vector2d location_utm_;
  int utm_zone_ = 0;
  double ins_heading_ = 0.;
  std::shared_ptr<hozon::hdmap::Map> local_msg_ = nullptr;
  std::shared_ptr<hozon::hdmap::Map> hq_map_ = nullptr;
  uint32_t id = -1;

  Eigen::Vector3d local_enu_center_;
  bool local_enu_center_flag_ = true;
  hozon::common::Pose init_pose_;
  Eigen::Isometry3d T_local_enu_to_local_;
  Eigen::Vector3d pos_local_;
  Eigen::Quaterniond quat_local_;
  double stamp_loc_ = 0.;

  uint32_t lifetime_sec_ = 0;
  uint32_t lifetime_nsec_ = 0;

  uint32_t sec_ = 0;
  uint32_t nsec_ = 0;
  const std::string kLocalFrameId_ = "local";

  VizMap viz_map_;
  MapTable map_info_;

  std::unordered_map<std::string, FarLane> far_table_;
  //   std::unordered_map<std::string, LocalLane> lane_table_;
  //   std::unordered_map<std::string, LocalRoad> road_table_;
  std::unordered_map<std::string, LaneInfo> lane_table_;
  std::unordered_map<std::string, RoadInfo> road_table_;
  std::unordered_map<std::string, std::vector<Eigen::Vector3d>>
      left_virtual_line_;
  std::vector<std::string> topo_lane_ids_;
  std::set<std::string> topo_section_ids_;
  std::vector<std::string> end_lane_ids_;
  std::set<std::string> end_section_ids_;
  std::set<std::string> add_lane_ids_;
  std::vector<std::string> add_section_ids_;
  std::unordered_map<std::string, uint32_t> end_prev_ids_;
  std::vector<std::string> all_section_ids_;

  std::string routing_lane_id_;
  std::shared_ptr<hozon::routing::RoutingResponse> current_routing_ = nullptr;
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
