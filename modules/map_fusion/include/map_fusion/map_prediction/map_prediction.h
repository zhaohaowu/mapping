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
#include "proto/map/map_lane.pb.h"
#include "proto/map/map_road.pb.h"

namespace hozon {
namespace mp {
namespace mf {

// Section
struct Boundary {
  std::string section_id;
  std::vector<std::string> lane_id;
  std::vector<std::vector<Eigen::Vector3d>> left_boundary;
  std::vector<std::vector<Eigen::Vector3d>> right_boundary;
};

struct LocalLane {
  enum LanePoseType { UNKNOWN = 0, FAR_LEFT, FAR_RIGHT, MIDDLE };

  LanePoseType flag = UNKNOWN;

  std::string lane_id;
  std::string road_id;
  std::string section_id;
  std::vector<Eigen::Vector3d> left_line;
  std::vector<Eigen::Vector3d> right_line;
  std::vector<std::string> left_lane_ids;
  std::vector<std::string> right_lane_ids;
  std::vector<std::string> prev_lane_ids;
  std::vector<std::string> next_lane_ids;
};

struct LocalRoad {
  std::string road_id;
  // <section_id, Boundary>
  std::unordered_map<std::string, Boundary> section_ids;
};

struct FarLane {
  std::string lane_id;
  uint32_t flag; //0-->left, 1-->right, 2-->left和right
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
  //   void OnLocalMap(const std::shared_ptr<const apollo::hdmap::Map>&
  //   localMap);
  //! 临时使用，用原始ins作为全局系定位，以后统一用OnLocalization
  void OnInsNodeInfo(
      const std::shared_ptr<hozon::localization::HafNodeInfo>& msg);
  //   void OnLocalMap(const std::shared_ptr<LocalMap>& msg);
  void OnLocalization(
      const std::shared_ptr<hozon::localization::Localization>& msg);
  void OnTopoMap(const std::shared_ptr<hozon::hdmap::Map>& topoMap);
  std::shared_ptr<hozon::hdmap::Map> GetPredictionMap();

  void Prediction();

 private:
  void OnLocationInGlobal(const Eigen::Vector3d& pos_gcj02, uint32_t utm_zone,
                          double utm_x, double utm_y);
  //  void HashTable(const hozon::hdmap::RoadInfoConstPtr& road_ptr,
  //                 const hozon::hdmap::LaneInfoConstPtr& lane_ptr,
  //                 std::unordered_map<std::string, Boundary>* hash_table);
  void PredictLeftRightLaneline(
      const std::set<std::string>& topo_section_ids_,
      const std::unordered_map<std::string, LocalLane>& lane_table_,
      const std::unordered_map<std::string, LocalRoad>& road_table_,
      const std::vector<std::string>& topo_lane_ids_);
  void ComputeDistLineToEdge(
      const std::pair<std::string, std::vector<Eigen::Vector3d>>& curr_line,
      const std::vector<Eigen::Vector3d>& edge, double* dist);
  void AddLeftOrRightLine(
      const std::vector<Eigen::Vector3d>& edge,
      const std::pair<std::string, std::vector<Eigen::Vector3d>>& curr_line,
      const uint32_t& mis_num, const uint32_t& record);
  void PredictAheadLaneLine(
      const std::vector<std::string> add_section_ids_,
      const std::unordered_map<std::string, LocalLane>& lane_table_,
      const std::unordered_map<std::string, LocalRoad>& road_table_);
  void DetermineEdgeAssPair(
      const std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>&
          all_edges,
      std::vector<std::pair<std::pair<uint32_t, uint32_t>, double>>&
          boundary_pairs);
  void FitAheadLaneLine(
      const std::vector<std::vector<Eigen::Vector3d>>& boundary1,
      const std::vector<std::vector<Eigen::Vector3d>>& boundary2,
      std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>&
          predict_lanelines,
      const uint32_t& lane_num);
  void FitLaneCenterline();
  void AddSideTopological(
      const std::vector<std::vector<Eigen::Vector3d>>& predict_line,
      const uint32_t& record, const std::string& curr_id);
  void AheadTopological(
      const std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>&
          predict_lanelines,
      const std::vector<std::string>& section_lane_id);
  void AddAheadLeftRightTopo(hozon::hdmap::Map* local_msg_);
  void separateBoundaries(
      const std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>&
          edge_,
      std::vector<std::vector<Eigen::Vector3d>>* boundary1,
      std::vector<std::vector<Eigen::Vector3d>>* boundary2);
  void findConnectedSegments(
      const std::vector<Eigen::Vector3d>& startSegment1,
      const std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>&
          edge_,
      std::vector<std::vector<Eigen::Vector3d>>* boundary1);
  void CompleteLaneline(
      const std::vector<std::string>& end_lane_id_,
      const std::set<std::string>& end_section_id,
      const std::unordered_map<std::string, LocalRoad>& road_table);
  void ExpansionLaneLine(
      const std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>&
          complete_lines,
      const std::vector<std::string>& com_id,
      const std::vector<std::string>& sec_id);
  void AddResTopo();
  void CatmullRoom(const std::vector<Eigen::Vector3d>& compan_point,
                   std::vector<Eigen::Vector3d>* cat_points);
  void SmoothAlignment();
  Eigen::Vector3d UtmPtToLocalEnu(const hozon::common::PointENU& utm_pt);
  void ConvertToLocal();
  void VizLocAndHqMap();

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
  uint32_t utm_zone_ = 0;
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
  VizMap viz_map_;

  std::unordered_map<std::string, FarLane> far_table_;
  std::unordered_map<std::string, LocalLane> lane_table_;
  std::unordered_map<std::string, LocalRoad> road_table_;
  std::vector<std::string> topo_lane_ids_;
  std::set<std::string> topo_section_ids_;
  std::vector<std::string> end_lane_ids_;
  std::set<std::string> end_section_ids_;
  std::set<std::string> add_lane_ids_;
  std::vector<std::string> add_section_ids_;
  std::unordered_map<std::string, uint32_t> end_prev_ids_;
  std::vector<std::string> all_section_ids_;
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
