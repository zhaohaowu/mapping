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
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"
// #include "Eigen/src/Core/Matrix.h"
#include "map_fusion/map_prediction/viz_map.h"
#include "proto/map/map_lane.pb.h"
#include "proto/map/map_road.pb.h"

#include "depend/common/math/line_segment2d.h";
#include "common/math/vec2d.h"


namespace hozon {
namespace mp {
namespace mf {

class MapPrediction {
 public:
  MapPrediction() = default;
  ~MapPrediction();

  int Init();
  void OnHqMap(const std::shared_ptr<hozon::hdmap::Map>& hqmap);
  //   void OnLocalMap(const std::shared_ptr<const apollo::hdmap::Map>&
  //   localMap);
  void OnInsNodeInfo(
      const std::shared_ptr<hozon::localization::HafNodeInfo>& msg);
  //   void OnLocalMap(const std::shared_ptr<LocalMap>& msg);
  void OnTopoMap(const std::shared_ptr<hozon::hdmap::Map>& topoMap);
  std::shared_ptr<hozon::hdmap::Map> GetPredictionMap();

 private:
  void Prediction();
  void PredictLeftRightLaneline(
      const std::shared_ptr<hozon::hdmap::Map>& topo_map_,
      const std::set<std::string>& curr_road_id_,
      const std::vector<hozon::hdmap::RoadInfoConstPtr>& roads);
  void ComputeDistLineToEdge(
      const std::pair<std::string, std::vector<Eigen::Vector3d>>& curr_line,
      const std::vector<Eigen::Vector3d>& edge, double* dist);
  void AddLeftOrRightLine(
      const std::vector<Eigen::Vector3d>& edge,
      const std::pair<std::string, std::vector<Eigen::Vector3d>>& curr_line,
      const double& dist, const uint32_t& record);
  void PredictAheadLaneLine(
      const std::shared_ptr<hozon::hdmap::Map>& topo_map_,
      const std::set<std::string>& add_road_id_,
      const std::vector<hozon::hdmap::RoadInfoConstPtr>& roads,
      const std::set<std::string>& add_lane_id_);
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
      const std::shared_ptr<hozon::hdmap::Map>& topo_map_,
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
      const std::set<std::string>& end_road_id_,
      const std::vector<hozon::hdmap::RoadInfoConstPtr>& roads);
  void ExpansionLaneLine(
      const std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>&
          complete_lines,
      const std::vector<std::string>& com_id,
      const std::vector<std::string>& sec_id);
  void AddResTopo();

  std::mutex mtx_;
  std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>
      hqmap_road_edge;
  std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>
      localmap_lanelines;
  std::shared_ptr<hozon::hdmap::HDMap> hq_map_server_ = nullptr;
  std::shared_ptr<hozon::hdmap::Map> topo_map_ = nullptr;
  std::shared_ptr<hozon::localization::HafNodeInfo> location_;
  //   std::shared_ptr<LocalMap> local_msg;
  std::shared_ptr<hozon::hdmap::Map> local_msg_ = nullptr;
  uint32_t id = -1;

  Eigen::Vector3d local_enu_center_;
  bool local_enu_center_flag_ = true;
  VizMap viz_map_;

  std::future<void> pred_proc_;
  std::atomic<bool> is_pred_proc_stop_{false};
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
