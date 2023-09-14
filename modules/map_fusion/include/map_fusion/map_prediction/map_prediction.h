/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： map_prediction.h
 *   author     ： zhangshuo
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once
// #include "depend/map/hdmap/hdmap.h"
#include <adsfi_proto/internal/node_info.pb.h>
#include <adsfi_proto/map/local_map.pb.h>
#include <depend/proto/map/map.pb.h>

#include <thread>
#include <string>
#include <utility>
#include <vector>
#include <memory>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"

namespace hozon {
namespace mp {
namespace mf {

class MapPrediction {
 public:
  MapPrediction() = default;
  ~MapPrediction() {
    Term();
  }

  int Init(const std::string& conf);
  void OnHqMap(const std::shared_ptr<const hozon::hdmap::Map>& hqMap);
  //   void OnLocalMap(const std::shared_ptr<const hozon::hdmap::Map>&
  //   localMap);
  void OnInsNodeInfo(
      const std::shared_ptr<adsfi_proto::internal::HafNodeInfo>& msg);
  void OnLocalMap(const std::shared_ptr<LocalMap>& msg);

 private:
  void PredictLeftRightLaneline(
      const std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>&
          hqMapRoadEdge,
      std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>&
          localMapLaneLines);
  void ComputeDisLineToEdge(
      const std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>&
          hqMapRoadEdge,
      std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>&
          localMapLaneLines,
      std::vector<std::pair<std::pair<uint32_t, uint32_t>, double>>&
          disLineToEdge);
  void AddLeftRightLine(
      const std::pair<std::pair<uint32_t, uint32_t>, double>& LineToEdge,
      const std::pair<uint32_t, std::vector<Eigen::Vector3d>>& line,
      const std::pair<uint32_t, std::vector<Eigen::Vector3d>>& edge,
      std::vector<std::vector<Eigen::Vector3d>>* predictLine);
  void PredictAheadLaneLine(
      std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>&
          localMapLaneLines,
      std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>&
          hqMapRoadEdge,
      const std::shared_ptr<adsfi_proto::internal::HafNodeInfo>& location);
  void DetermineEdgeAssPair(
      const std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>&
          allEdges,
      std::vector<std::pair<std::pair<uint32_t, uint32_t>, double>>&
          boundaryPairs);
  void FitAheadLaneLine(
      const std::vector<std::pair<std::pair<uint32_t, uint32_t>, double>>&
          boundaryPairs,
      const std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>&
          allEdges,
      std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>&
          predictLaneLines);
  void FitLaneCenterline(
      const std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>&
          completeLaneLines,
      std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>&
          allCenterLaneLines);
  void Prov();
  void Term();

  std::mutex mtx_;
  std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>> hqMapRoadEdge;
  std::vector<std::pair<uint32_t, std::vector<Eigen::Vector3d>>>
      localMapLaneLines;
  std::shared_ptr<adsfi_proto::internal::HafNodeInfo> location;
  int id = -1;
  std::atomic_bool running_ = {false};
  std::shared_ptr<std::thread> proc_th_ = nullptr;
  bool flag = true;

  Eigen::Vector3d local_enu_center_;
};

}  // namespace mf
}  // namespace mp
}  // namespace hozon
