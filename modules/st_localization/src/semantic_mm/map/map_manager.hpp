/*
 * Copyright (C) 2022 by SenseTime Group Limited. All rights reserved.
 * Zhao Ming <zhaoming@senseauto.com>
 */

#pragma once

#include <map>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Sophus/se3.hpp>

#include "eval/evaluator_smm.hpp"
#include "localization/data_type/base.hpp"
#include "localization/data_type/road_structure.hpp"
#include "localization/data_type/smart_ptr.hpp"
#include "localization/localization_param.hpp"

namespace senseAD {
namespace localization {
namespace smm {

class LaneLine;
class TrafficSign;
class Pole;

// class holds the map manager, contains map elements processing and related map
// query API interface
class MapManager {
 public:
  DEFINE_SMART_PTR(MapManager)

  explicit MapManager(const LocalizationParam& param) : param_(param) {}
  ~MapManager() = default;

  adLocStatus_t SwitchOriginProc();

  // @brief: update map
  adLocStatus_t UpdateMap(const RoadStructure::Ptr& road_structure,
                          const SE3d& pose);

  // @brief: update local map given pose and range
  adLocStatus_t UpdateLocalMap(
      const SE3d& pose, const Eigen::Matrix<double, 6, 6>& pose_cov,
      const std::pair<double, double>& lateral_range,
      const std::pair<double, double>& longitudinal_range);

  // @brief: clear map data
  adLocStatus_t ClearMap();

  adLocStatus_t ClearLocalMap();

  // @brief: get laneline interface
  const std::unordered_map<id_t, std::shared_ptr<LaneLine>>& GetLaneLines()
      const;
  const std::unordered_map<id_t, std::shared_ptr<LaneLine>>& GetLocalLaneLines()
      const;

  // @brief: get traffic sign interface
  const std::unordered_map<id_t, std::shared_ptr<TrafficSign>>&
  GetTrafficSigns() const;
  const std::unordered_map<id_t, std::shared_ptr<TrafficSign>>&
  GetLocalTrafficSigns() const;

  // @brief: get pole interface
  const std::unordered_map<id_t, std::shared_ptr<Pole>>& GetPoles() const;
  const std::unordered_map<id_t, std::shared_ptr<Pole>>& GetLocalPoles() const;

  // TODO(xxx): support other semantic types...

  // @brief: check quality of map data in bird view
  adLocStatus_t CheckMapData(const SE3d Tvw, SMMEvalData* eval_data) const;

 private:
  // @brief: process laneline
  adLocStatus_t ProcessMapLaneLine();
  adLocStatus_t ProcessMapLaneLineOld();

  // @brief: make roadside for inner map
  adLocStatus_t MakeRoadSideForInnerMap(
      float curb_laneline_dis, std::map<id_t, size_t>* line_id_idx_table);

  // @brief connect all lane lines to one lane line
  adLocStatus_t ConnectLaneLineForce(const std::vector<LineData>& lines,
                                     std::shared_ptr<LaneLine>* laneline);

  // @brief connect lane lines if their begin point and end point are close
  adLocStatus_t ConnectLaneLineByDistance(const std::vector<LineData>& lines,
                                          float x_connect_threshold,
                                          float y_connect_threshold,
                                          std::vector<LineData>* new_lines);
  adLocStatus_t ConnectLineDataByDistance(
      const std::unordered_set<size_t>& line_data_idxs,
      std::map<id_t, LineData>* connected_line_datas) const;

  // @brief make LaneLine object for each input LineData object
  adLocStatus_t MakeLaneLineData(
      const std::vector<LineData>& lines, id_t start_id,
      std::vector<std::shared_ptr<LaneLine>>* lanelines);

  // @brief: remove redundant lanelines
  adLocStatus_t RemoveRedundantLaneLines(
      std::unordered_map<id_t, std::shared_ptr<LaneLine>>* lanelines);

  // @brief: process traffic sign
  adLocStatus_t ProcessMapTrafficSign();

  // @brief: process pole
  adLocStatus_t ProcessMapPole();

  adLocStatus_t CovPropagate2Local2D(
      const SE3d& pose, const Eigen::Matrix<double, 6, 6>& pose_cov,
      const Point3D_t& local_point, Eigen::Matrix3d* local_cov) const;

  // TODO(xx): support extended, process other semantic elements ...

 private:
  LocalizationParam param_;

  // input map related data, 1hz update
  RoadStructure::Ptr road_structure_{nullptr};
  SE3d pose_;
  double ground_height_;

  // processed semantic map elements
  id_t id_counter_ = 0;  // unique id for all elements
  std::unordered_map<id_t, std::shared_ptr<LaneLine>> lanelines_;
  std::unordered_map<id_t, std::shared_ptr<TrafficSign>> traffic_signs_;
  std::unordered_map<id_t, std::shared_ptr<Pole>> poles_;

  // local map elements associated with query pose, 10hz update
  std::unordered_map<id_t, std::shared_ptr<LaneLine>> local_lanelines_;
  std::unordered_map<id_t, std::shared_ptr<TrafficSign>> local_traffic_signs_;
  std::unordered_map<id_t, std::shared_ptr<Pole>> local_poles_;
};

}  // namespace smm
}  // namespace localization
}  // namespace senseAD
