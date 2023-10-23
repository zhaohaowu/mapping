/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: luerwei/zhaohaowu
 *Date: 2023-09-01
 *****************************************************************************/
#pragma once

#include <gflags/gflags.h>

#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <Sophus/se3.hpp>

#include "modules/local_mapping/types/common.h"
#include "modules/local_mapping/utils/common.h"
#include "modules/util/include/util/rviz_agent/rviz_agent.h"

namespace hozon {
namespace mp {
namespace lm {

class MapManager {
 public:
  void Init();

  MapManager() = default;

  /**
   * @brief cut localmap to fix size
   *
   * @param cut_region : cut region size
   * @return
   */
  void CutLocalMap(const double& length_x, const double& length_y);

  /**
   * @brief add new lane in localmap
   *
   * @param lane : lane data
   * @return
   */
  void AddLane(const LocalMapLane& lane);

  /**
   * @brief add new edge in localmap
   *
   * @param edge : edge data
   * @return
   */
  void AddEdge(const LocalMapLane& edge);

  /**
   * @brief lane from last to current
   *
   * @param T_C_L : T_C_L data
   * @return
   */
  void UpdateLane(const Sophus::SE3d& T_C_L);

  /**
   * @brief lane from last to current
   *
   * @param T_C_L : T_C_L data
   * @return
   */
  void UpdateEdge(const Sophus::SE3d& T_C_L);
  /**
   * @brief update timestamp
   *
   * @param timestamp : lasted timestamp
   * @return
   */
  void UpdateTimestamp(const double& timestamp);

  /**
   * @brief extend old points to existed lane of localmap
   *
   * @param id : old points id
   * @param points : updated points
   * @return
   */
  void AppendOldLanePoints(const int& id,
                           const std::vector<Eigen::Vector3d>& points);
  /**
   * @brief extend new lane by points
   *
   * @param points : updated points
   * @return lane id
   */
  double CreateNewLane(const std::vector<Eigen::Vector3d>& points);

  /**
   * @brief extend new lane by points and specified id
   *
   * @param lane_id : specified id
   * @param points : updated points
   * @return lane id
   */
  void CreateNewLane(const std::vector<Eigen::Vector3d>& points,
                     const int& lane_id);

  /**
   * @brief extend some points to existed edge of localmap
   *
   * @param points : updated points
   * @return
   */
  void AppendEdgePoints(const int& id,
                        const std::vector<Eigen::Vector3d>& points);

  /**
   * @brief delete some lane points of localmap
   *
   * @param id : id
   * @param min_dis : min_dis
   * @return
   */
  void DeleteLanePoints(const int& id, const double& min_dis);

  /**
   * @brief create track vehicle curve of localmap
   *
   * @param id : id
   * @return
   */
  void CreateTrackVehicleLane(const int& id);

  /**
   * @brief create new track vehicle curve of localmap
   *
   * @param cur_lane : new vehicle curve
   * @return
   */
  void CreateNewTrackVehicleLane(std::shared_ptr<const Lane> cur_lane);

  /**
   * @brief create track vehicle curve of localmap
   *
   * @param id : id
   * @param T_V_W : T_V_W
   * @return
   */
  void CreateTrackVehicleLane(const int& id, const Sophus::SE3d& T_V_W);

  /**
   * @brief get lane in local map
   *
   * @param lanes : lane data
   * @return
   */
  void GetLanes(std::shared_ptr<std::vector<LocalMapLane>> lanes);

  /**
   * @brief get edge in local map
   *
   * @param edges : edge data
   * @return
   */
  void GetEdges(std::shared_ptr<std::vector<LocalMapLane>> edges);

  /**
   * @brief get local map
   *
   * @param local_map : local_map data
   * @return local map data
   */
  void GetLocalMap(std::shared_ptr<LocalMap> local_map);

  /**
   * @brief get timestamp
   *
   * @return timestamp
   */
  double GetTimestamp();

  /**
   * @brief publish local map
   *
   * @param local_map : time
   * @return
   */
  void PubLocalMap(uint64_t sec, uint64_t nsec);

  /**
   * @brief publish points
   *
   * @param local_map : time, points
   * @return
   */
  void PubPoints(const std::vector<Eigen::Vector3d>& points, uint64_t sec,
                 uint64_t nsec, const std::string topic);

  /**
   * @brief publish lines
   *
   * @param local_map : time, lines, topic name
   * @return
   */
  void PubLines(std::vector<LocalMapLane> lanes, uint64_t sec, uint64_t nsec);

  /**
   * @brief set lane property
   *
   * @param lane_id : lane id
   * @param frame_lane : lane of current frame
   * @return
   */
  void SetLaneProperty(int lane_id,
                       const std::shared_ptr<const Lane> frame_lane);
  LocalMap local_map_;

 private:
  const std::string ktopic_lane_ = "/localmap/lane";
  const std::string ktopic_roadedge_ = "/localmap/roadedge";
  const std::string ktopic_cur_lane_ = "/localmap/curlane";
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
