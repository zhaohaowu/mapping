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

#include "modules/local_mapping/lib/types/common.h"
#include "util/rviz_agent/rviz_agent.h"

namespace hozon {
namespace mp {
namespace lm {
struct CutRegion {
  float length_x;
  float length_y;
};

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
  void CutLocalMap(const CutRegion& cut_region, const Eigen::Vector3d& p_w_v);

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
   * @brief extend existed lane in localmap
   *
   * @param lane : lane data
   * @return
   */
  void UpdateLane(const LocalMapLane& lane);

  /**
   * @brief extend existed edge in localmap
   *
   * @param edge : edge data
   * @return
   */
  void UpdateEdge(const LocalMapLane& edge);

  /**
   * @brief extend some points to existed lane of localmap
   *
   * @param points : updated points
   * @return
   */
  void AppendLanePoints(const int& id,
                        const std::vector<Eigen::Vector3d>& points);

  /**
   * @brief extend some points to existed edge of localmap
   *
   * @param points : updated points
   * @return
   */
  void AppendEdgePoints(const int& id,
                        const std::vector<Eigen::Vector3d>& points);

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
  void GetLocalMap(std::shared_ptr<LocalMaps> local_map);

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

 private:
  LocalMaps local_map_;
  const std::string ktopic_lane_ = "/localmap/lane";
  const std::string ktopic_roadedge_ = "/localmap/roadedge";
  const std::string ktopic_cur_lane_ = "/localmap/curlane";
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
