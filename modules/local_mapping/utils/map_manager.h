/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: luerwei/zhaohaowu
 *Date: 2023-09-01
 *****************************************************************************/
#pragma once

#include <gflags/gflags.h>

#include <Eigen/Dense>
#include <algorithm>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Sophus/se3.hpp>

#include "modules/local_mapping/types/types.h"
#include "modules/local_mapping/utils/common.h"
#include "modules/util/include/util/rviz_agent/rviz_agent.h"
namespace hozon {
namespace mp {
namespace lm {

class MapManager {
 public:
  MapManager() = default;

  static void CutLocalMap(LocalMap* local_map, const double& length_x,
                          const double& length_y);

  static void UpdatePerception(LocalMap* local_map, const Sophus::SE3d& T_C_L);

  static void AddNewLaneLine(LocalMap* local_map,
                             const LaneLine& per_lane_line);

  static void AddNewEdgeLine(LocalMap* local_map,
                             const LaneLine& per_edge_line);

  static void AddNewStopLine(LocalMap* local_map,
                             const StopLine& per_stop_line);

  static void AddNewArrow(LocalMap* local_map, const Arrow& per_arrow);

  static void AddNewZebraCrossing(LocalMap* local_map,
                                  const ZebraCrossing& per_zebra_crossing);

  static void MergeOldLaneLine(LocalMap* local_map,
                               const LaneLine& per_lane_line,
                               const LaneLine& map_lane_line);

  static void MergeOldEdgeLine(LocalMap* local_map,
                               const LaneLine& per_edge_line,
                               const LaneLine& map_edge_line);

  static void MergeOldStopLine(LocalMap* local_map,
                               const StopLine& per_stop_line,
                               const StopLine& map_stop_line);

  static void MergeOldArrow(LocalMap* local_map, const Arrow& per_arrow,
                            const Arrow& map_arrow);

  static void MergeOldZebraCrossing(LocalMap* local_map,
                                    const ZebraCrossing& per_zebra_crossing,
                                    const ZebraCrossing& map_zebra_crossing);

  static void UpdateLaneByPerception(LocalMap* local_map,
                                     const Perception& cur_lane_lines);

  static void UpdateLeftAndRightLine(int left_lanepos, int right_lanepos,
                                     const std::vector<LaneLine>& lane_lines,
                                     std::vector<Lane>* lanes);

  static void UpdateLaneByLocalmap(LocalMap* local_map);

  static void PridictFrontMapLanes(LocalMap* local_map);

  static void PridictBackMapLanes(LocalMap* local_map);

  static void PridictCenterMapLanes(LocalMap* local_map);

  static void UpdateLanepos(LocalMap* local_map);

  static void ConvertToLocalFrame(const Sophus::SE3d& T_W_V,
                                  LocalMap* local_map);

  static void MergeMapLeftRight(LocalMap* local_map, LaneLine* cur_lane_line);
  static void MergeMapFrontBack(LocalMap* local_map, LaneLine* cur_lane_line);
  static void MergePointsLeftRight(LaneLine* cur_lane_line,
                                   LaneLine* local_map_line);
  static void MergePointsFrontBack(LaneLine* cur_lane_line,
                                   LaneLine* local_map_line);
  static void MapMerge(LocalMap* local_map);
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
