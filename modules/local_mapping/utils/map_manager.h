/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: luerwei/zhaohaowu
 *Date: 2023-09-01
 *****************************************************************************/
#pragma once

#include <gflags/gflags.h>

#include <Eigen/Dense>
#include <algorithm>
#include <iostream>
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

  static void UpdateLaneLine(LocalMap* local_map, const Sophus::SE3d& T_C_L);

  static void AddNewLaneLine(LocalMap* local_map, const LaneLine& cur_lane_line,
                             const bool& use_perception_match);

  static void MergeOldLaneLine(LocalMap* local_map,
                               const LaneLine& cur_lane_line,
                               const LaneLine& map_lane_line);

  static void UpdateLaneByPerception(LocalMap* local_map,
                                     const Perception& cur_lane_lines);

  static void UpdateLeftAndRightLine(int left_lanepos, int right_lanepos,
                                     const std::vector<LaneLine>& lane_lines,
                                     std::vector<Lane>* lanes);

  static void UpdateLaneByLocalmap(LocalMap* local_map);

  static void UpdateLanepos(LocalMap* local_map);
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
