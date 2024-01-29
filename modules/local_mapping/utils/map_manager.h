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
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Sophus/se3.hpp>

#include "modules/local_mapping/datalogger/load_data_singleton.h"
#include "modules/local_mapping/types/types.h"
#include "modules/local_mapping/utils/common.h"
#include "modules/util/include/util/mapping_log.h"
#include "modules/util/include/util/rviz_agent/rviz_agent.h"
namespace hozon {
namespace mp {
namespace lm {

class MapManager {
 public:
  MapManager() = default;

  static void CutLocalMap(LocalMap* local_map, const double& length_x,
                          const double& length_y);

  static void UpdateLocalMap(LocalMap* local_map, const Sophus::SE3d& T_C_L);

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

  static void UpdateHeading(LocalMap* local_map);

  static void UpdateLanepos(LocalMap* local_map);

  static void MergeLaneLine(LocalMap* local_map);

  static void MatchLaneLine(const std::vector<LaneLine>& per_lane_lines,
                            const std::vector<LaneLine>& map_lane_lines,
                            std::vector<LaneLineMatchInfo>* lane_line_matches);
  static void MatchEdgeLine(const std::vector<LaneLine>& per_edge_lines,
                            const std::vector<LaneLine>& map_edge_lines,
                            std::vector<EdgeLineMatchInfo>* edge_line_matches);
  static void MatchStopLine(const std::vector<StopLine>& per_stop_lines,
                            const std::vector<StopLine>& map_stop_lines,
                            std::vector<StopLineMatchInfo>* stop_line_matches);
  static void MatchArrow(const std::vector<Arrow>& per_arrows,
                         const std::vector<Arrow>& map_arrows,
                         std::vector<ArrowMatchInfo>* arrow_matches);
  static void MatchZebraCrossing(
      const std::vector<ZebraCrossing>& per_zebra_crossings,
      const std::vector<ZebraCrossing>& map_zebra_crossings,
      std::vector<ZebraCrossingMatchInfo>* zebra_crossing_matches);
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
