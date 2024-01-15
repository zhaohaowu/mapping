/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: wangjianguo
 *Date: 2023-09-05
 *****************************************************************************/
#pragma once

#include <iostream>
#include <list>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Sophus/se3.hpp>

#include "modules/local_mapping/datalogger/load_data_singleton.h"
#include "modules/local_mapping/types/types.h"
#include "modules/local_mapping/utils/common.h"
#include "modules/util/include/util/mapping_log.h"
namespace hozon {
namespace mp {
namespace lm {

class LaneOp {
 public:
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

  static ConstDrDataPtr GetDrPoseForTime(double timestamp);

 private:
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
