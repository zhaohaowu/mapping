/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: wangjianguo
 *Date: 2023-09-05
 *****************************************************************************/
#pragma once

#include <algorithm>
#include <iostream>
#include <list>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Sophus/se3.hpp>

#include "modules/local_mapping/datalogger/load_data_singleton.h"
#include "modules/local_mapping/ops/association/association.h"
#include "modules/local_mapping/ops/association/horizon_assoc.h"
#include "modules/local_mapping/types/types.h"
#include "modules/local_mapping/utils/common.h"
#include "modules/util/include/util/mapping_log.h"
namespace hozon {
namespace mp {
namespace lm {

class LaneOp {
 public:
  static void Match(const Perception& cur_lane_lines, const LocalMap& local_map,
                    std::vector<LaneMatchInfo>* match_info);

  void Match(const Perception& cur_lane_lines, const LocalMap& local_map,
             std::vector<LaneMatchInfo>* match_info,
             bool use_horizon_assoc_match);
  static bool MatchLeftRight(const LaneLine& query_lane_line,
                             const LaneLine& other_lane_line);
  static void MergePointsLeftRight(LaneLine* query_lane_line,
                                   LaneLine* other_lane_line);
  static bool MatchFrontBack(const LaneLine& query_lane_line,
                             const LaneLine& other_lane_line);
  static bool MergePointsFrontBack(LaneLine* query_lane_line,
                                   LaneLine* other_lane_line);
  static void MergeMapLeftRight(LocalMap* local_map);
  static void MergeMapFrontBack(LocalMap* local_map);

  static ConstDrDataPtr GetDrPoseForTime(const double& timestamp);

 private:
  LaneAssocOptions lane_assoc_options_;
  LaneAssocPtr lane_assoc_ = nullptr;
  HorizonLaneAssocPtr horizon_lane_assoc_ = nullptr;
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
