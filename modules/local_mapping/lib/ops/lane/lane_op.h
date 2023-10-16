/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: wangjianguo
 *Date: 2023-09-05
 *****************************************************************************/
#pragma once

#include <iostream>
#include <memory>
#include <unordered_map>
#include <vector>

#include <Sophus/se3.hpp>

#include "modules/local_mapping/lib/datalogger/load_data_singleton.h"
#include "modules/local_mapping/lib/ops/association/association.h"
#include "modules/local_mapping/lib/ops/association/bipartite_match.h"
#include "modules/local_mapping/lib/types/common.h"
#include "modules/local_mapping/lib/utils/common.h"
#include "modules/local_mapping/lib/utils/lane_filter.h"
#include "modules/util/include/util/temp_log.h"

namespace hozon {
namespace mp {
namespace lm {

class LaneOp {
 public:
  void Init(BipartiteAssocParams params);
  /**
   * @brief match lanes in current frame with map
   *
   * @param cur_lanes : lanes in current frame
   * @param map_lanes : lanes in map
   * @param match_info : match result
   * @return
   */
  void Match(ConstDrDataPtr lane_pose, std::shared_ptr<const Lanes> cur_lanes,
             std::shared_ptr<const std::vector<LocalMapLane>> map_lanes,
             std::shared_ptr<std::vector<LaneMatchInfo>> match_info,
             bool use_bipartite_assoc_match = false);

  void Match(std::shared_ptr<const Lanes> cur_lanes,
             std::shared_ptr<const std::vector<LocalMapLane>> map_lanes,
             std::shared_ptr<std::vector<LaneMatchInfo>> match_info);
  /**
   * @brief filter new lane, map as prediction and new lane as observation
   *
   * @param map_lane : map lane
   * @param cur_lane : new current lane
   * @param new_pts : new points, result of filter
   * @return
   */
  void FilterCurve(std::shared_ptr<const Lane> cur_lane,
                   std::vector<Eigen::Vector3d>* new_pts, int* lane_start_x,
                   const double& sample_interval);

  //  private:
  LaneAssocPtr lane_assoc_ = nullptr;
  BipartiteLaneAssocPtr bipar_lane_assoc_ = nullptr;
  bool SetCurLanePose(const double time);

  void SetLastLanePose();

  ConstDrDataPtr GetDrPoseForTime(const double timestamp);

 private:
  // std::shared_ptr<PtFilter> lane_filter_ = nullptr;
  double last_lane_timestamp_;
  Eigen::Vector3d cur_lane_pose_;
  Eigen::Vector3d last_lane_pose_;
  LaneAssocOptions lane_assoc_options_;

  std::unordered_map<int, std::shared_ptr<LaneFilter>> filter_map_;
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
