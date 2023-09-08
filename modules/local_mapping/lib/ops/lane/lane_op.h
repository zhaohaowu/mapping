/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: wangjianguo
 *Date: 2023-09-05
 *****************************************************************************/
#pragma once

#include <iostream>
#include <memory>
#include <vector>

#include "modules/local_mapping/lib/types/common.h"
#include "util/temp_log.h"

namespace hozon {
namespace mp {
namespace lm {

class LaneOp {
 public:
  /**
   * @brief match lanes in current frame with map
   *
   * @param cur_lanes : lanes in current frame
   * @param map_lanes : lanes in map
   * @param match_info : match result
   * @return
   */
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
  void FilterCurve(std::shared_ptr<const LocalMapLane> map_lane,
                   std::shared_ptr<const Lane> cur_lane,
                   std::shared_ptr<std::vector<Eigen::Vector3d>> new_pts);
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
