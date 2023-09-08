/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: wangjianguo
 *Date: 2023-09-05
 *****************************************************************************/
#include "modules/local_mapping/lib/ops/lane/lane_op.h"

namespace hozon {
namespace mp {
namespace lm {

void LaneOp::Match(std::shared_ptr<const Lanes> cur_lanes,
                   std::shared_ptr<const std::vector<LocalMapLane>> map_lanes,
                   std::shared_ptr<std::vector<LaneMatchInfo>> match_info) {
  HLOG_INFO << "GetMatches";
}

void LaneOp::FilterCurve(std::shared_ptr<const LocalMapLane> map_lane,
                         std::shared_ptr<const Lane> cur_lane,
                         std::shared_ptr<std::vector<Eigen::Vector3d>> new_pts) {
  HLOG_INFO << "FilterCurve";
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
