/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： pose_estimate.cc
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/

#include "modules/location/pose_estimation/lib/pose_estimate/pose_estimate.h"

#include <memory>
#include <vector>

namespace hozon {
namespace mp {
namespace loc {

MapMatch::MapMatch() {
  lane_line_ = std::make_shared<MatchLaneLine>();
}

void MapMatch::SetInsTs(const double &ins_ts) {
  ins_timestamp_ = ins_ts;
}

bool MapMatch::GoodMatchCheck(const SE3 &T) {
  return lane_line_->CheckIsGoodMatch(T);
}

bool MapMatch::CheckLaneWidth(const SE3 &T) {
  return lane_line_->CompareLaneWidth(T);
}

void MapMatch::Match(const HdMap &hd_map,
                     const std::shared_ptr<Perception> &perception,
                     const SE3 &T02_W_V) {
  lane_line_->set_ins_ts(ins_timestamp_);
  lane_line_->Match(hd_map, perception, T02_W_V);
  connect_.lane_line_match_pairs = lane_line_->get_match_pairs();
  HLOG_INFO << "CONNECT LANE LINE SIZE: "
            << connect_.lane_line_match_pairs.size();
}

Connect MapMatch::Result(void) { return connect_; }

}  // namespace loc
}  // namespace mp
}  // namespace hozon
