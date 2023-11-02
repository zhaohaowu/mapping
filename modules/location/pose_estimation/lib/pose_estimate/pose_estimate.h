/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： pose_estimate.h
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <memory>
#include <vector>

#include "modules/location/pose_estimation/lib/hd_map/hd_map.h"
#include "modules/location/pose_estimation/lib/perception/perception.h"
#include "modules/location/pose_estimation/lib/pose_estimate/pose_estimate_lane_line.h"

namespace hozon {
namespace mp {
namespace loc {

struct Connect {
  bool is_ok = true;
  std::vector<PointMatchPair> lane_line_match_pairs;
  //   std::vector<V3> _road_edge;
  //   std::vector<PoleConnect> _pole;
  //   std::vector<TrafficSignConnect> _traffic_sign;
};

class MapMatch {
 public:
  MapMatch();
  void Match(const HdMap &hd_map, const std::shared_ptr<Perception> &perception,
             const SE3 &T02_W_V);
  Connect Result(void);
  bool GoodMatchCheck(const SE3 &T);
  void SetInsTs(const double &ins_ts);
  int GetLanePairSize() { return lane_line_->get_match_line_size(); }
  //   void Match3D(const HDMap &hd_map, const Tracking &track, const SE3
  //   &T02_W_V);

 private:
  MatchLaneLine::Ptr lane_line_;
  //   MatchRoadEdge::Ptr road_edge;
  //   MatchTrafficSign::Ptr traffic_sign;
  //   MatchPole::Ptr pole;
  Connect connect_;
  double ts_;
  double ins_timestamp_;
};

}  // namespace loc
}  // namespace mp
}  // namespace hozon
