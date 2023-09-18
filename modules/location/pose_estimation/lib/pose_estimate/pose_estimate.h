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
             const SE3 &T02_W_V, const SE3 &T_fc, const VP &percep_points,
             const VP &nearest_map_points);
  Connect Result(void);
  bool GoodMatchCheck(const SE3 &T);
  void SetInsTs(const double &ins_ts);
  std::vector<V3> Debug();
  std::vector<V3> DebugConnect();
  inline bool HasError() { return has_err_; }
  inline int GetErrorType() { return err_type_; }
  int GetLanePairSize() { return lane_line_->get_match_line_size(); }
  //   CommonState IsInRoadedge(const SE3 &pose, const HDMap &hd_map);
  //   void Match3D(const HDMap &hd_map, const Tracking &track, const SE3
  //   &T02_W_V);

 private:
  MatchLaneLine::Ptr lane_line_;
  //   MatchRoadEdge::Ptr road_edge;
  //   MatchTrafficSign::Ptr traffic_sign;
  //   MatchPole::Ptr pole;
  Connect connect_;
  bool has_err_;
  int err_type_;
  double ts_;
  double ins_timestamp_;
};

}  // namespace loc
}  // namespace mp
}  // namespace hozon
