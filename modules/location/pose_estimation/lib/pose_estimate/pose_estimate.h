/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： pose_estimate.h
 *   author     ： ouyanghailin
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <memory>
#include <vector>
#include <list>
#include <unordered_map>
#include <string>

#include "modules/location/pose_estimation/lib/hd_map/hd_map.h"
#include "modules/location/pose_estimation/lib/perception/perception.h"
#include "modules/location/pose_estimation/lib/pose_estimate/pose_estimate_lane_line.h"

namespace hozon {
namespace mp {
namespace loc {

struct Connect {
  bool is_ok = true;
  std::vector<PointMatchPair> lane_line_match_pairs;
};

class MapMatch {
 public:
  MapMatch();
  void Match(
      const std::unordered_map<std::string, std::vector<ControlPoint>>&
          merged_map_lines,
      const std::list<std::list<LaneLinePerceptionPtr>>& percep_lanelines,
      const SE3& T02_W_V, const ValidPose& T_fc);
  Connect Result(void);
  Connect OriginResult(void);
  bool GoodMatchCheck(const SE3& T);
  bool CheckLaneWidth(const SE3 &T, double* lane_width_diff);
  void SetInsTs(const double &ins_ts);
  void SetVel(const Eigen::Vector3d &linear_vel);
  int GetLanePairSize() { return lane_line_->get_match_line_size(); }
  int GetMatchPairSize() { return connect_.lane_line_match_pairs.size(); }
  bool GetMatchBigCurvature() { return lane_line_->get_big_curvature(); }

 private:
  MatchLaneLine::MatchLaneLinePtr lane_line_;
  Connect connect_;
  Connect origin_connect_;
  double ts_;
  double ins_timestamp_;
  Eigen::Vector3d linear_vel_{0.0, 0.0, 0.0};
};

}  // namespace loc
}  // namespace mp
}  // namespace hozon
