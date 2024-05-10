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
};

class MapMatch {
 public:
  MapMatch();
  void Match(const HdMap &hd_map, const std::shared_ptr<Perception> &perception,
             const SE3 &T02_W_V, const ValidPose &T_fc);
  Connect Result(void);
  Connect OriginResult(void);
  inline bool hasError() { return has_err_; }
  inline int GetErrorType() { return err_type_; }
  bool GoodMatchCheck(const SE3 &T);
  bool CheckLaneWidth(const SE3 &T);
  void SetInsTs(const double &ins_ts);
  void SetVel(const Eigen::Vector3d &linear_vel);
  int GetLanePairSize() { return lane_line_->get_match_line_size(); }
  int GetMatchPairSize() { return connect_.lane_line_match_pairs.size(); }
  VP GetRvizMergeMapLines() { return lane_line_->SetRvizMergeMapLines(); }

 private:
  MatchLaneLine::MatchLaneLinePtr lane_line_;
  Connect connect_;
  Connect origin_connect_;
  double ts_;
  bool has_err_;
  int err_type_;
  double ins_timestamp_;
  Eigen::Vector3d linear_vel_{0.0, 0.0, 0.0};
};

}  // namespace loc
}  // namespace mp
}  // namespace hozon
