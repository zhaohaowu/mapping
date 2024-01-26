// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: LiGuo (liguo03@hozon.com)
// @file: lane_gatekeeper.h
// @brief: gatekeeper for lane tracker

#pragma once
#include <vector>

#include "modules/laneline_postprocess/lib/laneline/proto/lane_postprocess.pb.h"
#include "modules/laneline_postprocess/lib/laneline/tracker/bev_lane_tracker/base/lane_target.h"
#include "modules/laneline_postprocess/lib/laneline/tracker/bev_lane_tracker/base/roadedge_target.h"

namespace hozon {
namespace mp {
namespace environment {

struct PointLaneGatekeeperInitOptions {
  LaneGateKeepeParam lane_gate_keeper_param;
};

struct PointLaneGatekeeperOptions {};

class PointLaneGatekeeper {
 public:
  PointLaneGatekeeper() = default;
  ~PointLaneGatekeeper() = default;

  PointLaneGatekeeper(const PointLaneGatekeeper&) = delete;
  PointLaneGatekeeper& operator=(const PointLaneGatekeeper&) = delete;

 public:
  void Init(const PointLaneGatekeeperInitOptions& init_options);

  bool AbleToOutput(const PointLaneGatekeeperOptions& options,
                    const LaneTargetConstPtr& target,
                    const std::vector<LaneTargetConstPtr>& targets) const;

  bool AbleToOutput(const PointLaneGatekeeperOptions& options,
                    const RoadEdgeTargetConstPtr& target,
                    const std::vector<RoadEdgeTargetConstPtr>& targets) const;

 private:
  bool AbleToOutput(const LaneTargetConstPtr& target) const;

  bool AbleToOutput(const RoadEdgeTargetConstPtr& target) const;

 private:
  bool inited_ = false;
  LaneGateKeepeParam lane_gate_keeper_param_;
};

}  // namespace environment
}  // namespace mp
}  // namespace hozon
