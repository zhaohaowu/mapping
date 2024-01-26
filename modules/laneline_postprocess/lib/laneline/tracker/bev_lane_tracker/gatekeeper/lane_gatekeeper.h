// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: LiGuo (liguo03@hozon.com)
// @file: lane_gatekeeper.h
// @brief: gatekeeper for lane tracker

#pragma once
#include <vector>

#include "modules/laneline_postprocess/lib/laneline/proto/lane_postprocess.pb.h"
#include "modules/laneline_postprocess/lib/laneline/tracker/bev_lane_tracker/base/lane_target.h"

namespace hozon {
namespace mp {
namespace environment {

struct LaneGatekeeperInitOptions {
  LaneGateKeepeParam lane_gate_keeper_param;
};

struct LaneGatekeeperOptions {};

class LaneGatekeeper {
 public:
  LaneGatekeeper() = default;
  ~LaneGatekeeper() = default;

  LaneGatekeeper(const LaneGatekeeper&) = delete;
  LaneGatekeeper& operator=(const LaneGatekeeper&) = delete;

 public:
  void Init(const LaneGatekeeperInitOptions& init_options);

  bool AbleToOutput(const LaneGatekeeperOptions& options,
                    const LaneTargetConstPtr& target,
                    const std::vector<LaneTargetConstPtr>& targets) const;

 private:
  bool AbleToOutput(const LaneTargetConstPtr& target) const;

 private:
  bool inited_ = false;
  LaneGateKeepeParam lane_gate_keeper_param_;
};

}  // namespace environment
}  // namespace mp
}  // namespace hozon
