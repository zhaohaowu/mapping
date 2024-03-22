// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: LiGuo (liguo03@hozon.com)
// @file: lane_gatekeeper.h
// @brief: gatekeeper for lane tracker

#pragma once
#include <vector>

#include "modules/local_mapping/base/scene/stopline.h"
#include "modules/local_mapping/lib/target/base_target.h"
namespace hozon {
namespace mp {
namespace lm {

class StopLineGatekeeper {
 public:
  void Init();

  bool AbleToOutput(const StopLineTargetConstPtr& target,
                    const std::vector<StopLineTargetConstPtr>& targets) const;

 private:
  bool AbleToOutput(const StopLineTargetConstPtr& target) const;

 private:
  bool inited_ = false;
  bool use_debug_mode_ = false;
  // LaneGateKeepeParam lane_gate_keeper_param_;
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
