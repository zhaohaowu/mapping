// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: LiGuo (liguo03@hozon.com)
// @file: lane_gatekeeper.h
// @brief: gatekeeper for lane tracker

#pragma once
#include <vector>

#include "modules/local_mapping/base/scene/zebracrossing.h"
#include "modules/local_mapping/lib/target/base_target.h"
namespace hozon {
namespace mp {
namespace lm {

class ZebraCrossingGatekeeper {
 public:
  void Init();

  bool AbleToOutput(
      const ZebraCrossingTargetConstPtr& target,
      const std::vector<ZebraCrossingTargetConstPtr>& targets) const;

 private:
  bool AbleToOutput(const ZebraCrossingTargetConstPtr& target) const;

 private:
  bool inited_ = false;
  bool use_debug_mode_ = false;
  // ZebraCrossingGateKeepeParam lane_gate_keeper_param_;
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
