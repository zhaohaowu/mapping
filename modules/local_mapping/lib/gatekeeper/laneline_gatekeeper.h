// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: LiGuo (liguo03@hozon.com)
// @file: lane_gatekeeper.h
// @brief: gatekeeper for lane tracker

#pragma once
#include <vector>

#include "modules/local_mapping/base/scene/laneline.h"
#include "modules/local_mapping/lib/target/base_target.h"
namespace hozon {
namespace mp {
namespace lm {

class LaneGatekeeper {
 public:
  void Init();

  bool AbleToOutput(const LaneTargetConstPtr& target,
                    const std::vector<LaneTargetConstPtr>& targets) const;

 private:
  bool AbleToOutput(const LaneTargetConstPtr& target) const;

 private:
  bool inited_ = false;
  // todo 配置化参数
  // LaneGateKeepeParam lane_gate_keeper_param_;
  float output_lane_length_ = 10.0;
  float output_blind_start_ = 50.0;
  bool output_lost_lanes_ = true;
  float output_nms_distance_ = 1.0;
  float output_nms_length_ratio_ = 0.6;
  float output_lane_longitudinal_overlap_iou_ = 0.7;
  bool use_debug_mode_ = false;
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
