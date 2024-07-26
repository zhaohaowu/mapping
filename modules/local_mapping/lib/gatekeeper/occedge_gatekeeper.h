// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: LiGuo (liguo03@hozon.com)
// @file: occedge_gatekeeper.h
// @brief: gatekeeper for occedge tracker

#pragma once
#include <vector>

#include "modules/local_mapping/lib/target/base_target.h"

namespace hozon {
namespace mp {
namespace lm {

class OccEdgeGatekeeper {
 public:
  void Init();

  bool AbleToOutput(const OccEdgeTargetConstPtr& target,
                    const std::vector<OccEdgeTargetConstPtr>& targets) const;

 private:
  bool AbleToOutput(const OccEdgeTargetConstPtr& target) const;

 private:
  bool inited_ = false;
  float output_lane_length_ = 10.0;
  float output_blind_start_ = 50.0;
  bool output_lost_lanes_ = true;
  float output_nms_distance_ = 1.0;
  float output_nms_length_ratio_ = 0.6;
  bool use_debug_mode_ = false;
  float output_occedge_longitudinal_overlap_iou_ = 0.0;
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
