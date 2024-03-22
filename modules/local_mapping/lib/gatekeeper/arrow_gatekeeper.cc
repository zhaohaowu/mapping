// Copyright 2020 Hozon Inc. All Rights Reserved.
// @author: LiGuo (liguo03@hozon.com)
// @file: lane_gatekeeper.cc
// @brief: gatekeeper for lane tracker

#include "modules/local_mapping/lib/gatekeeper/arrow_gatekeeper.h"

#include <algorithm>

#include "perception-base/base/utils/log.h"
#include "perception-lib/lib/config_manager/config_manager.h"
namespace hozon {
namespace mp {
namespace lm {

namespace perception_lib = hozon::perception::lib;

void ArrowGatekeeper::Init() {
  const auto& config_manager = perception_lib::ConfigManager::Instance();
  const perception_lib::ModelConfig* model_config = nullptr;
  if (!config_manager->GetModelConfig("LocalMapApp", &model_config)) {
    HLOG_ERROR << "Parse config failed! Name: LaneGatekeepper";
  }

  model_config->get_value("use_debug", &use_debug_mode_);
  inited_ = true;
}

bool ArrowGatekeeper::AbleToOutput(const ArrowTargetConstPtr& target) const {
  if (target->IsTracked()) {
    return true;
  }
  // debug模式输出不成熟的元素
  if (use_debug_mode_ && !target->IsTracked()) {
    return true;
  }
  if (target->IsLost()) {
    return true;
  }

  return false;
}

bool ArrowGatekeeper::AbleToOutput(
    const ArrowTargetConstPtr& target,
    const std::vector<ArrowTargetConstPtr>& targets) const {
  if (!inited_) {
    return false;
  }
  if (!AbleToOutput(target)) {
    HLOG_DEBUG << "NOT AbleToOutput return false ";
    return false;
  }
  return true;
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
