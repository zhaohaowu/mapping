// Copyright 2020 Hozon Inc. All Rights Reserved.
// @file: laneline_type_filter.cc
// @brief: type filter

#include "modules/local_mapping/lib/filter/laneline_type_filter.h"

#include "perception-lib/lib/config_manager/config_manager.h"

namespace hozon {
namespace mp {
namespace lm {

namespace perception_lib = hozon::perception::lib;
LaneLineTypeFilter::LaneLineTypeFilter(LaneTargetPtr lane_target)
    : BaseLaneLineTypeFilter(lane_target) {

  auto* config_manager = perception_lib::ConfigManager::Instance();
  const perception_lib::ModelConfig* model_config = nullptr;
  if (!config_manager->GetModelConfig("LanelineTypeFilterParam", &model_config)) {
    HLOG_ERROR << "Parse config failed! Name: LanelineTypeFilterParam";
  }
  if (!model_config->get_value("max_history_window", &max_history_window_)) {
    HLOG_ERROR << "Get laneline type filter max_history_window failed!";
  }
  if (!model_config->get_value("type_keep_weight", &type_keep_weight_)) {
    HLOG_ERROR << "Get laneline type filter reserve_age failed!";
  }
  if (!model_config->get_value("type_count_threshold", &type_count_threshold_)) {
    HLOG_ERROR << "Get laneline type filter reserve_age failed!";
  }
}

}  // namespace lm
}  // namespace mp
}  // namespace hozon
