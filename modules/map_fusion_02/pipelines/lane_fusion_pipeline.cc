/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： lane_fusion_pipeline.cc
 *   author     ： chenlongxi
 *   date       ： 2023.09
 ******************************************************************************/

#include "modules/map_fusion_02/pipelines/lane_fusion_pipeline.h"

namespace hozon {
namespace mp {
namespace mf {

std::string LaneFusionPipeline::Name() const { return "LaneFusionPipeline"; }
bool LaneFusionPipeline::Init() {
  broken_point_search_ = std::make_unique<BrokenPointSearch>();
  broken_point_search_->Init();
  return true;
}

bool LaneFusionPipeline::Process(const ProcessOption& option,
                                 ElementMap::Ptr element_map_ptr,
                                 Group::Ptr group_ptr) {
  broken_point_search_->Process(element_map_ptr);
  return true;
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
