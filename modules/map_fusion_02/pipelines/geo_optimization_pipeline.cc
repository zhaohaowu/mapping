/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： geo_optimization_pipeline.cc
 *   author     ： chenlongxi
 *   date       ： 2023.09
 ******************************************************************************/

#include "modules/map_fusion_02/pipelines/geo_optimization_pipeline.h"

#include "glog/logging.h"

namespace hozon {
namespace mp {
namespace mf {
bool GeoOptimizationPipeline::Init() {
  elements_filter_ = std::make_unique<ElementsFilter>();
  CHECK_EQ(elements_filter_->Init(), true);
  occ_processor_ = std::make_unique<OccProcessor>();
  CHECK_EQ(occ_processor_->Init(), true);
  return true;
}
bool GeoOptimizationPipeline::Process(const ProcessOption& option,
                                      ElementMap::Ptr element_map_ptr) {
  // 元素过滤操作
  elements_filter_->Process(element_map_ptr);
  // occ处理操作
  occ_processor_->Process(element_map_ptr);
  // 处理结束,单帧容器数据清理
  Clear();
  return true;
}

void GeoOptimizationPipeline::Clear() {
  elements_filter_->Clear();
  occ_processor_->Clear();
}

}  // namespace mf
}  // namespace mp
}  // namespace hozon
