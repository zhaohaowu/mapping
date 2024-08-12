/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： lane_fusion_pipeline.h
 *   author     ： chenlongxi
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <memory>
#include <string>

#include "modules/map_fusion_02/base/element_map.h"
#include "modules/map_fusion_02/base/interface_option.h"
#include "modules/map_fusion_02/pipelines/base_fusion_pipeline.h"

namespace hozon {
namespace mp {
namespace mf {

class LaneFusionPipeline : public BaseFusionPipeline {
 public:
  typedef std::unique_ptr<LaneFusionPipeline> Ptr;

 public:
  bool Init() override;
  void Clear() override;

  bool Process(const ProcessOption& option, ElementMap::Ptr element_map_ptr,
               Group::Ptr group_ptr);

  std::string Name() const override;

 private:
  // 私有函数
 private:
  // 私有成员变量
};
}  // namespace mf
}  // namespace mp
}  // namespace hozon
