/******************************************************************************
 *   Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved.
 *   file       ： lane_fusion_pipeline.h
 *   author     ： chenlongxi
 *   date       ： 2023.09
 ******************************************************************************/

#pragma once

#include <yaml-cpp/yaml.h>

#include <deque>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "modules/map_fusion_02/base/element_map.h"
#include "modules/map_fusion_02/base/interface_option.h"
#include "modules/map_fusion_02/modules/lane/junction_check/junction_check.h"
#include "modules/map_fusion_02/modules/lane/path_manager.h"
#include "modules/map_fusion_02/modules/lane/road_builder/broken_point_search.h"
#include "modules/map_fusion_02/modules/lane/road_builder/road_construct.h"
#include "modules/map_fusion_02/pipelines/base_fusion_pipeline.h"
#include "modules/rviz/map_fusion_rviz.h"
#include "perception-lib/lib/config_manager/config_manager.h"

namespace hozon {
namespace mp {
namespace mf {

class LaneFusionPipeline : public BaseFusionPipeline {
 public:
  bool Init() override;
  void Clear() override;

  void InsertPose(const LocInfo::Ptr& pose);
  bool Process(const ElementMap::Ptr& element_map_ptr) const;

  std::string Name() const override;

 private:
  // 私有函数
 private:
  // 私有成员变量
  bool initialized_ = false;
  LaneFusionProcessOption options_;
  PathManagerPtr path_manager_ = nullptr;
  BrokenPointSearchPtr broken_pt_search_ = nullptr;
  RoadConstructPtr road_constructor_ = nullptr;
  JunctionCheckPtr junction_check_ = nullptr;
};

using LaneFusionPipelinePtr = std::unique_ptr<LaneFusionPipeline>;
}  // namespace mf
}  // namespace mp
}  // namespace hozon
