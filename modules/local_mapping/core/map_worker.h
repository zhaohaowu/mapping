/******************************************************************************
 Copyright (C) 2023 HOZON-AUTO Ltd. All rights reserved
 *Author: luerwei/zhaohaowu
 *Date: 2023-09-01
 *****************************************************************************/
#pragma once

#include <gflags/gflags.h>

#include <Eigen/Dense>
#include <algorithm>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Sophus/se3.hpp>

#include "Eigen/src/Geometry/Transform.h"
#include "modules/local_mapping/base/scene/localmap.h"
#include "modules/local_mapping/lib/interface/base_mapping_pipeline.h"
#include "modules/local_mapping/lib/pipeline/arrow_mapping_pipeline.h"
#include "modules/local_mapping/lib/pipeline/laneline_mapping_pipeline.h"
#include "modules/local_mapping/lib/pipeline/occedge_mapping_pipeline.h"
#include "modules/local_mapping/lib/pipeline/roadedge_mapping_pipeline.h"
#include "modules/local_mapping/lib/pipeline/stopline_mapping_pipeline.h"
#include "modules/local_mapping/lib/pipeline/zebracrossing_mapping_pipeline.h"
#include "modules/local_mapping/utils/common.h"
#include "modules/util/include/util/mapping_log.h"
#include "modules/util/include/util/rviz_agent/rviz_agent.h"
namespace hozon {
namespace mp {
namespace lm {

class MapWorker {
 public:
  MapWorker() = default;

  bool Init();

  // 生成localmap的整个处理逻辑。
  bool Process(const MeasurementFrameConstPtr& measurement_frame_ptr);

 private:
  // localmap的数据裁剪
  void CutLocalMap(std::shared_ptr<LocalMapFrame> local_map_ptr,
                   const double& length_x, const double& length_y);

  // 清空地图数据
  void ClearLocalMap(const LocalMapFramePtr& local_map_ptr_);

 private:
  // 车道线的localmap管理器
  std::unique_ptr<BaseMappingPipeline> laneline_mapping_pl_ptr_;
  // 路沿的localmap管理器
  std::unique_ptr<BaseMappingPipeline> roadedge_mapping_pl_ptr_;
  // 箭头的localmap管理器
  std::unique_ptr<BaseMappingPipeline> arrow_mapping_pl_ptr_;
  // 斑马线的localmap管理器
  std::unique_ptr<BaseMappingPipeline> zebracrossing_mapping_pl_ptr_;
  // 停止线的localmap管理器
  std::unique_ptr<BaseMappingPipeline> stopline_mapping_pl_ptr_;
  // occ路沿线的localmap管理器
  std::unique_ptr<BaseMappingPipeline> occedge_mapping_pl_ptr_;
  bool static_copy_map_once_ = false;
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
