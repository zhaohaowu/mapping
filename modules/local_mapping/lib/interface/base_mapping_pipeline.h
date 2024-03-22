// Copyright 2020 Hozon Inc. All Rights Reserved.
// Descriptions: lane tracker interface

#pragma once

#include <string>
#include <vector>

#include "Eigen/Eigen"
#include "base/point/point.h"
#include "lib/registerer/registerer.h"
#include "modules/local_mapping/base/scene/localmap.h"
#include "modules/local_mapping/lib/datalogger/pose_manager.h"
#include "modules/local_mapping/lib/interface/base_init_options.h"

namespace hozon {
namespace mp {
namespace lm {

class BaseMappingPipeline {
 public:
  virtual ~BaseMappingPipeline() = default;
  virtual bool Init() = 0;

  virtual std::string Name() const = 0;

  virtual bool Process(const ProcessOption& option,
                       MeasurementFrameConstPtr measurement_ptr,
                       LocalMapFramePtr fusion_ptr) = 0;
};

}  // namespace lm
}  // namespace mp
}  // namespace hozon
