/********************************************************
 * Copyright (C) 2021 HOZON-AUTO Ltd. All rights reserved.
 * Licensed Hozon
 * Author: hozon
 *******************************************************/
#pragma once

#include <iostream>
#include <string>
#include <vector>

#include "modules/local_mapping/lib/laneline/interface/base_init_options.h"
#include "perception-base/base/measurement/laneline_measurement.h"
#include "perception-base/base/scene/laneline.h"
#include "perception-base/base/utils/macros.h"
#include "perception-lib/lib/registerer/registerer.h"

namespace hozon {
namespace mp {
namespace environment {

namespace perception_base = hozon::perception::base;

class BaseLaneProcess {
 public:
  BaseLaneProcess() = default;
  virtual ~BaseLaneProcess() = default;
  /* config_file 已经是绝对路径，直接解析proto */
  virtual bool Init(const ProcessInitOption& init_option) = 0;

  virtual bool Process(
      const ProcessOption& options,
      perception_base::LaneLinesMeasurementConstPtr detect_measurements,
      const perception_base::LaneLinesPtr track_outputs) = 0;

  virtual std::string Name() const = 0;
  virtual void Reset() = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN_PECEPTION(BaseLaneProcess)
};

PERCEPTION_REGISTER_REGISTERER(BaseLaneProcess);
#define PERCEPTION_ENVIRONMENT_REGISTER_BASE_LANE_PROCESS(name) \
  PERCEPTION_REGISTER_CLASS(BaseLaneProcess, name)

}  // namespace environment
}  // namespace mp
}  // namespace hozon
