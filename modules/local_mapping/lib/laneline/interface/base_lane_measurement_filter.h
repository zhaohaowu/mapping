// Copyright 2020 Hozon Inc. All Rights Reserved.
// Descriptions: lane tracker interface

#pragma once

#include <string>
#include <vector>

#include "Eigen/Eigen"
#include "modules/local_mapping/lib/laneline/interface/base_init_options.h"
#include "perception-base/base/measurement/laneline_measurement.h"
#include "perception-base/base/scene/laneline.h"
#include "perception-lib/lib/registerer/registerer.h"

namespace hozon {
namespace mp {
namespace environment {

using hozon::perception::base::LaneLinesMeasurementConstPtr;
using hozon::perception::base::LaneLinesMeasurementPtr;

struct AnomalyFilterInitOptions {};

struct AnomalyFilterOptions {
  double timestamp = 0.0;
};

class BaseLaneMeasurementFilter {
 public:
  BaseLaneMeasurementFilter() = default;

  virtual ~BaseLaneMeasurementFilter() = default;

  virtual bool Init(
      const AnomalyFilterInitOptions& options = AnomalyFilterInitOptions()) = 0;

  virtual bool Filter(const AnomalyFilterOptions& options,
                      const LaneLinesMeasurementConstPtr& input_measurements,
                      const LaneLinesMeasurementPtr& output_measurements) = 0;

  virtual std::string Name() const = 0;

  BaseLaneMeasurementFilter(const BaseLaneMeasurementFilter&) = delete;
  BaseLaneMeasurementFilter& operator=(const BaseLaneMeasurementFilter&) =
      delete;
};

PERCEPTION_REGISTER_REGISTERER(BaseLaneMeasurementFilter);
#define PERCEPTION_ENVIRONMENT_REGISTER_BASE_LANE_MEASUREMENT_FILTER(name) \
  PERCEPTION_REGISTER_CLASS(BaseLaneMeasurementFilter, name)

}  // namespace environment
}  // namespace mp
}  // namespace hozon
